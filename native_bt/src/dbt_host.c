#include "dbt_host.h"

/* Bluetooth HCI opcodes and return-parameter lengths, including Status.
 * These are standard wire values, not a third-party command implementation. */
static const struct { uint16_t opcode; uint8_t result_bytes; } probes[] = {
    { 0x0c03, 1 },  /* Reset */
    { 0x1001, 9 },  /* Read Local Version Information */
    { 0x1002, 65 }, /* Read Local Supported Commands */
    { 0x1003, 9 },  /* Read Local Supported Features, page 0 */
    { 0x1009, 7 },  /* Read BD_ADDR */
    { 0x1005, 8 },  /* Read Buffer Size */
    { 0x2003, 9 },  /* LE Read Local Supported Features */
    { 0x2002, 4 }   /* LE Read Buffer Size, v1 */
};
#define PROBE_COUNT (sizeof(probes) / sizeof(probes[0]))
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && sizeof(unsigned int)==sizeof(uint32_t),
               "Command tokens need lock-free 32-bit atomics");
/* Survives host reinitialization. Saturate instead of allowing an old token to
 * alias a command after wrap or on another host. */
static _Atomic uint32_t command_token_serial;

static void copy_bytes(uint8_t *destination, const uint8_t *source, uint16_t size) {
    for (uint16_t i = 0; i < size; ++i) destination[i] = source[i];
}

static bool due(uint32_t now, uint32_t deadline) {
    return (uint32_t)(now - deadline) < 0x80000000u;
}
static void wipe(void *data, unsigned size) {
    volatile uint8_t *bytes=data;for(unsigned i=0;i<size;++i)bytes[i]=0;
}
static bool sensitive(const dbt_hci_packet_t *packet) {
    /* ACL may contain SMP keys in a first or continuation fragment. Avoid
     * cross-callback classification state: every ACL ingress is sensitive. */
    return packet->type==DBT_H4_ACL ||
        (packet->type==DBT_H4_EVENT && (packet->event==0x18 ||
         (packet->event==0x3e && packet->size && packet->payload[0]==9)));
}

/* Owner only, after borrowed RX callbacks return. Each queue_pop wipes a
 * sensitive record before publishing the freed SPSC range to the producer. */
static void discard_ingress(dbt_host_t *host) {
    for (unsigned i=0;i<DBT_QUEUE_BYTES/3u;i++) {
        uint16_t size=0;
        dbt_result_t rc=dbt_queue_pop(&host->ingress,host->rx_packet,sizeof(host->rx_packet),&size);
        if (rc==DBT_EMPTY) break;
        if (rc!=DBT_OK) {
            /* On a corrupt record, discard only the already-published range.
             * Producer-owned uncommitted bytes are never touched. */
            uint32_t read=atomic_load_explicit(&host->ingress.read,memory_order_relaxed);
            uint32_t written=atomic_load_explicit(&host->ingress.written,memory_order_acquire);
            uint32_t used=written-read;
            if (used<=DBT_QUEUE_BYTES) {
                volatile uint8_t *bytes=host->ingress.bytes;
                for(uint32_t j=0;j<used;j++) bytes[(read+j)&(DBT_QUEUE_BYTES-1u)]=0;
                atomic_store_explicit(&host->ingress.read,written,memory_order_release);
            }
            break;
        }
        wipe(host->rx_packet,size);
    }
    wipe(host->rx_packet,sizeof(host->rx_packet));
}
static void fail(dbt_host_t *host, dbt_fault_t fault) {
    if (host->state != DBT_HOST_FAILED) {
        host->fault = fault;
        host->state = DBT_HOST_FAILED;
    }
    uint32_t empty=DBT_FAULT_NONE;
    atomic_compare_exchange_strong_explicit(&host->ingress_fault,&empty,fault,
                                             memory_order_release,memory_order_relaxed);
    while (host->command_count) {
        dbt_command_t *command = &host->commands[host->command_head];
        dbt_command_done_t done = command->done;
        void *context = command->context;
        uint16_t opcode = command->opcode;
        wipe(command->parameters,sizeof(command->parameters));
        host->command_head = (uint8_t)((host->command_head + 1u) % DBT_COMMAND_SLOTS);
        --host->command_count;
        if (done) done(context, opcode, DBT_COMMAND_ABORTED, 0, 0, 0);
    }
    if (!host->in_poll) discard_ingress(host);
}

void dbt_host_abort(dbt_host_t *host, dbt_fault_t fault) {
    if (host && fault != DBT_FAULT_NONE) fail(host, fault);
}

static uint32_t allocate_token(void) {
    uint32_t previous=atomic_load_explicit(&command_token_serial,memory_order_relaxed);
    do {
        if(previous==UINT32_MAX)return 0;
    } while(!atomic_compare_exchange_weak_explicit(&command_token_serial,&previous,previous+1u,
                                                   memory_order_relaxed,memory_order_relaxed));
    return previous+1u;
}
static dbt_result_t enqueue_command(dbt_host_t *host, uint16_t opcode,
    const uint8_t *parameters, uint8_t size, dbt_command_response_t response,
    dbt_command_done_t done, void *context, uint32_t now_ms, uint32_t *token) {
    if (!host || !opcode || (size && !parameters) ||
        (response != DBT_COMMAND_COMPLETE && response != DBT_COMMAND_STATUS)) return DBT_INVALID;
    if(opcode==0x0c35)return DBT_UNSUPPORTED; /* Special no-credit/no-response RX ack. */
    if (host->state != DBT_HOST_IDENTIFIED) return DBT_BUSY;
    if (host->command_count == DBT_COMMAND_SLOTS) return DBT_NO_SPACE;
    uint32_t assigned=token?allocate_token():0;
    if(token && !assigned)return DBT_NO_SPACE;
    dbt_command_t *command = &host->commands[(host->command_head + host->command_count) % DBT_COMMAND_SLOTS];
    command->opcode = opcode; command->size = size; command->response = response;
    command->done = done; command->context = context;
    command->deadline = now_ms + host->timeout_ms;
    command->token = assigned;
    copy_bytes(command->parameters, parameters, size);
    ++host->command_count;
    if(token)*token=assigned;
    return DBT_OK;
}
dbt_result_t dbt_host_command(dbt_host_t *host,uint16_t opcode,const uint8_t *parameters,uint8_t size,
    dbt_command_response_t response,dbt_command_done_t done,void *context,uint32_t now_ms) {
    return enqueue_command(host,opcode,parameters,size,response,done,context,now_ms,0);
}
dbt_result_t dbt_host_command_tracked(dbt_host_t *host,uint16_t opcode,const uint8_t *parameters,uint8_t size,
    dbt_command_response_t response,dbt_command_done_t done,void *context,uint32_t now_ms,uint32_t *token) {
    if(!token)return DBT_INVALID;
    return enqueue_command(host,opcode,parameters,size,response,done,context,now_ms,token);
}
dbt_command_cancel_result_t dbt_host_cancel_command(dbt_host_t *host,uint32_t token) {
    if(!host || !token)return DBT_COMMAND_NOT_FOUND;
    for(unsigned offset=0;offset<host->command_count;++offset){
        unsigned index=(host->command_head+offset)%DBT_COMMAND_SLOTS;
        dbt_command_t *command=&host->commands[index];
        if(command->token!=token)continue;
        if(offset==0 && host->pending_opcode)return DBT_COMMAND_ALREADY_SUBMITTED;
        dbt_command_done_t done=command->done;void *context=command->context;uint16_t opcode=command->opcode;
        wipe(command,sizeof(*command));
        for(unsigned i=offset;i+1u<host->command_count;++i){
            dbt_command_t *dst=&host->commands[(host->command_head+i)%DBT_COMMAND_SLOTS];
            dbt_command_t *src=&host->commands[(host->command_head+i+1u)%DBT_COMMAND_SLOTS];
            copy_bytes((uint8_t *)dst,(const uint8_t *)src,(uint16_t)sizeof(*dst));
            wipe(src,sizeof(*src));
        }
        --host->command_count;
        if(done)done(context,opcode,DBT_COMMAND_ABORTED,0,0,0);
        return DBT_COMMAND_CANCELLED;
    }
    return DBT_COMMAND_NOT_FOUND;
}

static void command_done(dbt_host_t *host, dbt_command_outcome_t outcome,
                          uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_command_t *command = &host->commands[host->command_head];
    dbt_command_done_t done = command->done;
    void *context = command->context;
    uint16_t opcode = command->opcode;
    wipe(command->parameters,sizeof(command->parameters));
    host->command_head = (uint8_t)((host->command_head + 1u) % DBT_COMMAND_SLOTS);
    --host->command_count;
    host->pending_opcode = 0;
    if (done) done(context, opcode, outcome, status, returns, size);
}

dbt_result_t dbt_host_init(dbt_host_t *host, dbt_transport_t transport,
                          dbt_observer_t observer, void *context, uint32_t timeout_ms) {
    if (host == 0 || transport.can_send == 0 || transport.send == 0 ||
        timeout_ms == 0 || timeout_ms >= 0x80000000u) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)host;
    for (uint32_t i = 0; i < sizeof(*host); ++i) bytes[i] = 0;
    dbt_queue_init(&host->ingress);
    atomic_init(&host->ingress_fault, DBT_FAULT_NONE);
    host->transport = transport;
    host->observer = observer;
    host->observer_context = context;
    host->timeout_ms = timeout_ms;
    return DBT_OK;
}

dbt_result_t dbt_host_start(dbt_host_t *host, uint32_t now_ms) {
    if (host == 0 || host->state != DBT_HOST_IDLE) return DBT_INVALID;
    host->state = DBT_HOST_PROBING;
    host->command_credits = 1; /* HCI permits an initial command before first event. */
    host->deadline_ms = now_ms + host->timeout_ms;
    return DBT_OK;
}

dbt_result_t dbt_host_receive(dbt_host_t *host, const uint8_t *data, uint16_t size) {
    if (!host) return DBT_INVALID;
    dbt_hci_packet_t packet;
    /* A terminal ingress/host fault discards subsequent packets. Preserve the
     * decoder's result for late valid HCI events without retaining payloads. */
    if (atomic_load_explicit(&host->ingress_fault,memory_order_acquire)!=DBT_FAULT_NONE)
        return dbt_hci_decode(data,size,&packet);
    dbt_result_t result = dbt_hci_decode(data, size, &packet);
    dbt_fault_t fault = DBT_FAULT_NONE;
    if (result != DBT_OK) {
        fault = result == DBT_UNSUPPORTED ? DBT_FAULT_UNSUPPORTED_PACKET : DBT_FAULT_BAD_PACKET;
    } else {
        bool reserve=packet.type == DBT_H4_ACL || packet.type == DBT_H4_SCO;
        result = sensitive(&packet) ? dbt_queue_push_sensitive(&host->ingress,data,size,reserve) :
            dbt_queue_push(&host->ingress, data, size,reserve);
        if (result != DBT_OK) fault = DBT_FAULT_RX_OVERRUN;
    }
    if (fault != DBT_FAULT_NONE) {
        uint32_t empty = DBT_FAULT_NONE;
        atomic_compare_exchange_strong_explicit(&host->ingress_fault, &empty, fault,
                                                memory_order_release, memory_order_relaxed);
    }
    return result;
}

static void capture_result(dbt_host_t *host, const uint8_t *data) {
    dbt_controller_info_t *info = &host->controller;
    switch (host->probe_index) {
    case 1:
        info->hci_version = data[1];
        info->hci_revision = dbt_read_le16(data + 2);
        info->lmp_version = data[4];
        info->manufacturer = dbt_read_le16(data + 5);
        info->lmp_subversion = dbt_read_le16(data + 7);
        break;
    case 2: copy_bytes(info->supported_commands, data + 1, 64); break;
    case 3:
        copy_bytes(info->features, data + 1, 8);
        /* Feature bit 37: BR/EDR Not Supported; bit 38: LE Supported. */
        if ((info->features[4] & 0x20u) != 0 || (info->features[4] & 0x40u) == 0) {
            fail(host, DBT_FAULT_NOT_DUAL_MODE);
        }
        break;
    case 4: copy_bytes(info->address_le, data + 1, 6); break;
    case 5:
        info->classic_acl_bytes = dbt_read_le16(data + 1);
        info->classic_acl_packets = dbt_read_le16(data + 4);
        if (info->classic_acl_bytes == 0 || info->classic_acl_packets == 0) {
            fail(host, DBT_FAULT_BAD_RESPONSE);
        }
        break;
    case 6: copy_bytes(info->le_features, data + 1, 8); break;
    case 7:
        info->le_acl_bytes = dbt_read_le16(data + 1);
        info->le_acl_packets = data[3];
        info->shared_acl_pool = info->le_acl_packets == 0;
        if (info->shared_acl_pool) {
            info->le_acl_bytes = info->classic_acl_bytes;
            info->le_acl_packets = info->classic_acl_packets;
        } else if (info->le_acl_bytes == 0) {
            fail(host, DBT_FAULT_BAD_RESPONSE);
        }
        break;
    default: break;
    }
}

static void handle_event(dbt_host_t *host, const dbt_hci_packet_t *packet, uint32_t now_ms) {
    const uint8_t *data = packet->payload;
    if (packet->event == DBT_EVT_HARDWARE_ERROR) {
        if (packet->size != 1) { fail(host, DBT_FAULT_BAD_PACKET); return; }
        host->controller_status = data[0];
        fail(host, DBT_FAULT_CONTROLLER);
        return;
    }
    bool complete = packet->event == DBT_EVT_COMMAND_COMPLETE;
    bool status = packet->event == DBT_EVT_COMMAND_STATUS;
    if (!complete && !status) return;
    if ((complete && packet->size < 3) || (status && packet->size != 4)) {
        fail(host, DBT_FAULT_BAD_PACKET);
        return;
    }
    host->command_credits = complete ? data[0] : data[1];
    uint16_t opcode = dbt_read_le16(data + (complete ? 1 : 2));
    if (host->pending_opcode == 0 || opcode != host->pending_opcode) {
        host->unexpected_completions++;
        return;
    }
    if (host->state == DBT_HOST_IDENTIFIED) {
        if (!host->command_count) { fail(host, DBT_FAULT_BAD_RESPONSE); return; }
        dbt_command_t *command = &host->commands[host->command_head];
        if (status && command->response == DBT_COMMAND_COMPLETE && data[0] == 0) return;
        if ((complete && command->response != DBT_COMMAND_COMPLETE) ||
            (complete && packet->size < 4)) { fail(host, DBT_FAULT_BAD_RESPONSE); return; }
        uint8_t result_status = complete ? data[3] : data[0];
        host->controller_status = result_status;
        command_done(host, result_status ? DBT_COMMAND_REJECTED : DBT_COMMAND_SUCCESS,
                     result_status, complete ? data + 3 : data,
                     complete ? (uint8_t)(packet->size - 3u) : 1);
        return;
    }
    if (status) {
        /* Command Status acknowledges admission, never successful completion. */
        if (data[0] != 0) {
            host->controller_status = data[0];
            fail(host, DBT_FAULT_COMMAND_REJECTED);
        }
        return;
    }
    if (packet->size < 4) { fail(host, DBT_FAULT_BAD_RESPONSE); return; }
    host->controller_status = data[3];
    if (data[3] != 0) { fail(host, DBT_FAULT_COMMAND_REJECTED); return; }
    if (host->probe_index >= PROBE_COUNT ||
        packet->size != (uint16_t)(3u + probes[host->probe_index].result_bytes)) {
        fail(host, DBT_FAULT_BAD_RESPONSE);
        return;
    }
    capture_result(host, data + 3);
    if (host->state == DBT_HOST_FAILED) return;
    host->pending_opcode = 0;
    host->probe_index++;
    host->deadline_ms = now_ms + host->timeout_ms;
    if (host->probe_index == PROBE_COUNT) host->state = DBT_HOST_IDENTIFIED;
}

uint16_t dbt_host_poll(dbt_host_t *host, uint32_t now_ms, uint16_t rx_budget) {
    uint16_t processed = 0;
    if (!host || host->in_poll) return 0;
    if(host->tx_retained && host->transport.can_send(host->transport.context)) {
        wipe(host->tx_command,sizeof(host->tx_command));host->tx_retained=false;
    }
    if(host->state == DBT_HOST_FAILED) { discard_ingress(host); return 0; }
    if(host->state == DBT_HOST_IDLE)return 0;
    dbt_fault_t ingress_fault = (dbt_fault_t)atomic_load_explicit(&host->ingress_fault, memory_order_acquire);
    if (ingress_fault != DBT_FAULT_NONE) { host->fault_stage=1;fail(host, ingress_fault); return 0; }
    if (host->state == DBT_HOST_PROBING && due(now_ms, host->deadline_ms)) {
        fail(host, DBT_FAULT_TIMEOUT);
        return 0;
    }
    host->in_poll = true;
    while (processed < rx_budget) {
        uint16_t size = 0;
        dbt_result_t result = dbt_queue_pop(&host->ingress, host->rx_packet, sizeof(host->rx_packet), &size);
        if (result == DBT_EMPTY) break;
        if (result != DBT_OK) { host->fault_stage=2;fail(host, DBT_FAULT_BAD_PACKET); break; }
        dbt_hci_packet_t packet;
        if (dbt_hci_decode(host->rx_packet, size, &packet) != DBT_OK) {
            host->fault_stage=3;host->fault_packet_size=size;
            host->fault_packet_type=size?host->rx_packet[0]:0;
            host->fault_event=size>=2 && host->rx_packet[0]==DBT_H4_EVENT?host->rx_packet[1]:0;
            wipe(host->rx_packet,sizeof(host->rx_packet));
            fail(host, DBT_FAULT_BAD_PACKET);
            break;
        }
        ++processed;
        ++host->packets_processed;
        if (packet.type == DBT_H4_EVENT) handle_event(host, &packet, now_ms);
        bool event_failed=host->state==DBT_HOST_FAILED;
        if (host->observer != 0) host->observer(host->observer_context, &packet);
        if(sensitive(&packet))wipe(host->rx_packet,size);
        if (host->state == DBT_HOST_FAILED){
            host->fault_stage=event_failed?4:5;host->fault_packet_size=size;
            host->fault_packet_type=packet.type;host->fault_event=packet.event;
            break;
        }
    }
    if (host->state == DBT_HOST_PROBING && host->pending_opcode == 0 &&
        host->command_credits != 0 && host->transport.can_send(host->transport.context)) {
        uint16_t size = 0;
        uint16_t opcode = probes[host->probe_index].opcode;
        wipe(host->tx_command,sizeof(host->tx_command));
        if (dbt_hci_command(opcode, 0, 0, host->tx_command, sizeof(host->tx_command), &size) != DBT_OK) {
            fail(host, DBT_FAULT_BAD_RESPONSE);
        } else if (host->transport.send(host->transport.context, host->tx_command, size)) {
            host->tx_retained=true;
            host->pending_opcode = opcode;
            --host->command_credits;
            ++host->commands_submitted;
            host->deadline_ms = now_ms + host->timeout_ms;
        } else wipe(host->tx_command,sizeof(host->tx_command));
    }
    if (host->state == DBT_HOST_IDENTIFIED && host->command_count) {
        dbt_command_t *command = &host->commands[host->command_head];
        if (due(now_ms, command->deadline)) {
            if (host->pending_opcode) {
                /* Fail before the timeout callback, so it cannot queue new work. */
                host->state = DBT_HOST_FAILED; host->fault = DBT_FAULT_TIMEOUT;
                command_done(host, DBT_COMMAND_TIMEOUT, 0, 0, 0);
                fail(host, DBT_FAULT_TIMEOUT);
            } else command_done(host, DBT_COMMAND_TIMEOUT, 0, 0, 0);
        } else if (!host->pending_opcode && host->command_credits &&
                   host->transport.can_send(host->transport.context)) {
            uint16_t size = 0;
            wipe(host->tx_command,sizeof(host->tx_command));
            (void)dbt_hci_command(command->opcode, command->parameters, command->size,
                                 host->tx_command, sizeof(host->tx_command), &size);
            if (host->transport.send(host->transport.context, host->tx_command, size)) {
                host->tx_retained=true;
                wipe(command->parameters,sizeof(command->parameters));
                host->pending_opcode = command->opcode;
                --host->command_credits; ++host->commands_submitted;
                command->deadline = now_ms + host->timeout_ms;
            } else wipe(host->tx_command,sizeof(host->tx_command));
        }
    }
    if(host->state==DBT_HOST_FAILED) discard_ingress(host);
    host->in_poll = false;
    return processed;
}

uint32_t dbt_host_wait_ms(const dbt_host_t *host, uint32_t now_ms) {
    if (host->state == DBT_HOST_FAILED) return dbt_queue_used(&host->ingress) ? 0 : UINT32_MAX;
    if (host->state == DBT_HOST_IDLE) return UINT32_MAX;
    if (atomic_load_explicit(&host->ingress_fault, memory_order_acquire) != DBT_FAULT_NONE ||
        dbt_queue_used(&host->ingress) != 0) return 0;
    if (host->state == DBT_HOST_IDENTIFIED) {
        if (!host->command_count) return UINT32_MAX;
        if (!host->pending_opcode && host->command_credits && host->transport.can_send(host->transport.context)) return 0;
        uint32_t deadline = host->commands[host->command_head].deadline;
        return due(now_ms, deadline) ? 0 : deadline - now_ms;
    }
    if (host->state != DBT_HOST_PROBING) return UINT32_MAX;
    if (due(now_ms, host->deadline_ms)) return 0;
    return host->deadline_ms - now_ms;
}
