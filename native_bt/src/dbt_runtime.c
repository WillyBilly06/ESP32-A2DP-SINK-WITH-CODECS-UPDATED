#include "dbt_runtime.h"

static bool remember_failure(dbt_runtime_t *runtime,dbt_runtime_fault_origin_t origin,
    dbt_result_t result,const dbt_hci_packet_t *packet,uint16_t handle,uint16_t cid,uint16_t bytes){
    if(runtime->failure.origin!=DBT_RUNTIME_FAULT_NONE)return false;
    const dbt_link_t *link=dbt_acl_find(&runtime->acl,handle);
    dbt_runtime_failure_t *f=&runtime->failure;
    f->at_ms=runtime->now_ms;f->origin=(uint8_t)origin;f->result=(uint8_t)result;
    f->handle=handle;f->cid=cid;f->bytes=bytes;f->link_present=link!=0;
    if(link)f->generation=link->generation;
    if(packet){f->packet_type=packet->type;f->event=packet->event;f->pb=packet->pb;f->bc=packet->bc;}
    return true;
}

static void finish_tx(dbt_runtime_tx_t *tx, dbt_tx_outcome_t outcome) {
    dbt_tx_done_t done = tx->done;
    void *context = tx->context;
    tx->active = false;
    if (done) done(context, outcome);
}
static uint8_t *allocate(void *context, uint16_t bytes) {
    dbt_runtime_t *runtime = context;
    return runtime->hooks.allocate(runtime->hooks.context, bytes);
}
static void release(void *context, uint8_t *data) {
    dbt_runtime_t *runtime = context;
    runtime->hooks.release(runtime->hooks.context, data);
}
static void pdu(void *context, uint16_t handle, uint32_t generation,
                 uint16_t cid, const uint8_t *data, uint16_t size) {
    dbt_runtime_t *runtime = context;
    dbt_result_t result = dbt_channels_receive(&runtime->channels, handle, generation, cid, data, size);
    /* VHCI does not offer reliable ingress retry. Never silently discard a
     * signaling request after inbox/allocator exhaustion. Recovery is explicit. */
    if (result != DBT_OK && result != DBT_UNSUPPORTED){
        if(remember_failure(runtime,DBT_RUNTIME_FAULT_CHANNEL_PDU,result,0,handle,cid,size)){
            runtime->failure.generation=generation;
            if((cid==1 || cid==5) && size>=4){
                runtime->failure.code=data[0];runtime->failure.identifier=data[1];
                runtime->failure.declared=dbt_read_le16(data+2);
            }
        }
        dbt_host_abort(&runtime->host, result == DBT_NO_SPACE ? DBT_FAULT_RX_OVERRUN : DBT_FAULT_BAD_PACKET);
    }
}
static bool authorize(void *context, const dbt_link_t *link, uint16_t psm, dbt_security_t security) {
    dbt_runtime_t *runtime = context;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    return hooks->authorize ? hooks->authorize(hooks->context, link, psm, security) : security != DBT_SECURITY_AUTHENTICATED;
}
static bool tx_idle(void *context, uint16_t handle, uint16_t local_cid) {
    dbt_runtime_t *runtime = context;
    const dbt_link_t *link = dbt_acl_find(&runtime->acl, handle);
    /* Conservatively wait for all controller packets on this connection. */
    if (!link || link->outstanding) return false;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (runtime->tx[i].active && runtime->tx[i].pdu.handle == handle) return false;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    return !hooks->tx_idle || hooks->tx_idle(hooks->context, handle, local_cid);
}
static dbt_result_t request_security(void *context,const dbt_link_t *link,uint16_t psm,dbt_security_t security,bool start) {
    dbt_runtime_t *runtime=context;dbt_channel_hooks_t *hooks=&runtime->hooks.channels;
    return hooks->security_request?hooks->security_request(hooks->context,link,psm,security,start):DBT_UNSUPPORTED;
}
static bool accept_channel(void *context,const dbt_link_t *link,uint16_t psm) {
    dbt_runtime_t *runtime=context;dbt_channel_hooks_t *hooks=&runtime->hooks.channels;
    return !hooks->accept || hooks->accept(hooks->context,link,psm);
}
static uint16_t qualify_le_security(void *context,const dbt_link_t *link,uint16_t spsm,dbt_security_t security,uint8_t minimum){
    dbt_runtime_t *runtime=context;dbt_channel_hooks_t *h=&runtime->hooks.channels;
    return h->le_security?h->le_security(h->context,link,spsm,security,minimum):security==DBT_SECURITY_AUTHENTICATED?5:minimum?7:0;
}
static void event(void *context, dbt_channel_event_t kind, const dbt_channel_t *channel,
                   dbt_channel_reason_t reason, uint16_t result) {
    dbt_runtime_t *runtime = context;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    if(kind == DBT_CH_EVENT_CLOSED || kind == DBT_CH_EVENT_FAILED) {
        for(unsigned i=0;i<DBT_MAX_LINKS;++i) {
            dbt_runtime_ertm_tx_t *frame=&runtime->ertm_tx[i];
            if(frame->active && frame->pdu.handle==channel->handle && frame->pdu.generation==channel->link_generation &&
               frame->local_cid==channel->local_cid)frame->active=false;
            dbt_runtime_tx_t *tx=&runtime->tx[i];
            if(tx->active && tx->pdu.handle==channel->handle && tx->pdu.generation==channel->link_generation &&
                tx->local_cid==channel->local_cid) finish_tx(tx,reason==DBT_CH_REASON_LINK_LOST?DBT_TX_LINK_LOST:DBT_TX_ABORTED);
        }
    }
    if (hooks->event) hooks->event(hooks->context, kind, channel, reason, result);
}
static void data(void *context, const dbt_channel_t *channel, const uint8_t *bytes, uint16_t size) {
    dbt_runtime_t *runtime = context;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    if (hooks->data) hooks->data(hooks->context, channel, bytes, size);
}
static void fixed(void *context, const dbt_link_t *link, uint16_t cid, const uint8_t *bytes, uint16_t size) {
    dbt_runtime_t *runtime = context;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    if (hooks->fixed_data) hooks->fixed_data(hooks->context, link, cid, bytes, size);
}
static bool parameters(void *context, const dbt_link_t *link, uint16_t minimum,
                         uint16_t maximum, uint16_t latency, uint16_t timeout) {
    dbt_runtime_t *runtime = context;
    dbt_channel_hooks_t *hooks = &runtime->hooks.channels;
    return hooks->le_parameters && hooks->le_parameters(hooks->context, link, minimum, maximum, latency, timeout);
}
static bool initialize_layers(dbt_runtime_t *runtime) {
    if (runtime->ready) return true;
    if (runtime->host.state != DBT_HOST_IDENTIFIED) return false;
    dbt_channel_hooks_t hooks = {0};
    hooks.context = runtime; hooks.authorize = authorize; hooks.tx_idle = tx_idle;
    hooks.security_request=request_security;
    hooks.accept=accept_channel;
    hooks.event = event; hooks.data = data; hooks.fixed_data = fixed; hooks.le_parameters = parameters;
    hooks.allocate=allocate;hooks.release=release;hooks.le_security=qualify_le_security;
    if (dbt_acl_init(&runtime->acl, &runtime->host.controller) != DBT_OK ||
        dbt_channels_init(&runtime->channels, &runtime->acl, hooks, runtime->maximum_mtu, 1000, 60000) != DBT_OK) {
        dbt_host_abort(&runtime->host, DBT_FAULT_BAD_RESPONSE);
        return false;
    }
    /* Controller ACL MTUs may exceed this host's ingress bound. TX fragments
     * are legal at a smaller size; RX requires a compatible controller setting. */
    for (unsigned i = 0; i < 2; ++i)
        if (runtime->acl.mtu[i] > DBT_MAX_H4_BYTES - 5u) runtime->acl.mtu[i] = DBT_MAX_H4_BYTES - 5u;
    runtime->ready = true;
    return true;
}
static void packet(void *context, const dbt_hci_packet_t *view) {
    dbt_runtime_t *runtime = context;
    if (initialize_layers(runtime)) {
        if (view->type == DBT_H4_EVENT) {
            bool failed_before_flow=runtime->host.state==DBT_HOST_FAILED;
            dbt_rx_flow_event(&runtime->rx_flow,view);
            if(!failed_before_flow && runtime->host.state==DBT_HOST_FAILED)
                remember_failure(runtime,DBT_RUNTIME_FAULT_RX_FLOW,DBT_INVALID,view,view->handle,0,view->size);
            dbt_link_change_t change;
            dbt_result_t result = dbt_acl_event(&runtime->acl, view, &change);
            if (result != DBT_OK && result != DBT_UNSUPPORTED){
                remember_failure(runtime,DBT_RUNTIME_FAULT_ACL_EVENT,result,view,view->handle,0,view->size);
                dbt_host_abort(&runtime->host, DBT_FAULT_BAD_PACKET);
            }
            if (result == DBT_OK && change.kind != DBT_LINK_UNCHANGED) {
                dbt_l2cap_link_change(&runtime->rx, &change);
                dbt_channels_link_change(&runtime->channels, &change);
                dbt_rx_flow_link_change(&runtime->rx_flow,&change);
                if (change.kind == DBT_LINK_REMOVED) {
                    if (runtime->control_active && runtime->control.handle == change.handle &&
                        runtime->control.generation == change.generation) {
                        (void)dbt_channels_tx_cancel(&runtime->channels);
                        runtime->control_active = false;
                    }
                    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
                        dbt_runtime_tx_t *tx = &runtime->tx[i];
                        if (tx->active && tx->pdu.handle == change.handle && tx->pdu.generation == change.generation)
                            finish_tx(tx, DBT_TX_LINK_LOST);
                    }
                }
                if (runtime->hooks.link) runtime->hooks.link(runtime->hooks.context, &change);
            }
        } else if (view->type == DBT_H4_ACL) {
            dbt_result_t result = dbt_l2cap_receive(&runtime->rx, &runtime->acl, view, runtime->now_ms);
            if (result != DBT_OK){
                bool first=remember_failure(runtime,DBT_RUNTIME_FAULT_L2CAP,result,view,view->handle,0,view->size);
                if(first && view->pb==2 && view->size>=4){
                    runtime->failure.cid=dbt_read_le16(view->payload+2);
                    runtime->failure.declared=dbt_read_le16(view->payload);
                }
                dbt_host_abort(&runtime->host,result == DBT_NO_SPACE ? DBT_FAULT_RX_OVERRUN : DBT_FAULT_BAD_PACKET);
            }
        } else if(view->type==DBT_H4_SCO) {
            if(runtime->hooks.sco)runtime->hooks.sco(runtime->hooks.context,view);
            else dbt_host_abort(&runtime->host,DBT_FAULT_UNSUPPORTED_PACKET);
        }
    }
    if (runtime->hooks.packet) runtime->hooks.packet(runtime->hooks.context, view);
    if(view->type==DBT_H4_ACL && runtime->host.state==DBT_HOST_IDENTIFIED &&
       runtime->rx_flow.state==DBT_RX_FLOW_ACTIVE) {
        const dbt_link_t *link=dbt_acl_find(&runtime->acl,view->handle);
        dbt_result_t result=dbt_rx_flow_completed(&runtime->rx_flow,link);
        if(result!=DBT_OK){
            remember_failure(runtime,DBT_RUNTIME_FAULT_RX_FLOW,result,view,view->handle,0,view->size);
            dbt_host_abort(&runtime->host,result==DBT_NO_SPACE?DBT_FAULT_RX_OVERRUN:DBT_FAULT_BAD_PACKET);
        }
    }
}

dbt_result_t dbt_runtime_init(dbt_runtime_t *runtime, dbt_transport_t transport,
    dbt_runtime_hooks_t hooks, uint16_t maximum_mtu, uint32_t timeout_ms) {
    if (!runtime || !hooks.allocate || !hooks.release || maximum_mtu < 48) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)runtime;
    for (unsigned i = 0; i < sizeof(*runtime); ++i) bytes[i] = 0;
    const uint8_t *hook_bytes=(const uint8_t *)&hooks;uint8_t *saved_hooks=(uint8_t *)&runtime->hooks;
    for(unsigned i=0;i<sizeof(hooks);++i)saved_hooks[i]=hook_bytes[i];
    runtime->maximum_mtu = maximum_mtu;
    dbt_l2cap_hooks_t rx_hooks = {runtime, allocate, release, pdu};
    dbt_result_t result = dbt_l2cap_rx_init(&runtime->rx, rx_hooks, maximum_mtu, 1000);
    if (result != DBT_OK) return result;
    result=dbt_rx_flow_init(&runtime->rx_flow,&runtime->host,&runtime->acl);
    if(result!=DBT_OK)return result;
    return dbt_host_init(&runtime->host, transport, packet, runtime, timeout_ms);
}
dbt_result_t dbt_runtime_start(dbt_runtime_t *runtime, uint32_t now_ms) {
    if (!runtime) return DBT_INVALID;
    runtime->now_ms = now_ms;
    return dbt_host_start(&runtime->host, now_ms);
}
dbt_result_t dbt_runtime_enable_rx_flow(dbt_runtime_t *runtime,uint16_t packet_bytes,uint16_t packet_count) {
    if(!runtime)return DBT_INVALID;
    if(!runtime->ready)return DBT_BUSY;
    return dbt_rx_flow_start(&runtime->rx_flow,packet_bytes,packet_count);
}
dbt_result_t dbt_runtime_receive(dbt_runtime_t *runtime, const uint8_t *data, uint16_t size) {
    if (!runtime) return DBT_INVALID;
    return dbt_host_receive(&runtime->host, data, size);
}
void dbt_runtime_abort(dbt_runtime_t *runtime, dbt_fault_t fault) {
    if (!runtime || fault == DBT_FAULT_NONE) return;
    dbt_host_abort(&runtime->host, fault);
    dbt_rx_flow_abort(&runtime->rx_flow);
    if (runtime->cleared) return;
    runtime->cleared = true; runtime->ready = false;
    dbt_l2cap_rx_clear(&runtime->rx);
    if (runtime->control_active) (void)dbt_channels_tx_cancel(&runtime->channels);
    runtime->control_active = false;
    dbt_channels_clear(&runtime->channels);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (runtime->tx[i].active) finish_tx(&runtime->tx[i], DBT_TX_ABORTED);
}
static dbt_runtime_tx_t *slot(dbt_runtime_t *runtime, uint16_t handle) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (!runtime->acl.links[i].used || runtime->acl.links[i].handle != handle) continue;
        return &runtime->tx[i];
    }
    return 0;
}
static dbt_result_t send_pdu(dbt_runtime_t *runtime, uint16_t handle, uint16_t cid,
    const uint8_t *data, uint16_t size, dbt_tx_done_t done, void *context, bool fixed_channel) {
    if (!runtime || !runtime->ready || runtime->host.state != DBT_HOST_IDENTIFIED) return DBT_BUSY;
    dbt_runtime_tx_t *tx = slot(runtime, handle);
    if (!tx) return DBT_INVALID;
    if (tx->active) return DBT_BUSY;
    dbt_result_t result;
    if (fixed_channel) {
        const dbt_link_t *link = dbt_acl_find(&runtime->acl, handle);
        if (cid == 0 || cid >= 64 || cid == 1 || cid == 5 ||
            !(runtime->channels.fixed_mask[link->kind] & ((uint64_t)1u << cid))) return DBT_INVALID;
        if (size > runtime->maximum_mtu) return DBT_NO_SPACE;
        result = dbt_l2cap_tx_begin(&tx->pdu, link, cid, data, size);
    } else {
        const dbt_channel_t *channel=dbt_channels_find(&runtime->channels,handle,cid);
        tx->credit_channel=channel && channel->le;
        tx->ertm_channel=channel && channel->ertm;
        if(tx->credit_channel){
            result=dbt_channels_credit_begin(&runtime->channels,handle,cid,data,size);
            if(result==DBT_OK)result=dbt_l2cap_tx_begin(&tx->pdu,dbt_acl_find(&runtime->acl,handle),channel->remote_cid,0,0);
        }else if(tx->ertm_channel){
            result=dbt_channels_ertm_begin(&runtime->channels,handle,cid,data,size);
            if(result==DBT_OK)result=dbt_l2cap_tx_begin(&tx->pdu,dbt_acl_find(&runtime->acl,handle),channel->remote_cid,0,0);
        }else result=dbt_channels_prepare_data(&runtime->channels,handle,cid,data,size,&tx->pdu);
    }
    if (result != DBT_OK) return result;
    if(fixed_channel)tx->credit_channel=tx->ertm_channel=false;
    tx->frame_ready=!tx->credit_channel && !tx->ertm_channel;
    tx->local_cid = cid; tx->done = done; tx->context = context; tx->active = true;
    return DBT_OK;
}
dbt_result_t dbt_runtime_send(dbt_runtime_t *runtime, uint16_t handle, uint16_t cid,
    const uint8_t *data, uint16_t size, dbt_tx_done_t done, void *context) {
    return send_pdu(runtime, handle, cid, data, size, done, context, false);
}
dbt_result_t dbt_runtime_send_fixed(dbt_runtime_t *runtime, uint16_t handle, uint16_t cid,
    const uint8_t *data, uint16_t size, dbt_tx_done_t done, void *context) {
    return send_pdu(runtime, handle, cid, data, size, done, context, true);
}
static bool credit(const dbt_runtime_t *runtime, const dbt_link_t *link) {
    return runtime->acl.pools[runtime->acl.shared_pool ? 0 : (unsigned)link->kind].available != 0;
}
static bool send_fragment(dbt_runtime_t *runtime, unsigned index) {
    const dbt_link_t *link = &runtime->acl.links[index];
    dbt_runtime_tx_t *app = &runtime->tx[index];
    dbt_runtime_ertm_tx_t *enhanced=&runtime->ertm_tx[index];
    if(enhanced->active){
        const dbt_channel_t *c=dbt_channels_find(&runtime->channels,link->handle,enhanced->local_cid);
        if(!c || c->state==DBT_CH_DISCONNECTING){
            if(c)(void)dbt_channels_ertm_cancel(&runtime->channels,link->handle,enhanced->local_cid);
            enhanced->active=false;
        }
    }
    if(app->active && app->local_cid>=0x40){
        const dbt_channel_t *c=dbt_channels_find(&runtime->channels,link->handle,app->local_cid);
        if(!c || c->state==DBT_CH_DISCONNECTING){
            if(app->ertm_channel && enhanced->active && enhanced->local_cid==app->local_cid){
                if(c)(void)dbt_channels_ertm_cancel(&runtime->channels,link->handle,app->local_cid);
                enhanced->active=false;
            }
            finish_tx(app,DBT_TX_ABORTED);
        }
    }
    bool control = runtime->control_active && runtime->control.handle == link->handle;
    bool app_ready=app->active && !app->ertm_channel && (!app->credit_channel || app->frame_ready ||
        dbt_channels_credit_ready(&runtime->channels,link->handle,app->local_cid));
    if (!control && !(app_ready && app->pdu.cursor) && !(enhanced->active && enhanced->pdu.cursor) && !runtime->control_active) {
        dbt_signal_view_t view;
        if (dbt_channels_tx_select(&runtime->channels, link->handle, &view) == DBT_OK) {
            (void)dbt_l2cap_tx_begin(&runtime->control, link, view.cid, view.data, view.size);
            runtime->control_active = true; control = true;
        }
    }
    if(!control && !enhanced->active && !(app_ready && app->pdu.cursor) && (enhanced->prefer_ertm || !app_ready)){
        dbt_result_t prepared=dbt_channels_ertm_frame(&runtime->channels,link->handle,&enhanced->pdu,&enhanced->local_cid);
        if(prepared==DBT_OK)enhanced->active=true;
    }
    bool use_ertm=!control && enhanced->active && (enhanced->pdu.cursor || !app_ready || enhanced->prefer_ertm);
    dbt_l2cap_tx_t *tx = control ? &runtime->control : use_ertm ? &enhanced->pdu : app_ready ? &app->pdu : 0;
    if (!tx) return false;
    if(!control && !use_ertm && app->credit_channel && !app->frame_ready){
        dbt_result_t prepared=dbt_channels_credit_frame(&runtime->channels,link->handle,app->local_cid,tx);
        if(prepared==DBT_BUSY)return false;
        if(prepared!=DBT_OK){finish_tx(app,DBT_TX_ABORTED);return false;}
        app->frame_ready=true;
    }
    dbt_result_t result = dbt_l2cap_send_next(tx, &runtime->acl, runtime->host.transport,
                                           runtime->tx_packet, sizeof(runtime->tx_packet));
    if (result == DBT_BUSY) return false;
    if (result != DBT_OK) { dbt_runtime_abort(runtime, DBT_FAULT_BAD_RESPONSE); return false; }
    if (tx->cursor == 4u + tx->size) {
        if (control) {
            runtime->control_active = false;
            (void)dbt_channels_tx_commit(&runtime->channels, runtime->now_ms);
        } else if(use_ertm){
            uint16_t cid=enhanced->local_cid;enhanced->active=false;enhanced->prefer_ertm=false;
            (void)dbt_channels_ertm_commit(&runtime->channels,link->handle,cid,runtime->now_ms);
        } else if(app->credit_channel){
            bool complete=false;
            if(dbt_channels_credit_commit(&runtime->channels,link->handle,app->local_cid,&complete)!=DBT_OK)
                finish_tx(app,DBT_TX_ABORTED);
            else if(complete){enhanced->prefer_ertm=true;finish_tx(app,DBT_TX_ADMITTED);}
            else{app->frame_ready=false;app->pdu.cursor=0;}
        } else {enhanced->prefer_ertm=true;finish_tx(app, DBT_TX_ADMITTED);}
    }
    return true;
}
void dbt_runtime_poll(dbt_runtime_t *runtime, uint32_t now_ms, uint16_t rx_budget, uint16_t tx_budget) {
    if (!runtime || runtime->in_poll) return;
    runtime->in_poll = true; runtime->now_ms = now_ms;
    if(runtime->ready)runtime->channels.now_ms=now_ms;
    /* Return released RX buffers before another ordinary command can take the
     * transport. Acks remain independent of command and outbound ACL credits. */
    dbt_rx_flow_poll(&runtime->rx_flow,now_ms);
    (void)dbt_host_poll(&runtime->host, now_ms, rx_budget);
    (void)initialize_layers(runtime);
    dbt_rx_flow_poll(&runtime->rx_flow,now_ms);
    if (runtime->host.state == DBT_HOST_FAILED) {
        dbt_runtime_abort(runtime, runtime->host.fault);
        runtime->in_poll = false; return;
    }
    if (runtime->ready) {
        (void)dbt_l2cap_expire(&runtime->rx, now_ms);
        (void)dbt_channels_poll(&runtime->channels, now_ms, rx_budget);
        for(unsigned i=0;i<DBT_MAX_LINKS;++i){
            dbt_runtime_tx_t *tx=&runtime->tx[i];
            if(tx->active && tx->ertm_channel){
                const dbt_channel_t *c=dbt_channels_find(&runtime->channels,tx->pdu.handle,tx->local_cid);
                if(!c || c->state!=DBT_CH_OPEN){
                    dbt_runtime_ertm_tx_t *frame=&runtime->ertm_tx[i];
                    if(frame->active && frame->local_cid==tx->local_cid){
                        if(c)(void)dbt_channels_ertm_cancel(&runtime->channels,tx->pdu.handle,tx->local_cid);
                        frame->active=false;
                    }
                    finish_tx(tx,DBT_TX_ABORTED);
                }else if(!dbt_channels_ertm_sdu_active(&runtime->channels,tx->pdu.handle,tx->local_cid))
                    finish_tx(tx,DBT_TX_ADMITTED);
            }
        }
        for (unsigned sent = 0; sent < tx_budget; ++sent) {
            if (!runtime->host.transport.can_send(runtime->host.transport.context)) break;
            bool progressed = false;
            for (unsigned n = 0; n < DBT_MAX_LINKS; ++n) {
                unsigned index = (runtime->next_link + n) % DBT_MAX_LINKS;
                const dbt_link_t *link = &runtime->acl.links[index];
                if (!link->used || !credit(runtime, link)) continue;
                if (!send_fragment(runtime, index)) continue;
                runtime->next_link = (uint8_t)((index + 1u) % DBT_MAX_LINKS);
                progressed = true; break;
            }
            if (!progressed || runtime->host.state == DBT_HOST_FAILED) break;
        }
    }
    runtime->in_poll = false;
}
uint32_t dbt_runtime_wait_ms(const dbt_runtime_t *runtime, uint32_t now_ms) {
    if (!runtime) return UINT32_MAX;
    uint32_t wait = dbt_host_wait_ms(&runtime->host, now_ms);
    if (!runtime->ready || runtime->host.state == DBT_HOST_FAILED) return wait;
    uint32_t receive_flow=dbt_rx_flow_wait_ms(&runtime->rx_flow);
    if(receive_flow<wait)wait=receive_flow;
    uint32_t channels = dbt_channels_wait_ms(&runtime->channels, now_ms);
    if (channels < wait) wait = channels;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_l2cap_partial_t *partial = &runtime->rx.partial[i];
        if (!partial->data) continue;
        uint32_t remaining = (uint32_t)(now_ms - partial->deadline) < 0x80000000u ? 0 : partial->deadline - now_ms;
        if (remaining < wait) wait = remaining;
    }
    if (!runtime->host.transport.can_send(runtime->host.transport.context)) return wait;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_link_t *link = &runtime->acl.links[i];
        if (!link->used || !credit(runtime, link)) continue;
        const dbt_runtime_tx_t *tx=&runtime->tx[i];
        bool data_ready=tx->active && !tx->ertm_channel && (!tx->credit_channel || tx->frame_ready ||
            dbt_channels_credit_ready(&runtime->channels,link->handle,tx->local_cid));
        if (data_ready || runtime->ertm_tx[i].active || dbt_channels_ertm_ready(&runtime->channels,link->handle,now_ms) ||
            (runtime->control_active && runtime->control.handle == link->handle) ||
            (!runtime->control_active && dbt_channels_tx_pending(&runtime->channels, link->handle))) return 0;
    }
    return wait;
}
