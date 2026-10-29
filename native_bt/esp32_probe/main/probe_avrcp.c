#include "probe_avrcp.h"
#include <string.h>
#include <stdatomic.h>

#define CONTROL_PSM 0x17u
#define CONTROL_MTU 672u
#define MESSAGE_BYTES 512u
#define CONTINUATION_BYTES 2048u
#define MAX_PEER_MTU 4096u
#define WORK_COUNT 16u
#define SNAP_WORDS ((sizeof(probe_avrcp_snapshot_t)+3u)/4u)

typedef struct { uint64_t token; uint8_t opcode,pdu,value; bool applied, changed; } work_t;
typedef struct {
    dbt_runtime_t *runtime;
    probe_avrcp_set_volume_fn set_volume;
    void *volume_context;
    dbt_avctp_t transport;
    dbt_avrcp_t profile;
    dbt_avctp_bearer_t bearer;
    uint8_t *storage;
    work_t work[WORK_COUNT];
    uint64_t volume_tokens[16];
    uint8_t volume_values[16];
    uint16_t volume_dirty;
    uint8_t work_head,work_count,reg_wanted,reg_sent,reg_label[2];
    bool initialized,closing,cap_need,cap_sent,meta_need,meta_sent,status_need,status_sent;
    uint16_t close_handle,close_cid;
    uint32_t close_serial, close_generation, next_work, connect_generation, next_connect;
    uint8_t connect_attempts, metadata_retries;
    bool collision;
    probe_avrcp_snapshot_t view;
    _Atomic uint32_t sequence;
    _Atomic uint32_t words[SNAP_WORDS];
} probe_t;
static probe_t s;

static bool due(uint32_t now,uint32_t deadline) { return (int32_t)(now-deadline)>=0; }
static void publish(void) {
    const uint8_t *bytes=(const uint8_t *)&s.view;
    atomic_fetch_add_explicit(&s.sequence,1,memory_order_acq_rel);
    for (size_t i=0;i<SNAP_WORDS;i++) {
        uint32_t word=0;
        for (size_t j=0;j<4;j++) if (4*i+j<sizeof(s.view)) word|=(uint32_t)bytes[4*i+j]<<(8*j);
        atomic_store_explicit(&s.words[i],word,memory_order_relaxed);
    }
    atomic_fetch_add_explicit(&s.sequence,1,memory_order_release);
}
dbt_result_t probe_avrcp_snapshot(probe_avrcp_snapshot_t *out) {
    if (!out) return DBT_INVALID;
    for (unsigned tries=0;tries<3;tries++) {
        uint32_t before=atomic_load_explicit(&s.sequence,memory_order_acquire);
        if (before&1u) continue;
        uint8_t *bytes=(uint8_t *)out;
        for (size_t i=0;i<SNAP_WORDS;i++) {
            uint32_t word=atomic_load_explicit(&s.words[i],memory_order_relaxed);
            for (size_t j=0;j<4;j++) if (4*i+j<sizeof(*out)) bytes[4*i+j]=(uint8_t)(word>>(8*j));
        }
        atomic_thread_fence(memory_order_acquire);
        if (before==atomic_load_explicit(&s.sequence,memory_order_relaxed)) return DBT_OK;
    }
    return DBT_BUSY;
}
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]<<8|p[1]); }
/* Preserve complete UTF-8 characters. Invalid or unsupported encodings are
 * omitted rather than making malformed bytes part of the logger snapshot. */
static bool utf8_copy(char *dst,const uint8_t *src,uint16_t n) {
    size_t out=0; bool truncated=false;
    for (uint16_t i=0;i<n;) {
        uint8_t a=src[i]; unsigned k; uint32_t cp;
        if (a<0x80) { k=1; cp=a; }
        else if (a>=0xc2 && a<=0xdf) { k=2; cp=a&31u; }
        else if (a>=0xe0 && a<=0xef) { k=3; cp=a&15u; }
        else if (a>=0xf0 && a<=0xf4) { k=4; cp=a&7u; }
        else { truncated=true; break; }
        if ((uint32_t)i+k>n) { truncated=true; break; }
        bool valid=true;
        for (unsigned j=1;j<k;j++) { uint8_t c=src[i+j]; if ((c&0xc0)!=0x80) {valid=false; break;} cp=(cp<<6)|(c&63u); }
        if (!valid || (k==2 && cp<0x80) || (k==3 && cp<0x800) || (k==4 && cp<0x10000) ||
            cp>0x10ffff || (cp>=0xd800 && cp<=0xdfff) || cp==0) { truncated=true; break; }
        if (out+k>=PROBE_AVRCP_TEXT_BYTES) {truncated=true; break;}
        for (unsigned j=0;j<k;j++) dst[out++]=(char)src[i+j];
        i=(uint16_t)(i+k);
    }
    dst[out]=0; return truncated;
}
static void update_metadata(const uint8_t *p,uint16_t n) {
    s.view.title[0]=s.view.artist[0]=s.view.album[0]=0; s.view.metadata_truncated=0;
    uint32_t pos=1;
    for (unsigned i=0;i<p[0] && pos+8<=n;i++) {
        uint32_t id=be32(p+pos); uint16_t charset=be16(p+pos+4),len=be16(p+pos+6); pos+=8;
        if (pos+len>n) {s.view.metadata_truncated=1; break;}
        char *dst=id==1?s.view.title:id==2?s.view.artist:id==3?s.view.album:0;
        if (dst) { if (charset!=106 || utf8_copy(dst,p+pos,len)) s.view.metadata_truncated=1; }
        pos+=len;
    }
    s.view.metadata_updates++; publish();
}
static void local_changed(uint8_t volume) {
    if (s.view.volume==volume) return;
    s.view.volume=volume;
    for (unsigned i=0;i<16;i++) if (s.volume_tokens[i]) {s.volume_values[i]=volume; s.volume_dirty|=(uint16_t)(1u<<i);}
    publish();
}
void probe_avrcp_local_volume(uint8_t volume) { if (s.initialized && volume<=127) local_changed(volume); }
static void message(void *unused,const dbt_avrcp_message_t *m) {
    (void)unused;
    if (m->invalid_pid) {s.closing=true; return;}
    if (m->aborted) {
        s.meta_sent=false; s.view.metadata_truncated=1; s.view.error_count++;
        if (!s.profile.abort_needed && s.metadata_retries++<2) s.meta_need=true;
        publish(); return;
    }
    if (m->response) {
        if (m->opcode==0x7c) return;
        if (m->pdu==0x10) {
            s.cap_sent=false; s.meta_need=true; s.status_need=true;
            if (m->code==12 && m->size>=2 && m->parameters[0]==3)
                for (unsigned i=0;i<m->parameters[1];i++) if (m->parameters[2+i]==1 || m->parameters[2+i]==2)
                    s.reg_wanted|=(uint8_t)(1u<<(m->parameters[2+i]-1));
        } else if (m->pdu==0x20) {
            s.meta_sent=false;
            if (m->code==12) {s.metadata_retries=0; update_metadata(m->parameters,m->size);}
        } else if (m->pdu==0x30) {
            s.status_sent=false;
            if (m->code==12) { s.view.track_length_ms=be32(m->parameters); s.view.track_position_ms=be32(m->parameters+4);
                s.view.playback_status=m->parameters[8]; publish(); }
        } else if (m->pdu==0x31) {
            for (unsigned i=0;i<2;i++) if ((s.reg_sent & (1u<<i)) && s.reg_label[i]==m->label) {
                if ((m->code==15 || m->code==13) && i==0) {s.view.playback_status=m->parameters[1]; publish();}
                if (m->code!=15) {s.reg_sent&=(uint8_t)~(1u<<i);
                    if (m->code==13) {s.reg_wanted|=(uint8_t)(1u<<i); if (i==1) s.meta_need=true; else s.status_need=true;}}
                break;
            }
        }
        return;
    }
    if (s.work_count==WORK_COUNT) {s.closing=true; s.view.error_count++; publish(); return;}
    unsigned tail=(s.work_head+s.work_count)%WORK_COUNT;
    s.work[tail]=(work_t){.token=m->token,.opcode=m->opcode,.pdu=m->pdu,
                          .value=m->size?m->parameters[0]:0};
    s.work_count++;
}
static void clear_session(void) {
    if (s.bearer.attached && dbt_avctp_bearer_clear(&s.bearer)!=DBT_OK) return;
    if (s.storage) {s.runtime->hooks.release(s.runtime->hooks.context,s.storage); s.storage=0;}
    memset(&s.bearer,0,sizeof(s.bearer)); memset(&s.profile,0,sizeof(s.profile)); memset(&s.transport,0,sizeof(s.transport));
    memset(s.volume_tokens,0,sizeof(s.volume_tokens)); s.volume_dirty=0; s.work_head=s.work_count=0;
    s.reg_wanted=s.reg_sent=0; s.metadata_retries=0; s.cap_need=s.cap_sent=s.meta_need=s.meta_sent=s.status_need=s.status_sent=false;
    if (s.view.connected) {s.view.connected=0; s.view.title[0]=s.view.artist[0]=s.view.album[0]=0;
        s.view.playback_status=0xff; publish();}
}
static bool open_session(const dbt_channel_t *ch) {
    if (s.storage || s.bearer.attached || ch->tx_mtu>MAX_PEER_MTU) return false;
    uint16_t size=(uint16_t)(3u*MESSAGE_BYTES+CONTINUATION_BYTES+ch->tx_mtu);
    uint8_t *p=s.runtime->hooks.allocate(s.runtime->hooks.context,size);
    if (!p) return false;
    s.storage=p;
    dbt_result_t r=dbt_avrcp_init(&s.profile,&s.transport,message,0,5000);
    if (r==DBT_OK) r=dbt_avctp_init(&s.transport,dbt_avrcp_hooks(&s.profile),ch->serial,ch->rx_mtu,ch->tx_mtu,false,
                                     p,MESSAGE_BYTES,p+MESSAGE_BYTES,MESSAGE_BYTES,p+2*MESSAGE_BYTES,5000,5000);
    if (r==DBT_OK) r=dbt_avrcp_set_echo_buffer(&s.profile,p+2*MESSAGE_BYTES+ch->tx_mtu,MESSAGE_BYTES);
    if (r==DBT_OK) r=dbt_avrcp_set_continuation_buffer(&s.profile,p+3*MESSAGE_BYTES+ch->tx_mtu,CONTINUATION_BYTES);
    if (r==DBT_OK) r=dbt_avctp_bearer_init(&s.bearer,s.runtime,&s.transport,ch);
    if (r!=DBT_OK) {clear_session(); return false;}
    s.cap_need=true; s.view.connected=1; s.view.generation=ch->link_generation; s.view.connection_count++;
    s.view.playback_status=0xff; s.connect_attempts=0; publish(); return true;
}
dbt_result_t probe_avrcp_init(dbt_runtime_t *runtime,probe_avrcp_set_volume_fn fn,void *context,uint8_t volume) {
    if (!runtime || !runtime->ready || !runtime->hooks.allocate || !runtime->hooks.release || !fn || volume>127 || s.initialized)
        return DBT_INVALID;
    dbt_result_t r=dbt_channels_register(&runtime->channels,(dbt_service_t){CONTROL_PSM,CONTROL_MTU,48,DBT_SECURITY_ENCRYPTED});
    if (r!=DBT_OK) return r;
    s.runtime=runtime; s.set_volume=fn; s.volume_context=context; s.initialized=true;
    s.view.volume=volume; s.view.playback_status=0xff; publish(); return DBT_OK;
}
bool probe_avrcp_ready(void) {return s.initialized;}
bool probe_avrcp_accept(void *unused,const dbt_link_t *link,uint16_t psm) {
    (void)unused; (void)link;
    return psm!=CONTROL_PSM || (s.initialized && !s.bearer.attached && !s.closing);
}
static unsigned control_count(void) {
    unsigned count=0;
    for (unsigned i=0;i<DBT_CHANNEL_LIMIT;i++)
        if (s.runtime->channels.channels[i].state!=DBT_CH_FREE && s.runtime->channels.channels[i].psm==CONTROL_PSM) count++;
    return count;
}
static void request_close(const dbt_channel_t *ch) {
    if (!s.closing || (s.close_serial==ch->serial)) {
        s.closing=true; s.close_handle=ch->handle; s.close_cid=ch->local_cid;
        s.close_serial=ch->serial; s.close_generation=ch->link_generation;
    }
}
void probe_avrcp_event(dbt_channel_event_t event,const dbt_channel_t *ch) {
    if (!s.initialized || !ch || ch->psm!=CONTROL_PSM) return;
    if (event==DBT_CH_EVENT_OPEN) {
        if (control_count()>1) {s.collision=true; return;}
        if (!open_session(ch)) {s.view.error_count++; publish(); request_close(ch);}
    } else if (event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED) {
        dbt_avctp_bearer_event(&s.bearer,event,ch);
        if (s.storage && !s.bearer.attached) clear_session();
        if (s.close_serial==ch->serial) {s.closing=false; s.close_serial=0;}
        s.next_connect=s.runtime->now_ms+5000;
    } else if (event==DBT_CH_EVENT_RECONFIGURED && s.bearer.attached && ch->serial==s.bearer.serial) request_close(ch);
}
void probe_avrcp_receive(const dbt_channel_t *ch,const uint8_t *data,uint16_t n) {
    if (!s.initialized || !ch || !s.bearer.attached || ch->serial!=s.bearer.serial ||
        ch->handle!=s.bearer.handle || ch->local_cid!=s.bearer.cid || ch->link_generation!=s.bearer.generation) return;
    dbt_result_t r=dbt_avrcp_bearer_receive(&s.bearer,&s.profile,ch,data,n);
    if (r!=DBT_OK && !(r==DBT_NO_SPACE && !s.profile.failed && s.profile.abort_needed)) {
        s.view.error_count++; publish(); request_close(ch);
    }
}
static bool run_work(uint32_t now) {
    if (!s.work_count) return false;
    work_t *w=&s.work[s.work_head]; uint8_t p[5]; uint16_t n=0; uint8_t code=0x0a;
    if (w->opcode==0x7c) {
        p[0]=w->value; p[1]=0; n=2; uint8_t op=w->value&0x7f;
        if (op!=0x41 && op!=0x42) code=8;
        else {
            if (!(w->value&0x80) && !w->applied) {
                uint8_t volume=s.view.volume;
                if (op==0x41 && volume<127) volume++; else if (op==0x42 && volume>0) volume--;
                dbt_result_t r=s.set_volume(s.volume_context,volume);
                if (r==DBT_BUSY) {s.next_work=now+10; return true;}
                if (r!=DBT_OK) code=0x0a;
                else {local_changed(volume); w->applied=true; code=9;}
            } else code=9;
        }
    } else if (w->pdu==0x10) {
        code=12; p[0]=w->value; p[1]=1;
        if (w->value==2) {p[2]=0; p[3]=0x19; p[4]=0x58; n=5;}
        else {p[2]=13; n=3;}
    } else if (w->pdu==0x31 && w->value==13) {
        code=15; p[0]=13; p[1]=s.view.volume; n=2;
    } else if (w->pdu==0x50) {
        if (!w->applied) {
            dbt_result_t r=s.set_volume(s.volume_context,w->value);
            if (r==DBT_BUSY) {s.next_work=now+10; return true;}
            if (r==DBT_OK) {s.view.volume=w->value; publish(); w->applied=true;}
            else {p[0]=3; n=1; goto reply;}
        }
        code=9; p[0]=w->value; n=1;
    } else {p[0]=0; n=1;}
reply:
    if (w->opcode==0x7c && code==0x0a) { /* AV/C pass-through rejects echo the operation. */ }
    dbt_result_t r=dbt_avrcp_reply(&s.profile,w->token,code,p,n,now);
    if (r==DBT_BUSY) return true;
    if (r!=DBT_OK) {s.closing=true; s.view.error_count++; publish(); return true;}
    if (w->opcode==0 && w->pdu==0x31 && code==15) {
        unsigned label=0; while(label<16 && s.profile.remote_token[label]!=w->token) label++;
        if (label<16) s.volume_tokens[label]=w->token;
    }
    s.work_head=(uint8_t)((s.work_head+1)%WORK_COUNT); s.work_count--; return true;
}
static bool send_notification(uint32_t now) {
    for (unsigned i=0;i<16;i++) if (s.volume_dirty&(1u<<i)) {
        uint8_t p[]={13,s.volume_values[i]};
        dbt_result_t r=dbt_avrcp_reply(&s.profile,s.volume_tokens[i],13,p,sizeof(p),now);
        if (r==DBT_BUSY) return true;
        s.volume_dirty&=(uint16_t)~(1u<<i); s.volume_tokens[i]=0;
        if (r!=DBT_OK && r!=DBT_EMPTY) {s.closing=true; s.view.error_count++; publish();}
        return true;
    }
    return false;
}
static void controller(uint32_t now) {
    uint8_t label; dbt_result_t r;
    if (s.cap_need && !s.cap_sent) {uint8_t p=3; r=dbt_avrcp_command(&s.profile,0x10,&p,1,now,&label);
        if (r==DBT_OK) {s.cap_need=false; s.cap_sent=true;} return;}
    for (unsigned i=0;i<2;i++) if ((s.reg_wanted&(1u<<i)) && !(s.reg_sent&(1u<<i))) {
        uint8_t p[5]={(uint8_t)(i+1),0,0,0,0}; r=dbt_avrcp_command(&s.profile,0x31,p,5,now,&label);
        if (r==DBT_OK) {s.reg_wanted&=(uint8_t)~(1u<<i); s.reg_sent|=(uint8_t)(1u<<i); s.reg_label[i]=label;} return;
    }
    if (s.meta_need && !s.meta_sent && !s.profile.abort_needed && !s.profile.local_abort && !s.profile.continuation_active) {
        uint8_t p[9]={0}; r=dbt_avrcp_command(&s.profile,0x20,p,9,now,&label);
        if (r==DBT_OK) {s.meta_need=false; s.meta_sent=true;} return;
    }
    if (s.status_need && !s.status_sent) {r=dbt_avrcp_command(&s.profile,0x30,0,0,now,&label);
        if (r==DBT_OK) {s.status_need=false; s.status_sent=true;} }
}
static bool has_channel(void) { return control_count()!=0; }
static void connect_if_ready(uint32_t now) {
    if (s.storage || s.closing || has_channel() || !due(now,s.next_connect)) return;
    for (unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&s.runtime->acl.links[i];
        if (!l->used || l->kind!=DBT_LINK_CLASSIC || !l->encrypted) continue;
        if (s.runtime->hooks.channels.authorize && !s.runtime->hooks.channels.authorize(
            s.runtime->hooks.channels.context,l,CONTROL_PSM,DBT_SECURITY_ENCRYPTED)) continue;
        if (s.connect_generation!=l->generation) {s.connect_generation=l->generation; s.connect_attempts=0;}
        if (s.connect_attempts>=3) continue;
        uint16_t cid; dbt_result_t r=dbt_channels_open(&s.runtime->channels,l->handle,CONTROL_PSM,CONTROL_MTU,48,
                                                         DBT_SECURITY_ENCRYPTED,now,&cid);
        if (r!=DBT_BUSY && r!=DBT_NO_SPACE) s.connect_attempts++;
        s.next_connect=now+5000; return;
    }
}
void probe_avrcp_poll(uint32_t now) {
    if (!s.initialized) return;
    if (control_count()>1) s.collision=true;
    if (s.collision) {
        /* Simultaneous establishment closes both channels before retry. The
         * bounded jitter derives from changing connection timing/generation. */
        for (unsigned i=0;i<DBT_CHANNEL_LIMIT;i++) {
            dbt_channel_t *ch=&s.runtime->channels.channels[i];
            if (ch->state!=DBT_CH_FREE && ch->state!=DBT_CH_DISCONNECTING && ch->psm==CONTROL_PSM)
                (void)dbt_channels_close(&s.runtime->channels,ch->handle,ch->local_cid,now);
        }
        if (!has_channel()) {s.collision=false; s.next_connect=now+100u+((now^s.connect_generation*2654435761u)%901u);}
        if (s.storage && !s.bearer.attached && !s.bearer.active) clear_session();
        return;
    }
    if (s.closing) {
        if (!s.close_serial && s.bearer.attached) {
            s.close_handle=s.bearer.handle; s.close_cid=s.bearer.cid; s.close_serial=s.bearer.serial; s.close_generation=s.bearer.generation;
        }
        const dbt_channel_t *ch=dbt_channels_find(&s.runtime->channels,s.close_handle,s.close_cid);
        if (ch && ch->serial==s.close_serial && ch->link_generation==s.close_generation && ch->state==DBT_CH_OPEN)
            (void)dbt_channels_close(&s.runtime->channels,ch->handle,ch->local_cid,now);
        else if (!ch && (!s.storage || !s.bearer.active)) {clear_session(); s.closing=false; s.close_serial=0;}
    }
    if (s.bearer.attached && !s.closing) {
        if (due(now,s.next_work)) {
            if (!run_work(now) && !send_notification(now)) controller(now);
        }
        if (dbt_avrcp_bearer_poll(&s.bearer,&s.profile,now)!=DBT_OK) {
            s.view.error_count++; publish(); s.closing=true;
        }
    }
    if (s.storage && !s.bearer.attached && !s.bearer.active) clear_session();
    connect_if_ready(now);
}
uint32_t probe_avrcp_wait_ms(uint32_t now) {
    if (!s.initialized) return UINT32_MAX;
    uint32_t w=s.bearer.attached ? dbt_avrcp_bearer_wait_ms(&s.bearer,&s.profile,now) : UINT32_MAX;
    if (s.collision) {
        for (unsigned i=0;i<DBT_CHANNEL_LIMIT;i++) {
            const dbt_channel_t *ch=&s.runtime->channels.channels[i];
            if (ch->psm==CONTROL_PSM && ch->state!=DBT_CH_FREE && ch->state!=DBT_CH_DISCONNECTING) return 0;
        }
        return has_channel()?UINT32_MAX:0;
    }
    if (s.closing) {
        const dbt_channel_t *ch=dbt_channels_find(&s.runtime->channels,s.close_handle,s.close_cid);
        return ch && ch->state==DBT_CH_DISCONNECTING ? UINT32_MAX : (!ch && s.bearer.active ? UINT32_MAX : 0);
    }
    bool runnable=s.work_count || s.volume_dirty || (s.cap_need && !s.cap_sent) ||
        (s.reg_wanted & (uint8_t)~s.reg_sent) ||
        (s.meta_need && !s.meta_sent && !s.profile.abort_needed && !s.profile.local_abort && !s.profile.continuation_active) ||
        (s.status_need && !s.status_sent);
    if (s.bearer.attached && runnable) {
        uint32_t delay=due(now,s.next_work)?0:s.next_work-now;
        if (delay<w && !s.transport.tx_used && !s.bearer.active) w=delay;
    }
    if (!s.storage && !has_channel() && !due(now,s.next_connect) && s.next_connect-now<w) w=s.next_connect-now;
    return w;
}
dbt_result_t probe_avrcp_key(uint8_t operation,bool release,uint32_t now) {
    if (!s.initialized || !s.bearer.attached || s.closing) return DBT_EMPTY;
    if (operation!=0x44 && operation!=0x45 && operation!=0x46 && operation!=0x48 && operation!=0x49 && operation!=0x4b && operation!=0x4c)
        return DBT_UNSUPPORTED;
    uint8_t label; return dbt_avrcp_passthrough(&s.profile,operation,release,now,&label);
}
