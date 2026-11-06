#include "dbt_avdtp_sink.h"

static void zero(void *data, unsigned size) { uint8_t *p=data; for(unsigned i=0;i<size;++i)p[i]=0; }
static void copy(uint8_t *to,const uint8_t *from,uint16_t size) { for(uint16_t i=0;i<size;++i)to[i]=from[i]; }
static bool due(uint32_t now,uint32_t deadline) { return (int32_t)(now-deadline)>=0; }
static uint32_t until(uint32_t now,uint32_t deadline) { return due(now,deadline)?0:deadline-now; }
static void emit(dbt_avdtp_sink_t *s,dbt_sink_event_t event) {
    if(s->hooks.event)s->hooks.event(s->hooks.context,event,s->state);
}
static void fault(dbt_avdtp_sink_t *s) { if(!s->failed){s->failed=true;emit(s,DBT_SINK_FAULT);} }
static uint8_t *allocate_signal(void *context,uint16_t size) {
    dbt_avdtp_sink_t *s=context;return s->hooks.allocate(s->hooks.context,size);
}
static void release_signal(void *context,uint8_t *data) {
    dbt_avdtp_sink_t *s=context;s->hooks.release(s->hooks.context,data);
}
static int endpoint_index(const dbt_avdtp_sink_t *s,uint8_t seid) {
    for(unsigned i=0;i<s->count;++i)if(s->endpoints[i].seid==seid)return (int)i;
    return -1;
}
static void drop_pending(dbt_avdtp_sink_t *s) {
    if(s->pending.configuration)s->hooks.release(s->hooks.context,s->pending.configuration);
    zero(&s->pending,sizeof(s->pending));
}
static void idle(dbt_avdtp_sink_t *s) {
    if(s->configuration)s->hooks.release(s->hooks.context,s->configuration);
    s->configuration=0;s->configuration_size=0;s->state=DBT_STREAM_IDLE;
    s->media_binding=0;s->media_deadline=0;s->delay_enabled=false;s->remote_seid=0;s->media_initiator=false;
}
static uint32_t token(dbt_avdtp_sink_t *s) { if(!++s->next_token)++s->next_token;return s->next_token; }

static dbt_result_t respond(dbt_avdtp_sink_t *s,dbt_avdtp_type_t type,const uint8_t *body,uint16_t size) {
    return dbt_avdtp_respond(&s->signal,s->signal.remote_label,s->signal.remote_signal,
                            type,body,size,s->signal.now_ms);
}
static void reject(dbt_avdtp_sink_t *s,uint8_t detail,uint8_t error) {
    uint8_t body[2]={detail,error};uint8_t command=s->signal.remote_signal;
    bool detailed=command==3 || command==5 || command==7 || command==9;
    if(respond(s,DBT_AVDTP_REJECT,detailed?body:body+1,detailed?2:1)!=DBT_OK)fault(s);
}
static void reply_pending(dbt_avdtp_sink_t *s) {
    dbt_sink_pending_t *p=&s->pending;
    if(s->local || !p->used || p->queued || !p->done || s->retiring || s->failed)return;
    uint8_t body[2]={p->error_detail,p->error};
    bool detailed=p->signal==3 || p->signal==5 || p->signal==7 || p->signal==9;
    dbt_result_t result=respond(s,p->error?DBT_AVDTP_REJECT:DBT_AVDTP_ACCEPT,
        p->error?(detailed?body:body+1):0,p->error?(detailed?2:1):0);
    if(result==DBT_OK){p->queued=true;p->reply=p->error?DBT_AVDTP_REJECT:DBT_AVDTP_ACCEPT;}
    else fault(s);
}
static void begin(dbt_avdtp_sink_t *s,dbt_sink_operation_t operation,uint8_t endpoint,
                   bool work,bool apply) {
    dbt_sink_pending_t *p=&s->pending;
    zero(p,sizeof(*p));p->used=true;p->operation=operation;p->endpoint=endpoint;
    p->label=s->signal.remote_label;p->signal=s->signal.remote_signal;
    p->deadline=s->signal.now_ms+s->worker_ms;p->apply=apply;p->done=!work;
    if(work)p->token=token(s);
}

static void apply_configuration(dbt_avdtp_sink_t *s,dbt_sink_pending_t *p){
    if(s->configuration)s->hooks.release(s->hooks.context,s->configuration);
    s->configuration=p->configuration;s->configuration_size=p->size;p->configuration=0;
    s->endpoint=p->endpoint;s->remote_seid=p->remote_seid;
    dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
    (void)dbt_avdtp_caps_validate(s->configuration,s->configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error);
    s->delay_enabled=(caps.present&(1u<<8))!=0;
    if(p->operation==DBT_SINK_CONFIGURE)s->state=DBT_STREAM_CONFIGURED;
}
static void local_outcome(dbt_avdtp_local_t *c,dbt_avdtp_local_outcome_t outcome){
    if(c->result.outcome==DBT_AVDTP_LOCAL_NONE)c->result.outcome=outcome;
}
static void local_finish(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,dbt_avdtp_local_outcome_t outcome){
    if(s->signal.local_completion==&c->completion)return;
    local_outcome(c,outcome);s->local=0;c->active=false;c->done=true;c->result.phase=DBT_AVDTP_LOCAL_DONE;
}
static void local_retired(dbt_avdtp_sink_t *s){
    dbt_avdtp_local_t *c=s->local;if(!c)return;
    c->result.profile_retired=true;c->result.cleanup_complete=!s->worker_live;
    local_finish(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);
}
static bool local_accepted(const dbt_avdtp_local_t *c,dbt_sink_operation_t operation){
    return c->result.operation==operation && c->result.transport_done &&
        c->result.response_seen && c->result.response_type==DBT_AVDTP_ACCEPT;
}
static bool media_outstanding(const dbt_avdtp_sink_t *s){
    return s->media_binding || (s->local && s->local->result.media_was_pending &&
        s->local->result.media_binding && !s->local->result.media_closed);
}
static bool media_waiting(const dbt_avdtp_sink_t *s){
    bool waiting=(s->state==DBT_STREAM_OPEN && !s->media_binding) ||
        ((s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING) && media_outstanding(s));
    if(s->local && s->local->result.operation==DBT_SINK_ABORT && !s->local->result.media_close_requested)return false;
    return waiting;
}
static void local_response(void *context,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *m){
    dbt_avdtp_local_t *c=context;if(!c || !c->active)return;
    c->notifying=true;dbt_avdtp_sink_t *s=c->sink;dbt_avdtp_local_result_t *r=&c->result;
    r->transport_done=true;r->transport_outcome=outcome;
    if(outcome==DBT_AVDTP_TIMEOUT)local_outcome(c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);
    else if(outcome==DBT_AVDTP_CANCELLED)local_outcome(c,r->cancel_requested?DBT_AVDTP_LOCAL_CANCELLED:DBT_AVDTP_LOCAL_SIGNAL_ERROR);
    else if(!m || m->signal!=r->command || m->label!=r->label)
        local_outcome(c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);
    else {
        r->response_seen=true;r->response_type=m->type;
        if(m->type==DBT_AVDTP_ACCEPT && r->operation==DBT_SINK_ABORT && c->wire_released &&
           !r->cancel_requested && !s->failed && !s->retiring && s->state==DBT_STREAM_ABORTING &&
           (s->media_binding==r->media_binding || (!s->media_binding && (r->media_closed || r->media_was_pending))) &&
           s->pending.operation==DBT_SINK_ABORT){
            r->phase=s->pending.done?(r->media_binding && !r->media_closed?DBT_AVDTP_LOCAL_WAIT_MEDIA_CLOSE:DBT_AVDTP_LOCAL_CLEANUP):DBT_AVDTP_LOCAL_APPLYING;
            if(r->media_binding && !r->media_closed && !r->media_close_requested){
                r->media_close_requested=true;s->media_deadline=s->signal.now_ms+s->media_ms;emit(s,DBT_SINK_CLOSE_MEDIA);
            }
        }else if(m->type==DBT_AVDTP_ACCEPT && r->operation==DBT_SINK_SUSPEND && c->wire_released &&
           !r->cancel_requested && !s->failed && !s->retiring && s->state==DBT_STREAM_STREAMING &&
           s->media_binding==r->media_binding && s->pending.operation==DBT_SINK_SUSPEND && s->pending.done){
            begin(s,DBT_SINK_SUSPEND,c->endpoint,true,true);r->phase=DBT_AVDTP_LOCAL_APPLYING;
            r->worker_token=s->pending.token;r->worker_deadline_ms=s->pending.deadline;
        }else if(m->type==DBT_AVDTP_ACCEPT && r->operation==DBT_SINK_CLOSE && c->wire_released &&
           !r->cancel_requested && !s->failed && !s->retiring &&
           (s->state==DBT_STREAM_OPEN || s->state==DBT_STREAM_STREAMING || (s->state==DBT_STREAM_CLOSING && r->media_closed)) &&
           (s->media_binding==r->media_binding || (!s->media_binding && r->media_closed)) &&
           s->pending.operation==DBT_SINK_CLOSE && s->pending.done){
            begin(s,DBT_SINK_CLOSE,c->endpoint,true,true);r->phase=DBT_AVDTP_LOCAL_APPLYING;
            r->worker_token=s->pending.token;r->worker_deadline_ms=s->pending.deadline;
            s->state=DBT_STREAM_CLOSING;s->media_deadline=s->signal.now_ms+s->media_ms;
            emit(s,DBT_SINK_STATE);
            if(s->media_binding){r->media_close_requested=true;emit(s,DBT_SINK_CLOSE_MEDIA);}
        }else if(m->type==DBT_AVDTP_ACCEPT && c->wire_released && !r->cancel_requested &&
           !s->failed && !s->retiring && s->pending.operation==r->operation &&
           s->pending.done && r->worker_result_valid && r->worker_result==DBT_SINK_WORK_READY &&
            ((r->operation==DBT_SINK_CONFIGURE && s->state==DBT_STREAM_IDLE) ||
             (r->operation==DBT_SINK_RECONFIGURE && s->state==DBT_STREAM_OPEN && s->media_binding==r->media_binding) ||
             (r->operation==DBT_SINK_OPEN && s->state==DBT_STREAM_CONFIGURED) ||
             (r->operation==DBT_SINK_START && s->state==DBT_STREAM_OPEN && s->media_binding==r->media_binding))){
            if(r->operation==DBT_SINK_CONFIGURE || r->operation==DBT_SINK_RECONFIGURE){
                apply_configuration(s,&s->pending);r->resources_transferred=true;
            }else if(r->operation==DBT_SINK_OPEN){
                s->state=DBT_STREAM_OPEN;s->media_initiator=true;s->media_deadline=s->signal.now_ms+s->media_ms;
            }else s->state=DBT_STREAM_STREAMING;
            local_finish(s,c,DBT_AVDTP_LOCAL_SUCCESS);
            /* Commit before another peer command can arrive. The result getter
             * stays pending until this notification returns. */
            emit(s,DBT_SINK_STATE);drop_pending(s);
        }else if(m->type!=DBT_AVDTP_ACCEPT){
            if(m->size){
                if((r->command==DBT_AVDTP_SET_CONFIGURATION || r->command==DBT_AVDTP_RECONFIGURE) && m->size>1)r->peer_category=m->data[0];
                if((r->command==DBT_AVDTP_START || r->command==DBT_AVDTP_SUSPEND) && m->size>1)r->peer_seid=m->data[0]>>2;
                r->peer_error=m->data[m->size-1u];
            }
            local_outcome(c,DBT_AVDTP_LOCAL_PEER_REJECTED);
        }else if(!r->cancel_requested){
            local_outcome(c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);r->phase=DBT_AVDTP_LOCAL_RETIRING;fault(s);
        }
    }
    c->notifying=false;
}
static void local_failure(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,dbt_avdtp_local_outcome_t outcome){
    local_outcome(c,outcome);c->result.phase=DBT_AVDTP_LOCAL_RETIRING;fault(s);
}
static dbt_result_t local_worker_complete(dbt_avdtp_sink_t *s,dbt_sink_work_result_t result,uint32_t now){
    dbt_avdtp_local_t *c=s->local;dbt_sink_pending_t *p=&s->pending;
    p->done=true;
    if(p->operation==DBT_SINK_DROP){
        c->result.drop_result=result;c->result.drop_result_valid=true;
        if(result==DBT_SINK_WORK_READY){s->worker_live=false;c->result.cleanup_complete=true;}
        else local_failure(s,c,DBT_AVDTP_LOCAL_CLEANUP_ERROR);
    }else if(c->rollback_started && p->token==c->result.rollback_token && p->operation==c->result.rollback_operation){
        c->result.rollback_result=result;c->result.rollback_result_valid=true;
        if(due(now,p->deadline))c->result.rollback_timed_out=true;
        if(result==DBT_SINK_WORK_READY && !c->result.rollback_timed_out)c->result.cleanup_complete=true;
        else local_failure(s,c,DBT_AVDTP_LOCAL_ROLLBACK_ERROR);
    }else {
        c->result.worker_result=result;c->result.worker_result_valid=true;
        if(due(now,p->deadline)){c->result.worker_timed_out=true;local_outcome(c,DBT_AVDTP_LOCAL_WORKER_TIMEOUT);}
        if((c->result.operation==DBT_SINK_ABORT || local_accepted(c,DBT_SINK_SUSPEND) || local_accepted(c,DBT_SINK_CLOSE)) &&
           (result!=DBT_SINK_WORK_READY || c->result.worker_timed_out)){
            local_failure(s,c,c->result.worker_timed_out?DBT_AVDTP_LOCAL_WORKER_TIMEOUT:
                result==DBT_SINK_WORK_FATAL?DBT_AVDTP_LOCAL_WORKER_FATAL:DBT_AVDTP_LOCAL_WORKER_REJECTED);
        }else if((c->result.operation==DBT_SINK_ABORT || local_accepted(c,DBT_SINK_CLOSE)) && result==DBT_SINK_WORK_READY){
            s->worker_live=false;
            if(!s->failed)c->result.phase=c->result.operation==DBT_SINK_ABORT && !local_accepted(c,DBT_SINK_ABORT)?DBT_AVDTP_LOCAL_WAIT_PEER:
                (c->result.media_binding && !c->result.media_closed?DBT_AVDTP_LOCAL_WAIT_MEDIA_CLOSE:DBT_AVDTP_LOCAL_CLEANUP);
        }else if(result==DBT_SINK_WORK_REJECTED){
            if(c->result.operation==DBT_SINK_CONFIGURE)s->worker_live=false;
            local_outcome(c,DBT_AVDTP_LOCAL_WORKER_REJECTED);
        }else if(result==DBT_SINK_WORK_FATAL)local_failure(s,c,DBT_AVDTP_LOCAL_WORKER_FATAL);
    }
    return s->failed?DBT_INVALID:DBT_OK;
}
static void local_abort(dbt_avdtp_sink_t *s,const dbt_avdtp_message_t *m,unsigned index){
    dbt_avdtp_local_t *c=s->local;
    if(index!=c->endpoint){if(respond(s,DBT_AVDTP_ACCEPT,0,0)!=DBT_OK)fault(s);return;}
    c->result.remote_abort=true;c->result.cancel_requested=true;c->abort_label=m->label;
    local_outcome(c,DBT_AVDTP_LOCAL_REMOTE_ABORT);
}
static void local_poll(dbt_avdtp_sink_t *s,uint32_t now){
    dbt_avdtp_local_t *c=s->local;if(!c || s->retiring)return;
    dbt_avdtp_local_result_t *r=&c->result;dbt_sink_pending_t *p=&s->pending;
    if(!c->cleaning && !p->done && due(now,p->deadline)){
        r->worker_timed_out=true;local_outcome(c,DBT_AVDTP_LOCAL_WORKER_TIMEOUT);
        if(r->operation==DBT_SINK_ABORT || local_accepted(c,DBT_SINK_SUSPEND) || local_accepted(c,DBT_SINK_CLOSE)){local_failure(s,c,DBT_AVDTP_LOCAL_WORKER_TIMEOUT);return;}
    }
    bool cleanup=r->cancel_requested || r->outcome!=DBT_AVDTP_LOCAL_NONE;
    if(!cleanup && !c->wire_released && (p->done || r->operation==DBT_SINK_ABORT)){
        if(r->operation!=DBT_SINK_ABORT && r->operation!=DBT_SINK_SUSPEND && r->operation!=DBT_SINK_CLOSE && (!r->worker_result_valid || r->worker_result!=DBT_SINK_WORK_READY)){
            local_failure(s,c,DBT_AVDTP_LOCAL_WORKER_FATAL);return;
        }
        dbt_result_t ready=dbt_avdtp_command_ready(&s->signal,s->binding,r->label,&c->completion,now);
        if(ready!=DBT_OK){local_failure(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);return;}
        c->wire_released=true;r->phase=DBT_AVDTP_LOCAL_WAIT_PEER;
        if(r->operation==DBT_SINK_ABORT){
            emit(s,DBT_SINK_STATE);
            if(r->media_was_pending && !r->media_closed && !r->media_close_requested){
                r->media_close_requested=true;s->media_deadline=now+s->media_ms;emit(s,DBT_SINK_CLOSE_MEDIA);
            }
        }
        return;
    }
    if(!cleanup && local_accepted(c,DBT_SINK_SUSPEND) && p->done){
        if(!r->worker_result_valid || r->worker_result!=DBT_SINK_WORK_READY ||
           s->state!=DBT_STREAM_STREAMING || s->media_binding!=r->media_binding){
            local_failure(s,c,DBT_AVDTP_LOCAL_WORKER_FATAL);return;
        }
        c->notifying=true;s->state=DBT_STREAM_OPEN;local_finish(s,c,DBT_AVDTP_LOCAL_SUCCESS);
        emit(s,DBT_SINK_STATE);drop_pending(s);c->notifying=false;return;
    }
    if(!cleanup && local_accepted(c,DBT_SINK_CLOSE)){
        if(!p->done || s->media_binding)return;
        if(!r->worker_result_valid || r->worker_result!=DBT_SINK_WORK_READY || s->worker_live || !r->media_closed){
            local_failure(s,c,DBT_AVDTP_LOCAL_CLEANUP_ERROR);return;
        }
        c->notifying=true;idle(s);r->cleanup_complete=true;local_finish(s,c,DBT_AVDTP_LOCAL_SUCCESS);
        emit(s,DBT_SINK_STATE);drop_pending(s);c->notifying=false;return;
    }
    if(!cleanup && local_accepted(c,DBT_SINK_ABORT)){
        if(!p->done || s->media_binding || (r->media_binding && !r->media_closed))return;
        if(s->worker_live || (!r->worker_not_needed && (!r->worker_result_valid || r->worker_result!=DBT_SINK_WORK_READY))){
            local_failure(s,c,DBT_AVDTP_LOCAL_CLEANUP_ERROR);return;
        }
        c->notifying=true;idle(s);r->cleanup_complete=true;local_finish(s,c,DBT_AVDTP_LOCAL_SUCCESS);
        emit(s,DBT_SINK_STATE);drop_pending(s);c->notifying=false;return;
    }
    if(!cleanup)return;
    if(!r->transport_done && !c->cancel_attempted){
        dbt_result_t cancelled=dbt_avdtp_command_cancel(&s->signal,s->binding,r->label,&c->completion,now);
        if(cancelled!=DBT_OK && cancelled!=DBT_PENDING){local_failure(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);return;}
        c->cancel_attempted=true;r->command_cancelled_unsent=cancelled==DBT_OK;
    }
    if(r->cancel_requested && c->wire_released && !r->remote_abort && !r->command_cancelled_unsent &&
       (!r->transport_done || (r->response_seen && r->response_type==DBT_AVDTP_ACCEPT))){
        local_failure(s,c,DBT_AVDTP_LOCAL_CANCELLED);return;
    }
    if(r->operation==DBT_SINK_ABORT && !r->remote_abort){local_failure(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);return;}
    if(r->operation==DBT_SINK_CLOSE && r->media_closed && !r->remote_abort){
        local_failure(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);return;
    }
    if(!c->cleaning){c->cleaning=true;r->phase=DBT_AVDTP_LOCAL_CLEANUP;}
    bool release=r->operation==DBT_SINK_CONFIGURE || r->remote_abort;
    if(release && !c->drop_started){
        c->drop_started=true;r->cleanup_complete=false;drop_pending(s);
        if(!s->worker_live)r->cleanup_complete=true;
        else {begin(s,DBT_SINK_DROP,c->endpoint,true,false);s->pending.deadline=now+s->worker_ms;r->drop_token=s->pending.token;}
    }else if(!release){
        bool rollback=r->worker_accepted && (r->operation==DBT_SINK_START ||
            (r->operation==DBT_SINK_RECONFIGURE && !(r->worker_result_valid && r->worker_result==DBT_SINK_WORK_REJECTED)));
        if(rollback && !c->rollback_started){
            c->rollback_started=true;r->cleanup_complete=false;r->phase=DBT_AVDTP_LOCAL_ROLLBACK;drop_pending(s);
            r->rollback_operation=r->operation==DBT_SINK_RECONFIGURE?DBT_SINK_RECONFIGURE:DBT_SINK_SUSPEND;
            begin(s,r->rollback_operation,c->endpoint,true,false);s->pending.deadline=now+s->worker_ms;
            r->rollback_token=s->pending.token;r->rollback_deadline_ms=s->pending.deadline;
        }else if(!rollback && (!r->worker_accepted || p->done))r->cleanup_complete=true;
    }
    p=&s->pending;
    if(!r->cleanup_complete && p->used && !p->done && due(now,p->deadline)){
        r->cleanup_timed_out=true;
        bool rollback=c->rollback_started && p->token==r->rollback_token && p->operation==r->rollback_operation;
        if(rollback)r->rollback_timed_out=true;
        local_failure(s,c,rollback?DBT_AVDTP_LOCAL_ROLLBACK_ERROR:DBT_AVDTP_LOCAL_CLEANUP_ERROR);return;
    }
    if(r->cleanup_complete && r->remote_abort && !c->abort_queued){
        if(!s->signal.remote_pending || s->signal.remote_signal!=DBT_AVDTP_ABORT ||
           s->signal.remote_label!=c->abort_label || respond(s,DBT_AVDTP_ACCEPT,0,0)!=DBT_OK){
            local_failure(s,c,DBT_AVDTP_LOCAL_SIGNAL_ERROR);return;
        }
        c->abort_queued=true;
    }
    if(r->cleanup_complete && r->remote_abort && r->abort_response_admitted && media_outstanding(s)){
        if(s->state!=DBT_STREAM_ABORTING){s->state=DBT_STREAM_ABORTING;emit(s,DBT_SINK_STATE);}
        if(!r->media_close_requested){
            s->media_deadline=now+s->media_ms;r->phase=DBT_AVDTP_LOCAL_WAIT_MEDIA_CLOSE;r->media_close_requested=true;
            emit(s,DBT_SINK_CLOSE_MEDIA);
        }
        return; /* Media closure/event or its deadline wakes the owner. */
    }
    if(r->cleanup_complete && r->transport_done && (!r->remote_abort || r->abort_response_admitted)){
        if(release){bool changed=s->state!=DBT_STREAM_IDLE;idle(s);if(changed)emit(s,DBT_SINK_STATE);}
        drop_pending(s);local_finish(s,c,DBT_AVDTP_LOCAL_CANCELLED);
    }
}
static uint32_t local_wait(const dbt_avdtp_sink_t *s){
    const dbt_avdtp_local_t *c=s->local;if(!c || s->retiring)return UINT32_MAX;
    const dbt_avdtp_local_result_t *r=&c->result;
    if((r->cancel_requested || r->outcome!=DBT_AVDTP_LOCAL_NONE) && !c->cleaning)return 0;
    if(!c->wire_released && (s->pending.done || r->operation==DBT_SINK_ABORT) && !c->cleaning)return 0;
    if(local_accepted(c,DBT_SINK_SUSPEND) && s->pending.done && !c->cleaning)return 0;
    if(local_accepted(c,DBT_SINK_CLOSE) && s->pending.done && !s->media_binding && !c->cleaning)return 0;
    if(local_accepted(c,DBT_SINK_ABORT) && s->pending.done && !s->media_binding &&
       (!r->media_binding || r->media_closed) && !c->cleaning)return 0;
    if(c->cleaning && !c->drop_started && (r->remote_abort ||
       (!r->cleanup_complete && (!r->worker_accepted || s->pending.done))))return 0;
    if(r->cleanup_complete){
        if(r->remote_abort && !c->abort_queued)return 0;
        if(r->remote_abort && r->abort_response_admitted && media_outstanding(s) && (s->state!=DBT_STREAM_ABORTING || !r->media_close_requested))return 0;
        if(r->transport_done && (!r->remote_abort || r->abort_response_admitted) &&
           (!r->remote_abort || !media_outstanding(s)))return 0;
    }
    return UINT32_MAX;
}

static void capabilities(dbt_avdtp_sink_t *s,const dbt_avdtp_endpoint_t *ep,bool all) {
    if(all){
        if(respond(s,DBT_AVDTP_ACCEPT,ep->capabilities,ep->size)!=DBT_OK)reject(s,0,0x31);
        return;
    }
    uint16_t cursor=0,size=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(ep->capabilities,ep->size,&cursor,&cap)==DBT_OK)
        if(cap.category<8)size=(uint16_t)(size+2u+cap.size);
    if(size==ep->size){
        if(respond(s,DBT_AVDTP_ACCEPT,ep->capabilities,size)!=DBT_OK)reject(s,0,0x31);
        return;
    }
    uint8_t small[16];uint8_t *bytes=size<=sizeof(small)?small:s->hooks.allocate(s->hooks.context,size);
    if(!bytes){reject(s,0,0x31);return;}
    uint16_t offset=0;cursor=0;
    while(dbt_avdtp_cap_next(ep->capabilities,ep->size,&cursor,&cap)==DBT_OK)if(cap.category<8){
        bytes[offset++]=cap.category;bytes[offset++]=cap.size;copy(bytes+offset,cap.data,cap.size);offset=(uint16_t)(offset+cap.size);
    }
    dbt_result_t result=respond(s,DBT_AVDTP_ACCEPT,bytes,size);
    if(bytes!=small)s->hooks.release(s->hooks.context,bytes);
    if(result!=DBT_OK)reject(s,0,0x31);
}

static void configure(dbt_avdtp_sink_t *s,const dbt_avdtp_message_t *m,unsigned index,bool reconfigure) {
    uint16_t header=reconfigure?1:2;
    if(m->size<header){reject(s,0,0x11);return;}
    if(s->local){reject(s,0,reconfigure?0x31:0x13);return;}
    if((reconfigure && (s->state!=DBT_STREAM_OPEN || index!=s->endpoint)) ||
       (!reconfigure && s->state!=DBT_STREAM_IDLE)){
        reject(s,0,!reconfigure && index==s->endpoint?0x13:0x31);return;
    }
    uint8_t remote=reconfigure?s->remote_seid:m->data[1]>>2;
    if(!remote || remote>62){reject(s,0,0x18);return;}
    dbt_avdtp_caps_t caps,local;dbt_avdtp_cap_error_t error;
    const dbt_avdtp_endpoint_t *ep=&s->endpoints[index];
    if(dbt_avdtp_caps_validate(m->data+header,(uint16_t)(m->size-header),
        reconfigure?DBT_AVDTP_RECONFIGURATION:DBT_AVDTP_CONFIGURATION,&caps,&error)!=DBT_OK){
        reject(s,error.category,error.code);return;
    }
    (void)dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&local,&error);
    uint16_t cursor=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(m->data+header,(uint16_t)(m->size-header),&cursor,&cap)==DBT_OK)
        if(!(local.present&(1u<<cap.category))){reject(s,cap.category,0x29);return;}
    if(!caps.codec.data){reject(s,7,0x1a);return;}
    uint8_t code=ep->validate(ep,&caps.codec);
    if(code){reject(s,7,code);return;}
    uint16_t bytes=(uint16_t)(4u+caps.codec.size);
    bool delay=reconfigure?s->delay_enabled:(caps.present&(1u<<8))!=0;
    if(delay)bytes=(uint16_t)(bytes+2u);
    if(bytes>s->maximum_configuration){reject(s,7,0x29);return;}
    uint8_t *configuration=s->hooks.allocate(s->hooks.context,bytes);
    if(!configuration){reject(s,7,0x29);return;}
    configuration[0]=1;configuration[1]=0;configuration[2]=7;configuration[3]=caps.codec.size;
    copy(configuration+4,caps.codec.data,caps.codec.size);
    if(delay){configuration[bytes-2]=8;configuration[bytes-1]=0;}
    begin(s,reconfigure?DBT_SINK_RECONFIGURE:DBT_SINK_CONFIGURE,(uint8_t)index,true,true);
    s->pending.configuration=configuration;s->pending.size=bytes;s->pending.remote_seid=remote;
}

static void list_control(dbt_avdtp_sink_t *s,const dbt_avdtp_message_t *m,bool start) {
    if(!m->size){reject(s,0,0x11);return;}
    bool prefix=false;uint8_t bad=0,error=0;
    for(unsigned i=0;i<m->size;++i){
        uint8_t seid=m->data[i]>>2;int index=endpoint_index(s,seid);
        if(index<0){bad=(uint8_t)(seid<<2);error=0x12;break;}
        if(s->local || prefix || (unsigned)index!=s->endpoint ||
           s->state!=(start?DBT_STREAM_OPEN:DBT_STREAM_STREAMING) ||
           !s->worker_live || (start && !s->media_binding)){
            bad=(uint8_t)(seid<<2);error=0x31;break;
        }
        prefix=true;
    }
    if(!prefix){reject(s,bad,error);return;}
    /* With one active stream the valid prefix has at most one SEP. It is acted
     * on even if a later list item is rejected, as required by ordered lists. */
    begin(s,start?DBT_SINK_START:DBT_SINK_SUSPEND,s->endpoint,true,true);
    s->pending.error_detail=bad;s->pending.error=error;
}

static void command(void *context,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;
    if(s->failed || s->retiring){reject(s,0,0x31);return;}
    if(m->signal==DBT_AVDTP_DISCOVER){
        if(m->size){reject(s,0,0x11);return;}
        uint8_t bytes[2*DBT_AVDTP_SINK_ENDPOINT_LIMIT];
        for(unsigned i=0;i<s->count;++i){
            bytes[2*i]=(uint8_t)((s->endpoints[i].seid<<2)|(((s->state!=DBT_STREAM_IDLE && s->endpoint==i) || (s->local && s->local->endpoint==i))?2:0));
            bytes[2*i+1]=8; /* Audio, sink */
        }
        if(respond(s,DBT_AVDTP_ACCEPT,bytes,(uint16_t)(2u*s->count))!=DBT_OK)reject(s,0,0x31);
        return;
    }
    if(m->signal==DBT_AVDTP_START || m->signal==DBT_AVDTP_SUSPEND){
        list_control(s,m,m->signal==DBT_AVDTP_START);return;
    }
    if(!m->size){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x11);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    uint8_t seid=m->data[0]>>2;int index=endpoint_index(s,seid);
    if(index<0){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x12);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    if(m->signal==DBT_AVDTP_SET_CONFIGURATION || m->signal==DBT_AVDTP_RECONFIGURE){
        configure(s,m,(unsigned)index,m->signal==DBT_AVDTP_RECONFIGURE);return;
    }
    if(m->signal==DBT_AVDTP_SECURITY_CONTROL || m->signal==DBT_AVDTP_DELAY_REPORT){reject(s,0,0x19);return;}
    if(m->size!=1){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x11);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    if(s->local && m->signal==DBT_AVDTP_ABORT){local_abort(s,m,(unsigned)index);return;}
    switch(m->signal){
    case DBT_AVDTP_GET_CAPABILITIES:case DBT_AVDTP_GET_ALL_CAPABILITIES:
        capabilities(s,&s->endpoints[index],m->signal==DBT_AVDTP_GET_ALL_CAPABILITIES);return;
    case DBT_AVDTP_GET_CONFIGURATION:
        if(s->state==DBT_STREAM_IDLE || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        if(respond(s,DBT_AVDTP_ACCEPT,s->configuration,s->configuration_size)!=DBT_OK)reject(s,0,0x31);
        return;
    case DBT_AVDTP_OPEN:
        if(s->local){reject(s,0,0x31);return;}
        if(s->state!=DBT_STREAM_CONFIGURED || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        begin(s,DBT_SINK_OPEN,s->endpoint,true,true);return;
    case DBT_AVDTP_CLOSE:
        if(s->local){reject(s,0,0x31);return;}
        if((s->state!=DBT_STREAM_OPEN && s->state!=DBT_STREAM_STREAMING) || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        begin(s,DBT_SINK_CLOSE,s->endpoint,true,true);return;
    case DBT_AVDTP_ABORT:
        begin(s,DBT_SINK_ABORT,(uint8_t)index,(s->worker_live || s->state!=DBT_STREAM_IDLE) && (unsigned)index==s->endpoint,
              (s->worker_live || s->state!=DBT_STREAM_IDLE) && (unsigned)index==s->endpoint);
        reply_pending(s);return;
    default:reject(s,0,0x19);return;
    }
}

static void responded(void *context,uint8_t label,uint8_t signal,dbt_avdtp_type_t type) {
    dbt_avdtp_sink_t *s=context;dbt_sink_pending_t *p=&s->pending;
    if(s->local){
        dbt_avdtp_local_t *c=s->local;
        if(c->result.remote_abort && c->abort_queued && label==c->abort_label &&
           signal==DBT_AVDTP_ABORT && type==DBT_AVDTP_ACCEPT)c->result.abort_response_admitted=true;
        return;
    }
    if(!p->used || !p->queued || p->label!=label || p->signal!=signal || p->reply!=type)return;
    bool close_media=false;
    if(p->apply){
        switch(p->operation){
        case DBT_SINK_CONFIGURE:case DBT_SINK_RECONFIGURE:{
            apply_configuration(s,p);
            break;}
        case DBT_SINK_OPEN:s->state=DBT_STREAM_OPEN;s->media_initiator=false;s->media_deadline=s->signal.now_ms+s->media_ms;break;
        case DBT_SINK_START:s->state=DBT_STREAM_STREAMING;break;
        case DBT_SINK_SUSPEND:s->state=DBT_STREAM_OPEN;break;
        case DBT_SINK_CLOSE:case DBT_SINK_ABORT:
            s->state=p->operation==DBT_SINK_CLOSE?DBT_STREAM_CLOSING:DBT_STREAM_ABORTING;
            if(s->media_binding){close_media=true;s->media_deadline=s->signal.now_ms+s->media_ms;}
            else idle(s);
            break;
        default:break;
        }
        emit(s,DBT_SINK_STATE);
    }
    drop_pending(s);
    if(close_media)emit(s,DBT_SINK_CLOSE_MEDIA);
}
static void local_complete(void *context,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;(void)m;
    if(!s->retiring && outcome!=DBT_AVDTP_RESPONSE)fault(s);
    if(!s->retiring)emit(s,DBT_SINK_DELAY_RESPONSE);
}
static bool interrupt(void *context,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;dbt_sink_pending_t *p=&s->pending;
    if(s->local || s->failed || s->retiring || m->size!=1 || !p->used || p->queued ||
       (m->data[0]>>2)!=s->endpoints[p->endpoint].seid)return false;
    if(s->state==DBT_STREAM_IDLE && s->worker_live)s->endpoint=p->endpoint;
    /* Worker copied its job. A later serialized Abort releases any resources
     * it creates; the superseded completion token must no longer affect state. */
    drop_pending(s);
    return true;
}

void dbt_avdtp_local_init(dbt_avdtp_local_t *c){if(c)*c=(dbt_avdtp_local_t){0};}
static bool local_overlap(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool local_owned(const dbt_avdtp_sink_t *s,const void *p,unsigned n){
    return local_overlap(s,sizeof(*s),p,n) || local_overlap(s->configuration,s->configuration_size,p,n) ||
        local_overlap(s->pending.configuration,s->pending.size,p,n) || local_overlap(s->signal.rx,s->signal.rx_capacity,p,n) ||
        local_overlap(s->signal.packet,s->signal.packet_size,p,n) ||
        local_overlap(s->signal.tx[0].data,s->signal.tx[0].size,p,n) ||
        local_overlap(s->signal.tx[1].data,s->signal.tx[1].size,p,n);
}
dbt_result_t dbt_avdtp_sink_local_configure_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,
    uint8_t local_seid,uint8_t remote_seid,const uint8_t *configuration,uint16_t size,uint32_t now,uint32_t *id){
    if(!s || !c || !configuration || !id || !size || !remote_seid || remote_seid>62 ||
       local_owned(s,c,sizeof(*c)) || local_owned(s,configuration,size) || local_overlap(c,sizeof(*c),configuration,size) ||
       local_overlap(c,sizeof(*c),id,sizeof(*id)) || local_owned(s,id,sizeof(*id)) ||
       local_overlap(configuration,size,id,sizeof(*id)) || s->in_work || s->signal.in_callback)return DBT_INVALID;
    if(c->active || s->local || s->pending.used || s->state!=DBT_STREAM_IDLE ||
       s->worker_live || s->configuration || s->retiring || s->drained || s->signal.tx[0].used)return DBT_BUSY;
    if(s->failed || !s->binding || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)return DBT_INVALID;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    int index=endpoint_index(s,local_seid);if(index<0)return DBT_UNSUPPORTED;
    dbt_avdtp_caps_t caps,local;dbt_avdtp_cap_error_t error;
    if(dbt_avdtp_caps_validate(configuration,size,DBT_AVDTP_CONFIGURATION,&caps,&error)!=DBT_OK)return DBT_INVALID;
    const dbt_avdtp_endpoint_t *ep=&s->endpoints[index];
    if(dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK)return DBT_INVALID;
    uint16_t cursor=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(configuration,size,&cursor,&cap)==DBT_OK)
        if(!(local.present&(1u<<cap.category)))return DBT_UNSUPPORTED;
    if(!caps.codec.data || ep->validate(ep,&caps.codec))return DBT_UNSUPPORTED;
    bool delay=(caps.present&(1u<<8))!=0;
    uint16_t bytes=(uint16_t)(4u+caps.codec.size+(delay?2u:0u));
    if(bytes>s->maximum_configuration || (uint32_t)bytes+2u>s->signal.maximum_message)return DBT_INVALID;
    uint8_t *raw=s->hooks.allocate(s->hooks.context,(uint16_t)(bytes+2u));if(!raw)return DBT_NO_SPACE;
    raw[0]=(uint8_t)(remote_seid<<2);raw[1]=(uint8_t)(local_seid<<2);
    raw[2]=1;raw[3]=0;raw[4]=7;raw[5]=caps.codec.size;copy(raw+6,caps.codec.data,caps.codec.size);
    if(delay){raw[bytes]=8;raw[bytes+1u]=0;}
    c->completion=(dbt_avdtp_completion_t){c,local_response};uint8_t label;
    dbt_result_t reserved=dbt_avdtp_command_prepare(&s->signal,DBT_AVDTP_SET_CONFIGURATION,raw,
        (uint16_t)(bytes+2u),now,&label,&c->completion);
    if(reserved!=DBT_OK){s->hooks.release(s->hooks.context,raw);return reserved;}
    /* Signal owns its wire copy; retain a normalized, base-address allocation
     * for worker preparation and eventual committed Get Configuration data. */
    for(unsigned i=0;i<bytes;++i)raw[i]=raw[i+2u];
    uint32_t serial=c->serial+1u;*c=(dbt_avdtp_local_t){0};
    c->completion=(dbt_avdtp_completion_t){c,local_response};c->sink=s;c->serial=serial;c->active=true;c->endpoint=(uint8_t)index;
    c->result.token=serial;c->result.binding=s->binding;c->result.local_seid=local_seid;
    c->result.remote_seid=remote_seid;c->result.label=label;c->result.configuration_size=bytes;
    c->result.operation=DBT_SINK_CONFIGURE;c->result.command=DBT_AVDTP_SET_CONFIGURATION;
    c->result.phase=DBT_AVDTP_LOCAL_PREPARING;s->local=c;
    begin(s,DBT_SINK_CONFIGURE,(uint8_t)index,true,true);
    s->pending.configuration=raw;s->pending.size=bytes;s->pending.remote_seid=remote_seid;
    c->result.worker_token=s->pending.token;c->result.worker_deadline_ms=s->pending.deadline;*id=serial;return DBT_PENDING;
}
dbt_result_t dbt_avdtp_sink_local_reconfigure_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,
    uint32_t media_binding,const uint8_t *configuration,uint16_t size,uint32_t now,uint32_t *id){
    if(!s || !c || !configuration || !id || !size || s->in_work || s->signal.in_callback ||
       local_owned(s,c,sizeof(*c)) || local_owned(s,configuration,size) || local_owned(s,id,sizeof(*id)) ||
       local_overlap(c,sizeof(*c),configuration,size) || local_overlap(c,sizeof(*c),id,sizeof(*id)) ||
       local_overlap(configuration,size,id,sizeof(*id)))return DBT_INVALID;
    if(c->active || s->local || s->pending.used || s->retiring || s->drained || s->signal.tx[0].used)return DBT_BUSY;
    if(s->failed || !s->binding || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)return DBT_INVALID;
    if(s->state!=DBT_STREAM_OPEN || !s->worker_live || !s->configuration || !s->remote_seid ||
       s->endpoint>=s->count || !media_binding || media_binding!=s->media_binding)return DBT_INVALID;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    dbt_avdtp_caps_t caps,local;dbt_avdtp_cap_error_t error;
    if(dbt_avdtp_caps_validate(configuration,size,DBT_AVDTP_RECONFIGURATION,&caps,&error)!=DBT_OK)return DBT_INVALID;
    const dbt_avdtp_endpoint_t *ep=&s->endpoints[s->endpoint];
    if(dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK)return DBT_INVALID;
    uint16_t cursor=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(configuration,size,&cursor,&cap)==DBT_OK)
        if(!(local.present&(1u<<cap.category)))return DBT_UNSUPPORTED;
    if(!caps.codec.data || ep->validate(ep,&caps.codec))return DBT_UNSUPPORTED;
    bool delay=s->delay_enabled;
    uint16_t bytes=(uint16_t)(4u+caps.codec.size+(delay?2u:0u)),wire=(uint16_t)(3u+caps.codec.size);
    if(bytes>s->maximum_configuration || wire>s->signal.maximum_message)return DBT_INVALID;
    uint8_t *raw=s->hooks.allocate(s->hooks.context,bytes);if(!raw)return DBT_NO_SPACE;
    raw[0]=1;raw[1]=(uint8_t)(s->remote_seid<<2);raw[2]=7;raw[3]=caps.codec.size;
    copy(raw+4,caps.codec.data,caps.codec.size);
    if(delay){raw[bytes-2u]=8;raw[bytes-1u]=0;}
    c->completion=(dbt_avdtp_completion_t){c,local_response};uint8_t label;
    /* Reconfigure sends SEID + Codec only; normalized Transport/Delay stay
     * host-side. queue copies this body before the temporary byte is restored. */
    dbt_result_t reserved=dbt_avdtp_command_prepare(&s->signal,DBT_AVDTP_RECONFIGURE,raw+1,wire,now,&label,&c->completion);
    raw[1]=0;
    if(reserved!=DBT_OK){s->hooks.release(s->hooks.context,raw);return reserved;}
    uint32_t serial=c->serial+1u;*c=(dbt_avdtp_local_t){0};
    c->completion=(dbt_avdtp_completion_t){c,local_response};c->sink=s;c->serial=serial;c->active=true;c->endpoint=s->endpoint;
    c->result.token=serial;c->result.binding=s->binding;c->result.local_seid=ep->seid;c->result.remote_seid=s->remote_seid;
    c->result.media_binding=media_binding;c->result.label=label;c->result.configuration_size=bytes;
    c->result.operation=DBT_SINK_RECONFIGURE;c->result.command=DBT_AVDTP_RECONFIGURE;
    c->result.phase=DBT_AVDTP_LOCAL_PREPARING;s->local=c;
    begin(s,DBT_SINK_RECONFIGURE,s->endpoint,true,true);
    s->pending.configuration=raw;s->pending.size=bytes;s->pending.remote_seid=s->remote_seid;
    c->result.worker_token=s->pending.token;c->result.worker_deadline_ms=s->pending.deadline;
    *id=serial;return DBT_PENDING;
}
dbt_result_t dbt_avdtp_sink_local_abort_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,uint32_t media,uint32_t now,uint32_t *id){
    if(!s || !c || !id || s->in_work || s->signal.in_callback || local_owned(s,c,sizeof(*c)) ||
       local_owned(s,id,sizeof(*id)) || local_overlap(c,sizeof(*c),id,sizeof(*id)))return DBT_INVALID;
    if(c->active || s->local || s->pending.used || s->retiring || s->drained || s->signal.tx[0].used)return DBT_BUSY;
    if(s->failed || !s->binding || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)return DBT_INVALID;
    if((s->state!=DBT_STREAM_CONFIGURED && s->state!=DBT_STREAM_OPEN && s->state!=DBT_STREAM_STREAMING && s->state!=DBT_STREAM_CLOSING) ||
       !s->configuration || !s->remote_seid || s->endpoint>=s->count || (s->media_binding && media!=s->media_binding))return DBT_INVALID;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    uint8_t body=(uint8_t)(s->remote_seid<<2),label;
    c->completion=(dbt_avdtp_completion_t){c,local_response};
    dbt_result_t reserved=dbt_avdtp_command_prepare(&s->signal,DBT_AVDTP_ABORT,&body,1,now,&label,&c->completion);
    if(reserved!=DBT_OK)return reserved;
    uint32_t serial=c->serial+1u;*c=(dbt_avdtp_local_t){0};
    c->completion=(dbt_avdtp_completion_t){c,local_response};c->sink=s;c->serial=serial;c->active=true;c->endpoint=s->endpoint;
    c->result.token=serial;c->result.binding=s->binding;c->result.local_seid=s->endpoints[s->endpoint].seid;
    c->result.remote_seid=s->remote_seid;c->result.label=label;c->result.command=DBT_AVDTP_ABORT;
    c->result.media_binding=media;c->result.media_was_pending=media && !s->media_binding;
    c->result.operation=DBT_SINK_ABORT;c->result.configuration_size=s->configuration_size;
    c->result.phase=DBT_AVDTP_LOCAL_PREPARING;c->result.worker_not_needed=!s->worker_live;s->local=c;
    begin(s,DBT_SINK_ABORT,s->endpoint,s->worker_live,true);
    if(s->worker_live){c->result.worker_token=s->pending.token;c->result.worker_deadline_ms=s->pending.deadline;}
    s->state=DBT_STREAM_ABORTING;s->media_deadline=0;*id=serial;return DBT_PENDING;
}
static dbt_result_t local_control_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,dbt_sink_operation_t operation,
    uint32_t media_binding,uint32_t now,uint32_t *id){
    if(!s || !c || !id || s->in_work || s->signal.in_callback || local_owned(s,c,sizeof(*c)) ||
       local_owned(s,id,sizeof(*id)) || local_overlap(c,sizeof(*c),id,sizeof(*id)))return DBT_INVALID;
    if(c->active || s->local || s->pending.used || s->retiring || s->drained || s->signal.tx[0].used)return DBT_BUSY;
    if(s->failed || !s->binding || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)return DBT_INVALID;
    if(!s->worker_live || !s->configuration || !s->remote_seid || s->endpoint>=s->count)return DBT_INVALID;
    uint8_t command;
    if(operation==DBT_SINK_OPEN){
        if(s->state!=DBT_STREAM_CONFIGURED || s->media_binding)return DBT_INVALID;
        command=DBT_AVDTP_OPEN;
    }else if(operation==DBT_SINK_START){
        if(s->state!=DBT_STREAM_OPEN || !media_binding || media_binding!=s->media_binding)return DBT_INVALID;
        command=DBT_AVDTP_START;
    }else if(operation==DBT_SINK_SUSPEND){
        if(s->state!=DBT_STREAM_STREAMING || !media_binding || media_binding!=s->media_binding)return DBT_INVALID;
        command=DBT_AVDTP_SUSPEND;
    }else if(operation==DBT_SINK_CLOSE){
        if((s->state!=DBT_STREAM_OPEN && s->state!=DBT_STREAM_STREAMING) ||
           !media_binding || media_binding!=s->media_binding)return DBT_INVALID;
        command=DBT_AVDTP_CLOSE;
    }else return DBT_INVALID;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    uint8_t body=(uint8_t)(s->remote_seid<<2),label;
    c->completion=(dbt_avdtp_completion_t){c,local_response};
    dbt_result_t reserved=dbt_avdtp_command_prepare(&s->signal,command,&body,1,now,&label,&c->completion);
    if(reserved!=DBT_OK)return reserved;
    uint32_t serial=c->serial+1u;*c=(dbt_avdtp_local_t){0};
    c->completion=(dbt_avdtp_completion_t){c,local_response};c->sink=s;c->serial=serial;c->active=true;c->endpoint=s->endpoint;
    c->result.token=serial;c->result.binding=s->binding;c->result.local_seid=s->endpoints[s->endpoint].seid;
    c->result.remote_seid=s->remote_seid;c->result.label=label;c->result.command=command;c->result.media_binding=media_binding;
    c->result.operation=operation;c->result.configuration_size=s->configuration_size;
    c->result.phase=DBT_AVDTP_LOCAL_PREPARING;s->local=c;
    bool work=operation!=DBT_SINK_SUSPEND && operation!=DBT_SINK_CLOSE;begin(s,operation,s->endpoint,work,true);
    if(work){c->result.worker_token=s->pending.token;c->result.worker_deadline_ms=s->pending.deadline;}
    *id=serial;return DBT_PENDING;
}
dbt_result_t dbt_avdtp_sink_local_open_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,uint32_t now,uint32_t *id){
    return local_control_begin(s,c,DBT_SINK_OPEN,0,now,id);
}
dbt_result_t dbt_avdtp_sink_local_start_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,uint32_t media,uint32_t now,uint32_t *id){
    return local_control_begin(s,c,DBT_SINK_START,media,now,id);
}
dbt_result_t dbt_avdtp_sink_local_suspend_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,uint32_t media,uint32_t now,uint32_t *id){
    return local_control_begin(s,c,DBT_SINK_SUSPEND,media,now,id);
}
dbt_result_t dbt_avdtp_sink_local_close_begin(dbt_avdtp_sink_t *s,dbt_avdtp_local_t *c,uint32_t media,uint32_t now,uint32_t *id){
    return local_control_begin(s,c,DBT_SINK_CLOSE,media,now,id);
}
dbt_result_t dbt_avdtp_local_cancel(dbt_avdtp_local_t *c,uint32_t token){
    if(!c || !token)return DBT_INVALID;
    if(c->serial!=token || !c->sink)return DBT_EMPTY;
    c->result.cancel_requested=true;
    if(c->done)return c->notifying?DBT_PENDING:DBT_OK;
    local_outcome(c,DBT_AVDTP_LOCAL_CANCELLED);return DBT_PENDING;
}
dbt_result_t dbt_avdtp_local_result(const dbt_avdtp_local_t *c,uint32_t token,dbt_avdtp_local_result_t *result){
    if(!c || !token || !result || local_overlap(c,sizeof(*c),result,sizeof(*result)))return DBT_INVALID;
    if(c->serial!=token || !c->sink)return DBT_EMPTY;
    if((c->active || c->notifying) && local_owned(c->sink,result,sizeof(*result)))return DBT_INVALID;
    *result=c->result;return c->done && !c->notifying?DBT_OK:DBT_PENDING;
}

dbt_result_t dbt_avdtp_sink_init(dbt_avdtp_sink_t *s,dbt_avdtp_sink_hooks_t hooks,
    const dbt_avdtp_endpoint_t *endpoints,uint8_t count,uint32_t binding,
    uint16_t rx_mtu,uint16_t tx_mtu,uint16_t maximum_configuration,
    uint32_t signal_ms,uint32_t worker_ms,uint32_t media_ms) {
    if(!s || !hooks.allocate || !hooks.release || !hooks.work || !hooks.media ||
       !endpoints || !count || count>DBT_AVDTP_SINK_ENDPOINT_LIMIT || maximum_configuration<6 ||
       maximum_configuration>1024 || !worker_ms || worker_ms>INT32_MAX || !media_ms || media_ms>INT32_MAX)return DBT_INVALID;
    for(unsigned i=0;i<count;++i){
        const dbt_avdtp_endpoint_t *ep=&endpoints[i];dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
        if(!ep->seid || ep->seid>62 || !ep->validate || ep->size>1024 ||
            ((!ep->raw_codeword_bytes)!=(!ep->raw_codeword_samples)) ||
            dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&caps,&error)!=DBT_OK ||
            (caps.present&0x82u)!=0x82u || (caps.present&~0x182u) || caps.codec.data[0]>>4)return DBT_INVALID;
        for(unsigned j=0;j<i;++j)if(endpoints[j].seid==ep->seid)return DBT_INVALID;
    }
    zero(s,sizeof(*s));s->hooks=hooks;s->endpoints=endpoints;s->count=count;s->binding=binding;
    s->maximum_configuration=maximum_configuration;s->worker_ms=worker_ms;s->media_ms=media_ms;
    dbt_avdtp_hooks_t signaling={s,allocate_signal,release_signal,command,local_complete,responded,interrupt};
    return dbt_avdtp_init(&s->signal,signaling,binding,rx_mtu,tx_mtu,1024,signal_ms,signal_ms);
}

dbt_result_t dbt_avdtp_sink_complete(dbt_avdtp_sink_t *s,uint32_t binding,uint32_t job,
    dbt_sink_work_result_t result,uint32_t now) {
    if(!s || s->in_work || result>DBT_SINK_WORK_FATAL || result<DBT_SINK_WORK_READY)return DBT_INVALID;
    dbt_sink_pending_t *p=&s->pending;dbt_avdtp_local_t *c=s->local;
    /* Cleanup can supersede an accepted preparation token. Retain that late
     * receipt as metadata, without completing or changing the newer job. */
    if(c && binding==s->binding && job==c->result.worker_token && c->result.worker_accepted &&
       !c->result.worker_result_valid && (!p->used || p->token!=job)){
        c->result.worker_result=result;c->result.worker_result_valid=true;
        if(due(now,c->result.worker_deadline_ms))c->result.worker_timed_out=true;
        if(result==DBT_SINK_WORK_FATAL && !c->drop_started && !s->retiring)
            local_failure(s,c,DBT_AVDTP_LOCAL_WORKER_FATAL);
        return s->failed?DBT_INVALID:DBT_OK;
    }
    if(c && binding==s->binding && job==c->result.rollback_token && c->result.rollback_accepted &&
       !c->result.rollback_result_valid && (!p->used || p->token!=job)){
        c->result.rollback_result=result;c->result.rollback_result_valid=true;
        if(due(now,c->result.rollback_deadline_ms))c->result.rollback_timed_out=true;
        return s->failed?DBT_INVALID:DBT_OK;
    }
    if(binding!=s->binding || !p->used || !p->submitted || p->done || job!=p->token)return DBT_EMPTY;
    if(s->local && !s->retiring)return local_worker_complete(s,result,now);
    if(p->operation==DBT_SINK_DROP){
        if(s->local){s->local->result.drop_result=result;s->local->result.drop_result_valid=true;}
        if(result!=DBT_SINK_WORK_READY){fault(s);return DBT_INVALID;}
        s->worker_live=false;drop_pending(s);idle(s);s->drained=true;local_retired(s);emit(s,DBT_SINK_DRAINED);return DBT_OK;
    }
    if(s->retiring)return DBT_EMPTY;
    if(dbt_avdtp_poll(&s->signal,now)!=DBT_OK || due(now,p->deadline)){fault(s);return DBT_INVALID;}
    if(result==DBT_SINK_WORK_FATAL){fault(s);return DBT_INVALID;}
    p->done=true;
    if(result==DBT_SINK_WORK_REJECTED){
        p->apply=false;p->error=(p->operation==DBT_SINK_CONFIGURE || p->operation==DBT_SINK_RECONFIGURE)?0x29:0x31;
        p->error_detail=(p->signal==3 || p->signal==5)?7:(uint8_t)(s->endpoints[p->endpoint].seid<<2);
        if(p->operation==DBT_SINK_CONFIGURE)s->worker_live=false;
    }else if(p->operation==DBT_SINK_CLOSE || p->operation==DBT_SINK_ABORT)s->worker_live=false;
    reply_pending(s);return s->failed?DBT_INVALID:DBT_OK;
}

dbt_result_t dbt_avdtp_sink_poll(dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || !s->binding || s->in_work)return DBT_INVALID;
    if(s->drained)return DBT_OK;
    if(!s->retiring && (s->failed || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)){fault(s);return DBT_INVALID;}
    bool media_wait=media_waiting(s);
    if(!s->retiring && media_wait && due(now,s->media_deadline)){
        if(s->local){s->local->result.media_timed_out=true;local_failure(s,s->local,DBT_AVDTP_LOCAL_MEDIA_TIMEOUT);}
        else fault(s);
        return DBT_INVALID;
    }
    if(s->local && !s->retiring){local_poll(s,now);if(s->failed)return DBT_INVALID;}
    dbt_sink_pending_t *p=&s->pending;
    if(p->used && !p->done && due(now,p->deadline)){fault(s);return DBT_INVALID;}
    if(p->used && !p->submitted && !p->done){
        const uint8_t *configuration=p->configuration?p->configuration:s->configuration;
        uint16_t size=p->configuration?p->size:s->configuration_size;
        dbt_sink_job_t request={s->binding,p->token,p->operation,s->endpoints[p->endpoint].seid,configuration,size};
        p->attempted=true;
        s->in_work=true;dbt_result_t result=s->hooks.work(s->hooks.context,&request);s->in_work=false;
        if(s->local){
            dbt_avdtp_local_result_t *r=&s->local->result;
            if(p->operation==DBT_SINK_DROP){r->drop_submit_result=result;if(result==DBT_OK)r->drop_accepted=true;
                else if(result==DBT_BUSY && r->drop_busy<UINT16_MAX)++r->drop_busy;}
            else if(s->local->rollback_started && p->token==r->rollback_token){r->rollback_submit_result=result;if(result==DBT_OK)r->rollback_accepted=true;
                else if(result==DBT_BUSY && r->rollback_busy<UINT16_MAX)++r->rollback_busy;}
            else {r->worker_submit_result=result;if(result==DBT_OK)r->worker_accepted=true;
                else if(result==DBT_BUSY && r->worker_busy<UINT16_MAX)++r->worker_busy;}
        }
        if(result==DBT_OK){p->submitted=true;s->worker_live=true;}
        else if(result!=DBT_BUSY){
            if(p->operation==DBT_SINK_DROP){fault(s);return DBT_INVALID;}
            p->submitted=true;
            /* Enqueue refusal accepted no worker state; CONFIGURE has none. */
            (void)dbt_avdtp_sink_complete(s,s->binding,p->token,DBT_SINK_WORK_REJECTED,now);
        }
    }
    reply_pending(s);return s->failed?DBT_INVALID:DBT_OK;
}

dbt_result_t dbt_avdtp_sink_media_open(dbt_avdtp_sink_t *s,uint32_t binding,uint32_t now) {
    if(!s || !binding || s->retiring || s->failed || s->state!=DBT_STREAM_OPEN || s->pending.used || s->media_binding)return DBT_INVALID;
    if(due(now,s->media_deadline)){fault(s);return DBT_INVALID;}
    s->media_binding=binding;s->media_deadline=0;s->media_packets=s->media_rejected=0;
    s->raw_sequence=0;s->raw_timestamp=0;return DBT_OK;
}
dbt_result_t dbt_avdtp_sink_media_closed_ex(dbt_avdtp_sink_t *s,uint32_t binding,bool transport_failed) {
    if(!s || !binding)return DBT_EMPTY;
    bool pending=s->local && s->local->result.operation==DBT_SINK_ABORT && s->local->result.media_was_pending &&
        s->local->result.media_binding==binding && !s->media_binding;
    if(s->media_binding!=binding && !pending)return DBT_EMPTY;
    s->media_binding=0;if(s->local)s->local->result.media_closed=true;
    if(transport_failed){
        if(s->local){s->local->result.media_closed=true;s->local->result.media_close_failed=true;local_failure(s,s->local,DBT_AVDTP_LOCAL_MEDIA_ERROR);}
        else fault(s);
        return DBT_INVALID;
    }
    if(s->local && s->local->result.remote_abort){
        s->local->result.media_closed=true;s->local->result.phase=DBT_AVDTP_LOCAL_CLEANUP;s->state=DBT_STREAM_ABORTING;
        emit(s,DBT_SINK_STATE);return DBT_OK;
    }
    if(s->local && s->local->result.operation==DBT_SINK_ABORT){
        dbt_avdtp_local_t *c=s->local;s->state=DBT_STREAM_ABORTING;
        c->result.phase=local_accepted(c,DBT_SINK_ABORT)?
            (s->pending.done?DBT_AVDTP_LOCAL_CLEANUP:DBT_AVDTP_LOCAL_APPLYING):DBT_AVDTP_LOCAL_WAIT_PEER;
        emit(s,DBT_SINK_STATE);return DBT_OK;
    }
    if(s->local && s->local->result.operation==DBT_SINK_CLOSE && s->local->wire_released){
        dbt_avdtp_local_t *c=s->local;c->result.media_closed=true;s->state=DBT_STREAM_CLOSING;
        c->result.phase=local_accepted(c,DBT_SINK_CLOSE)?
            (c->result.worker_result_valid?DBT_AVDTP_LOCAL_CLEANUP:DBT_AVDTP_LOCAL_APPLYING):DBT_AVDTP_LOCAL_WAIT_PEER;
        emit(s,DBT_SINK_STATE);return DBT_OK;
    }
    if(s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING){idle(s);emit(s,DBT_SINK_STATE);return DBT_OK;}
    if(s->local)local_failure(s,s->local,DBT_AVDTP_LOCAL_MEDIA_ERROR);else fault(s);
    return DBT_INVALID;
}
dbt_result_t dbt_avdtp_sink_media_closed(dbt_avdtp_sink_t *s,uint32_t binding){
    return dbt_avdtp_sink_media_closed_ex(s,binding,false);
}
dbt_result_t dbt_avdtp_sink_media_receive(dbt_avdtp_sink_t *s,uint32_t binding,const uint8_t *packet,uint16_t size) {
    if(!s || !binding || binding!=s->media_binding)return DBT_EMPTY;
    if(s->failed || s->retiring || s->state!=DBT_STREAM_STREAMING)return DBT_BUSY;
    if(s->pending.used && s->pending.operation!=DBT_SINK_START){
        const dbt_avdtp_local_t *c=s->local;
        if(!c || (c->result.operation!=DBT_SINK_SUSPEND && c->result.operation!=DBT_SINK_CLOSE) ||
           c->result.remote_abort || local_accepted(c,c->result.operation))return DBT_BUSY;
    }
    if(s->endpoint>=s->count)return DBT_INVALID;
    const dbt_avdtp_endpoint_t *ep=&s->endpoints[s->endpoint];
    dbt_avdtp_media_t media;dbt_result_t result;
    if(ep->raw_codeword_bytes){
        result=(!packet || !size || size%ep->raw_codeword_bytes)?DBT_INVALID:DBT_OK;
        if(result==DBT_OK){
            media=(dbt_avdtp_media_t){packet,s->raw_timestamp,0,size,s->raw_sequence++,DBT_AVDTP_RAW_PAYLOAD_TYPE,false};
            s->raw_timestamp+=(uint32_t)(size/ep->raw_codeword_bytes)*ep->raw_codeword_samples;
        }
    }else result=dbt_avdtp_media_decode(packet,size,&media);
    if(result==DBT_OK)result=s->hooks.media(s->hooks.context,&media);
    if(result==DBT_OK)++s->media_packets;else ++s->media_rejected;
    return result;
}
dbt_result_t dbt_avdtp_sink_delay(dbt_avdtp_sink_t *s,uint16_t delay,uint32_t now) {
    if(!s || s->failed || s->retiring || !s->delay_enabled || s->state==DBT_STREAM_IDLE ||
       s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING)return DBT_INVALID;
    uint8_t body[3]={(uint8_t)(s->remote_seid<<2),(uint8_t)(delay>>8),(uint8_t)delay},label=0;
    return dbt_avdtp_command(&s->signal,DBT_AVDTP_DELAY_REPORT,body,3,now,&label);
}
uint32_t dbt_avdtp_sink_wait_ms(const dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || s->drained || (!s->retiring && s->failed))return UINT32_MAX;
    uint32_t wait=s->retiring?UINT32_MAX:dbt_avdtp_wait_ms(&s->signal,now);
    uint32_t setup=local_wait(s);if(setup<wait)wait=setup;
    const dbt_sink_pending_t *p=&s->pending;
    if(p->used && !p->done){
        if(!p->attempted)return 0;
        if(!(s->retiring && s->failed && due(now,p->deadline))){
            uint32_t next=until(now,p->deadline);if(next<wait)wait=next;
        }
    }
    bool media_wait=media_waiting(s);
    if(!s->retiring && media_wait){
        uint32_t next=until(now,s->media_deadline);if(next<wait)wait=next;
    }
    /* Work queue availability is a provider wakeup; BUSY must not spin. */
    return wait;
}
dbt_result_t dbt_avdtp_sink_shutdown(dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || !s->binding || s->in_work || s->signal.in_callback)return DBT_INVALID;
    if(s->drained)return DBT_OK;
    if(s->signal.pinned)return DBT_BUSY;
    if(!s->retiring){
        s->retiring=true;
        if(s->signal.binding && dbt_avdtp_clear(&s->signal)!=DBT_OK)return DBT_BUSY;
        drop_pending(s);
        if(!s->worker_live){idle(s);s->drained=true;local_retired(s);emit(s,DBT_SINK_DRAINED);return DBT_OK;}
        dbt_sink_pending_t *p=&s->pending;p->used=true;p->operation=DBT_SINK_DROP;
        p->endpoint=s->local?s->local->endpoint:s->endpoint;p->token=token(s);p->deadline=now+s->worker_ms;
        if(s->local)s->local->result.drop_token=p->token;
    }
    return DBT_BUSY;
}
