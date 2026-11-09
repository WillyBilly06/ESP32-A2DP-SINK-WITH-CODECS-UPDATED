#include "dbt_storage_reserve.h"
#include <string.h>
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_POINTER_LOCK_FREE==2,"Storage gate words must be lock-free");
#ifdef ESP_PLATFORM
_Static_assert(sizeof(dbt_storage_reserve_t)<=112,"Storage gate static SRAM budget");
#endif
#define PHASE_MASK 15u
static unsigned phase(uint32_t word){return word&PHASE_MASK;}
static void publish(dbt_storage_reserve_t *r,uint32_t word,unsigned next){
    atomic_store_explicit(&r->state,(word&~PHASE_MASK)|next,memory_order_release);
}
static void add(_Atomic uint32_t *p,uint32_t n){atomic_store(p,atomic_load(p)+n);}
static void maximum(_Atomic uint32_t *p,uint32_t n){if(n>atomic_load(p))atomic_store(p,n);}
static bool cancelled(const dbt_storage_reserve_t *r){return (atomic_load_explicit(&r->owner,memory_order_acquire)&1u)!=0;}
static void wake_worker(dbt_storage_reserve_t *r){r->ops->wake_worker(r->context,r->waiter);}
static void detached(dbt_storage_reserve_t *r,uint32_t word,dbt_result_t result){
    r->result=result;publish(r,word,DBT_STORAGE_DETACHED);wake_worker(r);
}
dbt_result_t dbt_storage_reserve_init(dbt_storage_reserve_t *r,const dbt_storage_reserve_ops_t *ops,void *context){
    if(!r || !ops || !ops->allocate || !ops->release || !ops->wake_render || !ops->wake_worker ||
       !ops->clock_us || !ops->guard_enter || !ops->guard_exit)return DBT_INVALID;
    memset(r,0,sizeof *r);r->ops=ops;r->context=context;
    atomic_init(&r->state,0);atomic_init(&r->owner,0);
    atomic_init(&r->published_word,0);atomic_init(&r->published_us,0);
    atomic_init(&r->requests,0);atomic_init(&r->claims,0);atomic_init(&r->refused,0);atomic_init(&r->regrants,0);
    atomic_init(&r->live_bytes,0);atomic_init(&r->peak_bytes,0);atomic_init(&r->max_age_us,0);atomic_init(&r->max_wait_us,0);
    return DBT_OK;
}
void dbt_storage_reserve_cancel(dbt_storage_reserve_t *r,const void *cookie){
    if(!r || !r->ops || !cookie || ((uintptr_t)cookie&1u))return;
    uintptr_t expected=(uintptr_t)cookie;
    if(atomic_compare_exchange_strong_explicit(&r->owner,&expected,expected|1u,memory_order_acq_rel,memory_order_acquire))
        r->ops->wake_render(r->context);
}
static void free_pages(dbt_storage_reserve_t *r){
    for(unsigned i=0;i<DBT_STORAGE_RESERVE_PAGES;++i)if(r->pages[i]){
        r->ops->release(r->context,r->pages[i]);r->pages[i]=0;add(&r->live_bytes,(uint32_t)-(int32_t)(DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u));
    }
}
static dbt_result_t finish(dbt_storage_reserve_t *r,uint32_t word){
    dbt_result_t result=r->result;
    free_pages(r);
    atomic_store_explicit(&r->owner,0,memory_order_release);
    publish(r,word,DBT_STORAGE_IDLE);r->ops->wake_render(r->context);return result;
}
dbt_result_t dbt_storage_reserve_prepare(dbt_storage_reserve_t *r,const void *cookie,uintptr_t waiter){
    if(!r || !r->ops || !cookie || ((uintptr_t)cookie&1u) || !waiter)return DBT_INVALID;
    uint32_t word=atomic_load_explicit(&r->state,memory_order_acquire);
    unsigned p=phase(word);
    if(p==DBT_STORAGE_IDLE){
        if((word&~PHASE_MASK)==0xfffffff0u)return DBT_NO_SPACE;
        r->waiter=waiter;r->request_us=r->ops->clock_us(r->context);
        atomic_store_explicit(&r->owner,(uintptr_t)cookie,memory_order_release);add(&r->requests,1);
        atomic_store_explicit(&r->state,((word&~PHASE_MASK)+16u)|DBT_STORAGE_REQUEST,memory_order_release);
        r->ops->wake_render(r->context);return DBT_PENDING;
    }
    if((atomic_load_explicit(&r->owner,memory_order_acquire)&~(uintptr_t)1)!=(uintptr_t)cookie)return DBT_BUSY;
    if(p==DBT_STORAGE_ALLOCATE){
        r->result=DBT_OK;
        if(!cancelled(r))for(unsigned i=0;i<r->needed;++i){
            r->pages[i]=r->ops->allocate(r->context,DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u);
            if(!r->pages[i]){r->result=DBT_NO_SPACE;break;}
            add(&r->live_bytes,DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u);maximum(&r->peak_bytes,atomic_load(&r->live_bytes));
        }
        publish(r,word,DBT_STORAGE_READY);r->ops->wake_render(r->context);return DBT_PENDING;
    }
    if(p==DBT_STORAGE_GRANTED){
        if(cancelled(r)){r->result=DBT_INVALID;publish(r,word,DBT_STORAGE_RELEASE);r->ops->wake_render(r->context);return DBT_PENDING;}
        /* Unattached pages belong to the worker. A stop during allocation
         * no longer holds them across a no-audio storage job. Attached pages
         * have already been cleared from this mailbox by the renderer. */
        if(!r->target)free_pages(r);
        uint32_t now=r->ops->clock_us(r->context),age=now-r->grant_us;
        if(r->target && age>DBT_STORAGE_GRANT_MAX_AGE_US){
            add(&r->regrants,1);publish(r,word,DBT_STORAGE_PREFILL);r->ops->wake_render(r->context);return DBT_PENDING;
        }
        maximum(&r->max_age_us,r->target?age:0);maximum(&r->max_wait_us,now-r->request_us);add(&r->claims,1);
        publish(r,word,DBT_STORAGE_RUNNING);return DBT_OK;
    }
    if(p==DBT_STORAGE_DETACHED){add(&r->refused,1);return finish(r,word);}
    return p==DBT_STORAGE_RUNNING?DBT_BUSY:DBT_PENDING;
}
void dbt_storage_reserve_release(dbt_storage_reserve_t *r){
    if(!r || !r->ops)return;
    uint32_t word=atomic_load_explicit(&r->state,memory_order_acquire);
    if(phase(word)==DBT_STORAGE_RUNNING){publish(r,word,DBT_STORAGE_RELEASE);r->ops->wake_render(r->context);}
}
dbt_result_t dbt_storage_reserve_retire(dbt_storage_reserve_t *r){
    if(!r || !r->ops)return DBT_INVALID;
    uint32_t word=atomic_load_explicit(&r->state,memory_order_acquire);
    if(phase(word)==DBT_STORAGE_IDLE)return DBT_OK;
    if(phase(word)==DBT_STORAGE_DETACHED){(void)finish(r,word);return DBT_OK;}
    return DBT_PENDING;
}
static void grant(dbt_storage_reserve_t *r,uint32_t word,uint32_t observed_us){
    r->grant_us=observed_us;r->result=DBT_OK;
    publish(r,word,DBT_STORAGE_GRANTED);wake_worker(r);
}
void dbt_storage_reserve_published(dbt_storage_reserve_t *r,const dbt_audio_t *a,uint32_t epoch){
    if(!r || !r->ops || !a)return;
    uint32_t word=atomic_load_explicit(&r->state,memory_order_acquire);
    if(phase(word)!=DBT_STORAGE_PREFILL || r->epoch!=epoch || !r->target)return;
    /* Timestamp before the occupancy load: DMA may consume immediately after
     * that load, so every later grant retains the original age bound. The
     * producer holds the existing DSP borrow through this publication. */
    uint32_t observed=r->ops->clock_us(r->context);
    if(dbt_audio_output_frames(a)<r->target)return;
    atomic_store_explicit(&r->published_us,observed,memory_order_relaxed);
    atomic_store_explicit(&r->published_word,word,memory_order_release);
}
dbt_result_t dbt_storage_reserve_render(dbt_storage_reserve_t *r,dbt_audio_t *a,uint32_t epoch,bool running,bool stopping){
    if(!r || !r->ops)return DBT_OK;
    uint32_t word=atomic_load_explicit(&r->state,memory_order_acquire);
    unsigned p=phase(word);
    if(p==DBT_STORAGE_REQUEST){
        if(cancelled(r)){detached(r,word,DBT_INVALID);return DBT_OK;}
        if(!running){r->target=0;grant(r,word,r->ops->clock_us(r->context));return DBT_OK;}
        if(stopping || !a || !a->opened || a->render_pending)return DBT_OK;
        uint64_t target=((uint64_t)a->config.rate*DBT_STORAGE_RESERVE_MS+999u)/1000u;
        if(target<a->config.target_frames)target=a->config.target_frames;
        uint64_t frames=((target+128u+DBT_AUDIO_OUTPUT_PAGE_FRAMES-1u)/DBT_AUDIO_OUTPUT_PAGE_FRAMES)*DBT_AUDIO_OUTPUT_PAGE_FRAMES;
        uint32_t needed=frames>a->output.frames?(uint32_t)(frames-a->output.frames)/DBT_AUDIO_OUTPUT_PAGE_FRAMES:0;
        if(frames>32768u || a->output.frames%DBT_AUDIO_OUTPUT_PAGE_FRAMES || needed>DBT_STORAGE_RESERVE_PAGES){detached(r,word,DBT_NO_SPACE);return DBT_OK;}
        r->base_frames=a->output.frames;r->target=(uint32_t)target;r->rate=a->config.rate;r->epoch=epoch;r->needed=needed;r->result=DBT_OK;
        publish(r,word,needed?DBT_STORAGE_ALLOCATE:DBT_STORAGE_PREFILL);
        if(needed)wake_worker(r);
        return DBT_OK;
    }
    if(p==DBT_STORAGE_READY){
        if(cancelled(r) || r->result!=DBT_OK){detached(r,word,cancelled(r)?DBT_INVALID:r->result);return DBT_OK;}
        if(!running){r->target=0;grant(r,word,r->ops->clock_us(r->context));return DBT_OK;}
        if(stopping || !a || !a->opened || a->render_pending)return DBT_OK;
        if(epoch!=r->epoch || a->output.frames!=r->base_frames){detached(r,word,DBT_NO_SPACE);return DBT_OK;}
        uint32_t mask=r->ops->guard_enter(r->context);
        dbt_result_t result=dbt_audio_output_extend(a,r->pages,r->needed);
        r->ops->guard_exit(r->context,mask);
        if(result==DBT_BUSY)return DBT_OK;
        if(result!=DBT_OK){detached(r,word,result);return DBT_OK;}
        r->attached=r->needed;
        for(unsigned i=0;i<r->needed;++i)r->pages[i]=0;
        publish(r,word,DBT_STORAGE_PREFILL);return DBT_OK;
    }
    if(p==DBT_STORAGE_PREFILL){
        if(cancelled(r)){r->result=DBT_INVALID;publish(r,word,DBT_STORAGE_RELEASE);r->ops->wake_render(r->context);return DBT_OK;}
        if(!running){r->target=0;grant(r,word,r->ops->clock_us(r->context));return DBT_OK;}
        if(stopping || !a || !a->opened || a->render_pending)return DBT_OK;
        if(epoch!=r->epoch){r->result=DBT_NO_SPACE;publish(r,word,DBT_STORAGE_RELEASE);r->ops->wake_render(r->context);return DBT_OK;}
        uint32_t observed=r->ops->clock_us(r->context);
        if(dbt_audio_output_frames(a)>=r->target)grant(r,word,observed);
        else if(atomic_load_explicit(&r->published_word,memory_order_acquire)==word){
            uint32_t published=atomic_load_explicit(&r->published_us,memory_order_relaxed);
            if(observed-published<=DBT_STORAGE_GRANT_MAX_AGE_US)grant(r,word,published);
        }
        return DBT_OK;
    }
    if(p==DBT_STORAGE_RELEASE){
        if(r->attached){
            if(!a || !a->opened || a->render_pending)return DBT_OK;
            if(a->output.frames<r->base_frames ||
               a->output.frames-r->base_frames!=r->attached*DBT_AUDIO_OUTPUT_PAGE_FRAMES)return DBT_INVALID;
            uint32_t n=0,mask=r->ops->guard_enter(r->context);
            dbt_result_t result=dbt_audio_output_trim(a,r->base_frames,r->pages,DBT_STORAGE_RESERVE_PAGES,&n);
            r->ops->guard_exit(r->context,mask);
            if(result==DBT_BUSY)return DBT_OK;
            if(result!=DBT_OK || n!=r->attached)return DBT_INVALID; /* Caller must close; never free ambiguously live pages. */
            r->attached=0;
        }
        detached(r,word,r->result);return DBT_OK;
    }
    return DBT_OK;
}
void dbt_storage_reserve_closed(dbt_storage_reserve_t *r){
    if(r && r->ops){
        if(r->attached){r->attached=0;atomic_store(&r->live_bytes,0);}
        if(phase(atomic_load(&r->state))!=DBT_STORAGE_IDLE)r->ops->wake_render(r->context);
    }
}
uint32_t dbt_storage_reserve_target(const dbt_storage_reserve_t *r,uint32_t epoch){
    if(!r || !r->ops)return 0;
    unsigned p=phase(atomic_load_explicit(&r->state,memory_order_acquire));
    return (p==DBT_STORAGE_PREFILL || p==DBT_STORAGE_GRANTED || p==DBT_STORAGE_RUNNING) && r->epoch==epoch?r->target:0;
}
bool dbt_storage_reserve_blocks_start(const dbt_storage_reserve_t *r){
    return r && r->ops && phase(atomic_load_explicit(&r->state,memory_order_acquire))!=DBT_STORAGE_IDLE;
}
void dbt_storage_reserve_stats(const dbt_storage_reserve_t *r,dbt_storage_reserve_stats_t *out){
    if(!out)return;
    *out=(dbt_storage_reserve_stats_t){0};if(!r || !r->ops)return;
    out->state=atomic_load(&r->state);
#define LOAD(n) out->n=atomic_load(&r->n)
    LOAD(requests);LOAD(claims);LOAD(refused);LOAD(regrants);LOAD(live_bytes);LOAD(peak_bytes);LOAD(max_age_us);LOAD(max_wait_us);
#undef LOAD
}
