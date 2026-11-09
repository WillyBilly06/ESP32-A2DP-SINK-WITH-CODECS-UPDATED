#include "probe_flash_guard.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"IRQ observation requires lock-free words");
static DRAM_ATTR struct {
    _Atomic uint32_t enabled,token,begin_us;
    _Atomic uint32_t peak_token,peak_begin_us,peak_elapsed_us;
} flash_guard;
_Static_assert(sizeof(flash_guard)==24,"Flash observer state budget");
/* Linker aliases preserve the SDK's complete guard and ordering. No guard
 * table, cache register, interrupt policy or SDK implementation is replaced. */
extern void __real_spi_flash_disable_interrupts_caches_and_other_cpu(void);
extern void __real_spi_flash_enable_interrupts_caches_and_other_cpu(void);
void probe_flash_guard_enable(void){
    uint32_t unused=0;(void)atomic_compare_exchange_strong(&flash_guard.enabled,&unused,1);
}
void IRAM_ATTR __wrap_spi_flash_disable_interrupts_caches_and_other_cpu(void){
    /* The SDK takes the flash-operation lock inside this call. Publish only
     * after it returns: callers waiting for that lock must not reuse tokens
     * or replace the active owner's metadata. */
    __real_spi_flash_disable_interrupts_caches_and_other_cpu();
    if(atomic_load(&flash_guard.enabled)==1u){
        uint32_t previous=atomic_load(&flash_guard.token);
        if((previous&~7u)==0xfffffff8u){
            atomic_store(&flash_guard.enabled,2);atomic_store(&flash_guard.token,0);
        }else {
            uint32_t serial=(previous&~7u)+8u;
            atomic_store(&flash_guard.token,serial|1u);
            atomic_store(&flash_guard.begin_us,(uint32_t)esp_timer_get_time());
            atomic_store(&flash_guard.token,serial|2u|((uint32_t)xPortGetCoreID()<<2));
        }
    }
}
void IRAM_ATTR __wrap_spi_flash_enable_interrupts_caches_and_other_cpu(void){
    uint32_t previous=atomic_load(&flash_guard.token);
    if((previous&3u)==2u){
        uint32_t begin=atomic_load(&flash_guard.begin_us);
        uint32_t elapsed=(uint32_t)esp_timer_get_time()-begin;
        if(elapsed>atomic_load(&flash_guard.peak_elapsed_us)){
            /* Serialized SDK owner, before restore/unlock. Readers accept
             * the record only between unchanged finished guard tokens. */
            atomic_store(&flash_guard.peak_token,previous);
            atomic_store(&flash_guard.peak_begin_us,begin);
            atomic_store(&flash_guard.peak_elapsed_us,elapsed);
        }
    }
    /* One strong attempt: a later guard can never be cleared by this end.
     * The SPI1 driver serializes these nonrecursive guard calls. */
    (void)atomic_compare_exchange_strong(&flash_guard.token,&previous,previous&~3u);
    __real_spi_flash_enable_interrupts_caches_and_other_cpu();
}
void IRAM_ATTR probe_flash_guard_sample(probe_flash_guard_sample_t *out){
    if(!out)return;
    uint32_t before=atomic_load(&flash_guard.token);
    out->begin_us=atomic_load(&flash_guard.begin_us);
    out->scheduler_state=(uint32_t)xTaskGetSchedulerState();
    out->token=before;
    out->coherent=before && (before&3u)!=1u && before==atomic_load(&flash_guard.token);
}
/* Background-only one-attempt read. The existing live token is the writer
 * sequence: all peak updates precede its finished publication. A new guard
 * invalidates this observation rather than making the reader wait. */
void probe_flash_guard_peak(probe_flash_guard_peak_t *out){
    if(!out)return;
    uint32_t before=atomic_load(&flash_guard.token);
    out->token=atomic_load(&flash_guard.peak_token);
    out->begin_us=atomic_load(&flash_guard.peak_begin_us);
    out->elapsed_us=atomic_load(&flash_guard.peak_elapsed_us);
    out->coherent=before && !(before&3u) && out->token &&
        before==atomic_load(&flash_guard.token);
}
