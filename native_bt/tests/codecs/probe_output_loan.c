#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../../esp32_probe/main/probe_output.c"
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#include <sanitizer/asan_interface.h>
#define POISON(p,n) __asan_poison_memory_region(p,n)
#define UNPOISON(p,n) __asan_unpoison_memory_region(p,n)
#endif
#endif
#ifndef POISON
#define POISON(p,n) ((void)0)
#define UNPOISON(p,n) ((void)0)
#endif
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static request_t queue[2];static unsigned queued,notifications,allocations,releases,live,errors,fail_allocation;
static void *blocks[70];static int64_t clock_us;
static unsigned irq_priority;static uint32_t configured_rate;static bool cancel_completion;
void *heap_caps_malloc(size_t n,unsigned caps){
    (void)caps;if(++allocations==fail_allocation)return 0;
    void *p=malloc(n);if(!p)return 0;
    for(unsigned i=0;i<70;++i)if(!blocks[i]){blocks[i]=p;++live;return p;}
    ++errors;free(p);return 0;
}
void heap_caps_free(void *p){
    if(!p)return;
    for(unsigned i=0;i<70;++i)if(blocks[i]==p){blocks[i]=0;--live;++releases;free(p);return;}
    ++errors;
}
int64_t esp_timer_get_time(void){return clock_us++;}
void xTaskNotifyGive(TaskHandle_t task){(void)task;++notifications;}
void vTaskNotifyGiveFromISR(TaskHandle_t task,BaseType_t *wake){(void)task;(void)wake;++notifications;}
unsigned ulTaskNotifyTake(int clear,unsigned ticks){(void)clear;(void)ticks;return 0;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task){(void)task;return 4096;}
int xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)fn;(void)name;(void)stack;(void)arg;(void)priority;(void)core;*task=(void *)3;return pdPASS;
}
QueueHandle_t xQueueCreate(unsigned count,unsigned size){if(count!=2 || size!=sizeof(request_t))++errors;return (void *)1;}
int xQueueSend(QueueHandle_t handle,const void *item,unsigned wait){
    if(handle!=(void *)1 || wait){++errors;return pdFALSE;}
    if(queued==2)return pdFALSE;queue[queued++]=*(const request_t *)item;return pdTRUE;
}
int xQueueReceive(QueueHandle_t handle,void *item,unsigned wait){
    if(handle!=(void *)1 || wait){++errors;return pdFALSE;}
    if(!queued)return pdFALSE;*(request_t *)item=queue[0];if(--queued)queue[0]=queue[1];return pdTRUE;
}
esp_err_t i2s_new_channel(const i2s_chan_config_t *cfg,i2s_chan_handle_t *channel,void *rx){(void)rx;irq_priority=(unsigned)cfg->intr_priority;*channel=(void *)1;return ESP_OK;}
esp_err_t i2s_del_channel(i2s_chan_handle_t c){(void)c;return ESP_OK;}
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t c,const i2s_std_config_t *cfg){(void)c;configured_rate=cfg->clk_cfg.sample_rate_hz;return ESP_OK;}
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t c,const i2s_event_callbacks_t *cb,void *p){(void)c;(void)cb;(void)p;return ESP_OK;}
esp_err_t i2s_channel_disable(i2s_chan_handle_t c){(void)c;return ESP_OK;}
esp_err_t i2s_channel_enable(i2s_chan_handle_t c){(void)c;return ESP_OK;}
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t c,const void *p,size_t size,size_t *loaded){(void)c;(void)p;*loaded=size;return ESP_OK;}
void dbt_output_test_step(unsigned point){
    if(point==6 && cancel_completion){
        cancel_completion=false;
        if(output.audio.input_view || output.inflight.pcm || probe_output_pcm_cancel(output.inflight.id)!=DBT_PENDING)++errors;
    }
}
static bool open_stream(uint32_t rate,dbt_pcm_format_t format){
    uint32_t id=0;if(probe_output_stream_begin(true,rate,2,format,&id)!=DBT_PENDING || !id)return false;
    (void)render_step();return probe_output_request_poll(id)==DBT_OK;
}
static bool stop_stream(void){
    uint32_t id=0;if(probe_output_stream_begin(false,0,0,0,&id)!=DBT_PENDING)return false;
    (void)render_step();return probe_output_request_poll(id)==DBT_OK;
}
static int formats_and_allocations(void){
#if DBT_PROBE_PCM_TARGET_MS == 16
    const unsigned payload=28672,pages=7,delay48=267;
#elif DBT_PROBE_PCM_TARGET_MS == 32
    const unsigned payload=53248,pages=13,delay48=427;
#else
    const unsigned payload=102400,pages=25,delay48=747;
#endif
    uint8_t raw[256*8];memset(raw,0x31,sizeof(raw));
    CHECK(probe_output_init(0,0)==DBT_OK);
    CHECK(probe_output_delay_100us(48000)==delay48);
    for(unsigned width=2;width<=4;++width){
        CHECK(open_stream(192000,(dbt_pcm_format_t)width));
        CHECK(output.audio.borrowed_input && !output.audio.input.bytes && output.audio.allocated_bytes==payload && live==pages && irq_priority==2 && configured_rate==192000);
        probe_output_stats_t stats;probe_output_stats(&stats);
        CHECK(stats.pcm_loan_mode==1 && stats.pcm_target_ms==DBT_PROBE_PCM_TARGET_MS && stats.input_ring_bytes_avoided==8192u*2u*width && stats.requested_bytes==payload+3072);
#ifdef DBT_PROBE_BLOCK_DSP
        CHECK(stats.block_dsp==1);
#else
        CHECK(stats.block_dsp==0);
#endif
        CHECK(probe_output_try_pcm_format(raw,1,2,192000,(dbt_pcm_format_t)width)==DBT_INVALID);
        uint32_t id=0;CHECK(probe_output_pcm_begin(raw,256,2,192000,(dbt_pcm_format_t)width,&id)==DBT_PENDING);
        (void)render_step();CHECK(output.pcm_loan_id==id && output.audio.input_view==raw && probe_output_pcm_poll(id)==DBT_PENDING);
        uint32_t stop=9;CHECK(probe_output_stream_begin(false,0,0,0,&stop)==DBT_BUSY && !stop);
        (void)render_step();CHECK(!output.audio.input_view && !output.pcm_loan_id && probe_output_pcm_poll(id)==DBT_OK);
        POISON(raw,sizeof(raw));(void)render_step();CHECK(stop_stream() && !live && !errors);UNPOISON(raw,sizeof(raw));
    }
    unsigned before=releases;fail_allocation=allocations+2u;
    uint32_t id=0;CHECK(probe_output_stream_begin(true,192000,2,DBT_PCM_S32_LE,&id)==DBT_PENDING);
    (void)render_step();CHECK(probe_output_request_poll(id)==DBT_NO_SPACE && !live && releases==before+1u && !output.audio.input_view);
    probe_output_stats_t failed;probe_output_stats(&failed);
    CHECK(failed.allocation_failed_bytes==4096 && failed.allocation_failure_audio_requested==payload);
    CHECK(failed.allocation_failure_audio_allocated==4096 && failed.allocation_failure_pages==1);
    fail_allocation=allocations+1u;
    CHECK(probe_output_stream_begin(true,192000,2,DBT_PCM_S32_LE,&id)==DBT_PENDING);
    (void)render_step();CHECK(probe_output_request_poll(id)==DBT_NO_SPACE && !live);
    probe_output_stats(&failed);CHECK(failed.allocation_failure_audio_allocated==0 && failed.allocation_failure_pages==0);
    CHECK(failed.allocation_failure_audio_requested==payload && failed.allocation_failure_rate==192000);
    fail_allocation=0;return 0;
}
static int cancellation_and_identity(void){
    uint8_t raw[256*8];memset(raw,0x52,sizeof(raw));
    for(unsigned stage=0;stage<4;++stage){
        CHECK(open_stream(48000,DBT_PCM_S32_LE));uint32_t id=0;
        CHECK(probe_output_pcm_begin(raw,stage==2?1:256,2,48000,DBT_PCM_S32_LE,&id)==DBT_PENDING);
        if(stage==2)cancel_completion=true;
        if(stage!=0)(void)render_step();
        if(stage==3){(void)render_step();CHECK(dbt_ticket_state(&output.ticket,id)==DBT_TICKET_DONE);}
        if(stage!=2)CHECK(probe_output_pcm_cancel(id)==DBT_PENDING);
        (void)render_step();CHECK(probe_output_pcm_poll(id)==DBT_CANCELLED && !output.audio.input_view && !output.pcm_loan_id && output.active);
        CHECK(probe_output_pcm_poll(id)==DBT_INVALID && probe_output_pcm_cancel(id)==DBT_INVALID);
        POISON(raw,sizeof(raw));CHECK(stop_stream() && !live && !errors);UNPOISON(raw,sizeof(raw));
    }
    CHECK(open_stream(48000,DBT_PCM_S32_LE));uint32_t id=0;
    CHECK(probe_output_pcm_begin(raw,256,2,44100,DBT_PCM_S32_LE,&id)==DBT_PENDING);
    (void)render_step();CHECK(probe_output_pcm_poll(id)==DBT_CANCELLED && !output.audio.input_view && !atomic_load(&output.pcm_loans_begun));
    uint32_t old=id;CHECK(probe_output_pcm_begin(raw,256,2,48000,DBT_PCM_S32_LE,&id)==DBT_PENDING);(void)render_step();
    CHECK(probe_output_pcm_poll(old)==DBT_INVALID && probe_output_pcm_cancel(old)==DBT_INVALID && output.pcm_loan_id==id && output.audio.input_view==raw);
    i2s_event_data_t bad={0};(void)sent(0,&bad,&output);(void)render_step();
    CHECK(probe_output_pcm_poll(id)==DBT_CANCELLED && !output.audio.input_view && atomic_load(&output.pcm_loans_cancelled)==1);
    POISON(raw,sizeof(raw));CHECK(stop_stream() && !live && !errors);UNPOISON(raw,sizeof(raw));return 0;
}
static int carry_and_restart(void){
    int32_t frame[2]={123456789,-123456789};CHECK(open_stream(48000,DBT_PCM_S32_LE));
    output.audio.fraction=UINT32_MAX-1u;output.audio.step_adjust=2147483;
    uint32_t id=0;CHECK(probe_output_pcm_begin(frame,1,2,48000,DBT_PCM_S32_LE,&id)==DBT_PENDING);
    (void)render_step();CHECK(probe_output_pcm_poll(id)==DBT_OK && !output.audio.input_view && output.audio.input_carry_valid && dbt_audio_input_frames(&output.audio)==1);
    POISON(frame,sizeof(frame));(void)render_step();UNPOISON(frame,sizeof(frame));frame[0]=314159;frame[1]=-271828;
    CHECK(probe_output_pcm_begin(frame,1,2,48000,DBT_PCM_S32_LE,&id)==DBT_PENDING);(void)render_step();
    CHECK(probe_output_pcm_poll(id)==DBT_OK && !output.audio.input_view && !dbt_audio_input_frames(&output.audio));
    probe_output_restart_hold(true);id=99;
    CHECK(probe_output_pcm_begin(frame,1,2,48000,DBT_PCM_S32_LE,&id)==DBT_BUSY && !id);
    CHECK(stop_stream() && !live && !errors);probe_output_restart_hold(false);return 0;
}
int main(void){
    CHECK(!formats_and_allocations());CHECK(!cancellation_and_identity());CHECK(!carry_and_restart());
    CHECK(!live && !queued && !errors);puts("PASS actual PCM-loan renderer: formats, 64KiB input omission, ack/cancel phases, stale ids, DMA fault, carry, OOM and restart");return 0;
}
