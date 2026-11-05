#include <stdlib.h>
#include <string.h>
#include "../../esp32_probe/main/probe_output.c"
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static unsigned live,queues_created;
static request_t pending_request;
static response_t pending_response;
static bool request_pending,response_pending;
static int64_t test_clock_us;
static int64_t test_clock_step;
static unsigned configured_irq_priority;
static unsigned allocation_attempt,fail_at_allocation;
static unsigned decoder_wakes;
static void wake_compact(void *unused){(void)unused;++decoder_wakes;}
void *heap_caps_malloc(size_t n,unsigned flags){(void)flags;if(++allocation_attempt==fail_at_allocation)return 0;void *p=malloc(n);if(p)live++;return p;}
void heap_caps_free(void *p){if(p){live--;free(p);}}
int64_t esp_timer_get_time(void){int64_t value=test_clock_us;test_clock_us+=test_clock_step;return value;}
void xTaskNotifyGive(TaskHandle_t t){
    (void)t;if(!request_pending)return;request_pending=false;
    dbt_result_t r;
    if(pending_request.start)r=start_output(pending_request.rate,pending_request.channels,pending_request.format);
    else {close_output();r=DBT_OK;}
    report_done(pending_request.id,r);
}
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){(void)t;(void)w;}
unsigned ulTaskNotifyTake(int b,unsigned t){(void)b;(void)t;return 0;}
/* Metadata plumbing only; this host mock cannot measure an ESP32 stack. */
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t t){(void)t;return 4096;}
int xTaskCreatePinnedToCore(void (*f)(void *),const char *n,unsigned s,void *a,unsigned p,TaskHandle_t *t,int core){(void)f;(void)n;(void)s;(void)a;(void)p;(void)core;*t=(void *)3;return pdPASS;}
QueueHandle_t xQueueCreate(unsigned n,unsigned s){(void)n;(void)s;return (void *)(uintptr_t)(++queues_created);}
int xQueueSend(QueueHandle_t q,const void *p,unsigned t){
    (void)t;if(q==(void *)1){pending_request=*(const request_t *)p;request_pending=true;return pdTRUE;}
    if(q==(void *)2){pending_response=*(const response_t *)p;response_pending=true;return pdTRUE;}return pdFALSE;
}
int xQueueReceive(QueueHandle_t q,void *p,unsigned t){
    (void)t;if(q==(void *)2 && response_pending){*(response_t *)p=pending_response;response_pending=false;return pdTRUE;}return pdFALSE;
}
esp_err_t i2s_new_channel(const i2s_chan_config_t *c,i2s_chan_handle_t *ch,void *rx){configured_irq_priority=(unsigned)c->intr_priority;(void)rx;*ch=(void *)1;return ESP_OK;}
esp_err_t i2s_del_channel(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t ch,const i2s_std_config_t *c){(void)ch;(void)c;return ESP_OK;}
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t ch,const i2s_event_callbacks_t *cb,void *ctx){(void)ch;(void)cb;(void)ctx;return ESP_OK;}
esp_err_t i2s_channel_disable(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_enable(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t ch,const void *p,size_t n,size_t *loaded){(void)ch;(void)p;*loaded=n;return ESP_OK;}
EX int probe_output_formats(void){
    CHECK(event_wait_ticks(2)>0);
    CHECK(event_wait_ticks(0)==1);
    CHECK(probe_output_init(0,0)==DBT_OK);
    CHECK(render_wait_ticks()==portMAX_DELAY);
    static const uint8_t sample16[]={0,0x80,0xff,0x7f,1,2,3,4};
    static const uint8_t sample24[]={0,0,0x80,0xff,0xff,0x7f,1,2,3,0xfd,0xfe,0xff};
    static const uint8_t sample32[]={0,0,0,0x80,0xff,0xff,0xff,0x7f,1,2,3,4,5,6,7,8};
    const uint8_t *samples[]={sample16,sample24,sample32};
    for(unsigned i=0;i<3;i++){
        dbt_pcm_format_t format=(dbt_pcm_format_t)(i+2);size_t bytes=(i+2)*4u;
        atomic_store(&output.descriptor_shape_errors,9);atomic_store(&output.descriptor_pool_errors,8);atomic_store(&output.descriptor_order_errors,7);
        atomic_store(&output.descriptor_expected_index,6);atomic_store(&output.descriptor_observed_index,5);atomic_store(&output.descriptor_observed_size,4);
        atomic_store(&output.descriptor_fault_at_us,3);atomic_store(&output.descriptor_fault_gap_us,2);atomic_store(&output.descriptor_fault_stopping,1);
        CHECK(probe_output_stream_format(true,48000,2,format)==DBT_OK && output.audio.config.format==format && output.audio.input.stride==(i+2)*2u);
        CHECK(output.audio.config.output_frames==3584 && output.audio.config.target_frames==3072);
        CHECK(configured_irq_priority==2 && !atomic_load(&output.callback_calls) && !atomic_load(&output.callback_elapsed_us) && !atomic_load(&output.callback_max_us));
        CHECK(!atomic_load(&output.descriptor_shape_errors) && !atomic_load(&output.descriptor_pool_errors) && !atomic_load(&output.descriptor_order_errors));
        CHECK(!atomic_load(&output.descriptor_expected_index) && !atomic_load(&output.descriptor_observed_index) && !atomic_load(&output.descriptor_observed_size));
        CHECK(!atomic_load(&output.descriptor_fault_at_us) && !atomic_load(&output.descriptor_fault_gap_us) && !atomic_load(&output.descriptor_fault_stopping));
        CHECK(probe_output_pcm_format(samples[i],2,2,48000,format)==DBT_OK && !memcmp(output.audio.input.bytes,samples[i],bytes));
        CHECK(probe_output_pcm_format(samples[i],2,2,44100,format)==DBT_INVALID);
        CHECK(probe_output_pcm_format(samples[i],2,1,48000,format)==DBT_INVALID);
        CHECK(probe_output_pcm_format(samples[i],2,2,48000,format==DBT_PCM_S32_LE?DBT_PCM_S16_LE:DBT_PCM_S32_LE)==DBT_INVALID);
        CHECK(probe_output_pcm_format(0,2,2,48000,format)==DBT_INVALID);
        CHECK(probe_output_stream_format(false,0,0,0)==DBT_OK && !live && !output.active);
    }
    CHECK(probe_output_stream(true,48000,2)==DBT_OK && output.audio.config.format==DBT_PCM_S16_LE);
    CHECK(probe_output_pcm((const int16_t *)sample16,2,2,48000)==DBT_OK);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && !live);
    CHECK(probe_output_stream_format(true,48000,2,(dbt_pcm_format_t)1)==DBT_INVALID);
    CHECK(probe_output_delay_100us(48000)==747 && probe_output_delay_100us(0)==0);
    CHECK(probe_output_stream_format(true,0,2,DBT_PCM_S16_LE)==DBT_INVALID);
    CHECK(probe_output_stream_format(true,48000,3,DBT_PCM_S16_LE)==DBT_INVALID);
    CHECK(probe_output_stream_format(true,UINT32_MAX,2,DBT_PCM_S16_LE)==DBT_INVALID);
    const uint32_t rates[]={96000,192000},capacities[]={6656,12800},bytes[]={86016,167936};
    for(unsigned i=0;i<2;++i){
        CHECK(probe_output_stream_format(true,rates[i],2,DBT_PCM_S32_LE)==DBT_OK);
        CHECK(output.audio.config.output_frames==capacities[i] && output.audio.allocated_bytes==bytes[i]);
        CHECK(output.audio.config.target_frames==rates[i]*64u/1000u);
        CHECK(probe_output_stream(false,0,0)==DBT_OK && !live);
    }
    /* A rejected 192 kHz request must not inherit the previous 48 kHz format
     * in failure telemetry, and partially allocated pages must be reclaimed. */
    fail_at_allocation=allocation_attempt+3u;test_clock_us=424242;
    CHECK(probe_output_stream_format(true,192000,2,DBT_PCM_S32_LE)==DBT_NO_SPACE && !live && !output.active);
    probe_output_stats_t stats;probe_output_stats(&stats);
    CHECK(stats.allocation_failed_bytes==4096 && stats.allocation_failure_rate==192000);
    CHECK(stats.allocation_failure_audio_requested==167936 && stats.allocation_failure_audio_allocated==69632 && stats.allocation_failure_pages==1);
    CHECK(stats.allocation_failure_channels==2 && stats.allocation_failure_sample_bytes==4 && stats.allocation_failure_at_us==424242);
    fail_at_allocation=0;
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    probe_output_stats(&stats);
    CHECK(!stats.allocation_failed_bytes && !stats.allocation_failure_rate && !stats.allocation_failure_channels && !stats.allocation_failure_sample_bytes && !stats.allocation_failure_at_us);
    CHECK(!stats.allocation_failure_audio_requested && !stats.allocation_failure_audio_allocated && !stats.allocation_failure_pages);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && !live);test_clock_us=0;
    return 0;
}
EX int probe_output_descriptor_diagnostics(void){
    memset(&output,0,sizeof(output));static int32_t dma[4][DMA_FRAMES*2];
    test_clock_us=123456;output.last_eof=120000;
    i2s_event_data_t event={0};event.dma_buf=dma[0];event.size=0;
    (void)sent(0,&event,&output);
    CHECK(atomic_load(&output.descriptor_errors)==1 && atomic_load(&output.descriptor_shape_errors)==1);
    CHECK(atomic_load(&output.descriptor_observed_index)==UINT32_MAX && !atomic_load(&output.descriptor_observed_size));
    CHECK(atomic_load(&output.descriptor_fault_at_us)==123456 && atomic_load(&output.descriptor_fault_gap_us)==3456 && !atomic_load(&output.descriptor_fault_stopping));
    output.descriptor_count=DMA_DESCRIPTORS;
    for(unsigned i=0;i<DMA_DESCRIPTORS;++i)output.descriptors[i].address=dma[i];
    test_clock_us=200000;event.dma_buf=dma[3];event.size=DMA_FRAMES*8u;(void)sent(0,&event,&output);
    CHECK(atomic_load(&output.descriptor_errors)==2 && atomic_load(&output.descriptor_pool_errors)==1);
    CHECK(atomic_load(&output.descriptor_observed_index)==3 && atomic_load(&output.descriptor_observed_size)==DMA_FRAMES*8u);
    CHECK(atomic_load(&output.descriptor_fault_at_us)==200000 && atomic_load(&output.descriptor_fault_gap_us)==80000);
    test_clock_us=201000;atomic_store(&output.stop_requested,1);event.dma_buf=dma[2];(void)sent(0,&event,&output);
    CHECK(atomic_load(&output.descriptor_errors)==3 && atomic_load(&output.descriptor_order_errors)==1);
    CHECK(!atomic_load(&output.descriptor_expected_index) && atomic_load(&output.descriptor_observed_index)==2);
    CHECK(atomic_load(&output.descriptor_fault_at_us)==201000 && atomic_load(&output.descriptor_fault_gap_us)==1000 && atomic_load(&output.descriptor_fault_stopping)==1);
    CHECK(atomic_load(&output.callback_calls)==3 && !atomic_load(&output.callback_elapsed_us));
    test_clock_us=300000;test_clock_step=10;event.size=0;(void)sent(0,&event,&output);
    CHECK(atomic_load(&output.callback_calls)==4 && atomic_load(&output.callback_elapsed_us)==10 && atomic_load(&output.callback_max_us)==10);
    test_clock_step=0;test_clock_us=0;return 0;
}
#ifdef DBT_PROBE_PCM_COPIED
EX int probe_output_compact_copy(void){
    CHECK(probe_output_init(wake_compact,0)==DBT_OK);
    const uint32_t rates[]={8000,48000,96000,192000},capacities[]={256,256,512,1024};
    const uint32_t payloads[]={13312,17408,23552,39936};
    for(unsigned i=0;i<4;++i)for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels){
        CHECK(probe_output_stream_format(true,rates[i],(uint8_t)channels,(dbt_pcm_format_t)width)==DBT_OK);
        probe_output_stats_t stats;probe_output_stats(&stats);
        CHECK(stats.pcm_input_mode==2 && stats.input_capacity_frames==capacities[i]);
        CHECK(stats.input_ring_bytes==capacities[i]*width*channels && stats.requested_bytes==payloads[i]-capacities[i]*8u+capacities[i]*width*channels);
        CHECK(probe_output_stream_format(false,0,0,0)==DBT_OK && !live);
        probe_output_stats(&stats);CHECK(!stats.input_capacity_frames && !stats.input_ring_bytes);
    }
    CHECK(probe_output_stream_format(true,192000,2,DBT_PCM_S32_LE)==DBT_OK);
    uint8_t raw[960*8],expected[960*8];
    for(unsigned i=0;i<sizeof raw;++i)raw[i]=(uint8_t)(i*37u+11u);
    memcpy(expected,raw,sizeof raw);
    for(unsigned i=0;i<3;++i)CHECK(probe_output_try_pcm_format(raw+256u*8u*i,256,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(probe_output_try_pcm_format(raw+768u*8u,192,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_input_frames(&output.audio)==960 && !memcmp(output.audio.input.bytes,expected,sizeof expected));
    memset(raw,0x5a,sizeof raw); /* Decoder may immediately reuse its buffer. */
    CHECK(!memcmp(output.audio.input.bytes,expected,sizeof expected));
    uint32_t written=atomic_load(&output.audio.input.written);
    CHECK(probe_output_try_pcm_format(raw,256,2,192000,DBT_PCM_S32_LE)==DBT_BUSY && atomic_load(&output.audio.input.written)==written);
    unsigned wakes=decoder_wakes;(void)render_step();CHECK(decoder_wakes>wakes);
    CHECK(probe_output_try_pcm_format(raw,256,2,192000,DBT_PCM_S32_LE)==DBT_BUSY);
    (void)render_step();CHECK(probe_output_try_pcm_format(raw,256,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(probe_output_stream_format(false,0,0,0)==DBT_OK && !live);return 0;
}
#endif
