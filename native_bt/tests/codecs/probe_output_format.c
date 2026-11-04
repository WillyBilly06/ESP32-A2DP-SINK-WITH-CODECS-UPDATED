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
void *heap_caps_malloc(size_t n,unsigned flags){(void)flags;void *p=malloc(n);if(p)live++;return p;}
void heap_caps_free(void *p){if(p){live--;free(p);}}
int64_t esp_timer_get_time(void){return 0;}
void xTaskNotifyGive(TaskHandle_t t){
    (void)t;if(!request_pending)return;request_pending=false;
    dbt_result_t r;
    if(pending_request.start)r=start_output(pending_request.rate,pending_request.channels,pending_request.format);
    else {close_output();r=DBT_OK;}
    report_done(pending_request.id,r);
}
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){(void)t;(void)w;}
unsigned ulTaskNotifyTake(int b,unsigned t){(void)b;(void)t;return 0;}
int xTaskCreatePinnedToCore(void (*f)(void *),const char *n,unsigned s,void *a,unsigned p,TaskHandle_t *t,int core){(void)f;(void)n;(void)s;(void)a;(void)p;(void)core;*t=(void *)3;return pdPASS;}
QueueHandle_t xQueueCreate(unsigned n,unsigned s){(void)n;(void)s;return (void *)(uintptr_t)(++queues_created);}
int xQueueSend(QueueHandle_t q,const void *p,unsigned t){
    (void)t;if(q==(void *)1){pending_request=*(const request_t *)p;request_pending=true;return pdTRUE;}
    if(q==(void *)2){pending_response=*(const response_t *)p;response_pending=true;return pdTRUE;}return pdFALSE;
}
int xQueueReceive(QueueHandle_t q,void *p,unsigned t){
    (void)t;if(q==(void *)2 && response_pending){*(response_t *)p=pending_response;response_pending=false;return pdTRUE;}return pdFALSE;
}
esp_err_t i2s_new_channel(const i2s_chan_config_t *c,i2s_chan_handle_t *ch,void *rx){(void)c;(void)rx;*ch=(void *)1;return ESP_OK;}
esp_err_t i2s_del_channel(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t ch,const i2s_std_config_t *c){(void)ch;(void)c;return ESP_OK;}
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t ch,const i2s_event_callbacks_t *cb,void *ctx){(void)ch;(void)cb;(void)ctx;return ESP_OK;}
esp_err_t i2s_channel_disable(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_enable(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t ch,const void *p,size_t n,size_t *loaded){(void)ch;(void)p;*loaded=n;return ESP_OK;}
EX int probe_output_formats(void){
    CHECK(probe_output_init(0,0)==DBT_OK);
    static const uint8_t sample16[]={0,0x80,0xff,0x7f,1,2,3,4};
    static const uint8_t sample24[]={0,0,0x80,0xff,0xff,0x7f,1,2,3,0xfd,0xfe,0xff};
    static const uint8_t sample32[]={0,0,0,0x80,0xff,0xff,0xff,0x7f,1,2,3,4,5,6,7,8};
    const uint8_t *samples[]={sample16,sample24,sample32};
    for(unsigned i=0;i<3;i++){
        dbt_pcm_format_t format=(dbt_pcm_format_t)(i+2);size_t bytes=(i+2)*4u;
        CHECK(probe_output_stream_format(true,48000,2,format)==DBT_OK && output.audio.config.format==format && output.audio.input.stride==(i+2)*2u);
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
    return 0;
}
