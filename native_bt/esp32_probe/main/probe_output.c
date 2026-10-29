#include "probe_output.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#if !CONFIG_I2S_ISR_IRAM_SAFE
#error "Native output requires an IRAM-safe I2S ISR for immediate NVS commits"
#endif
#define DMA_FRAMES 128u
#define DMA_DESCRIPTORS 3u
typedef struct {bool start;uint32_t rate;uint8_t channels;uint32_t id;} request_t;
typedef struct {uint32_t id;dbt_result_t result;} response_t;
typedef struct {void *address;uint32_t valid;bool silent;} descriptor_t;
typedef struct {
    dbt_audio_t audio;
    dbt_audio_fade_t fade;
    descriptor_t descriptors[DMA_DESCRIPTORS];
    i2s_chan_handle_t channel;
    TaskHandle_t task;
    QueueHandle_t requests;
    QueueHandle_t completed;
    void (*wake_decoder)(void *);
    void *wake_context;
    bool active,enabled,stopping;
    uint32_t descriptor_count,next_descriptor,zero_completions,last_eof,clock_at;
    uint32_t next_request,stop_request,stop_at;
    int32_t held_left,held_right;
    _Atomic uint32_t stop_requested,stop_finished;
    _Atomic uint32_t running,failed,rate,requested_bytes,accepted_frames,rendered_frames,ready_frames,input_ready_frames;
    _Atomic uint32_t dma_eofs,dma_source_frames,underrun_frames,descriptor_errors,max_eof_gap_us;
    _Atomic uint32_t maximum_render_us,stop_fill_frames;
    _Atomic uint32_t first_underrun_accepted_frames,first_underrun_dma_frames;
} output_t;
static DRAM_ATTR output_t output;
static uint8_t *allocate(void *context,uint32_t bytes){(void)context;return heap_caps_malloc(bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void release(void *context,uint8_t *bytes){(void)context;heap_caps_free(bytes);}
static uint32_t power2(uint32_t n){uint32_t result=2;while(result<n)result<<=1;return result;}
static void report_done(uint32_t id,dbt_result_t result){
    response_t response={id,result};
    if(xQueueSend(output.completed,&response,0)!=pdTRUE)atomic_store(&output.failed,1);
}

static bool IRAM_ATTR sent(i2s_chan_handle_t channel,i2s_event_data_t *event,void *context){
    (void)channel;output_t *o=context;BaseType_t wake=pdFALSE;
    if(!event->dma_buf || event->size!=DMA_FRAMES*8u){
        atomic_store(&o->descriptor_errors,atomic_load(&o->descriptor_errors)+1u);atomic_store(&o->failed,1);return false;
    }
    uint32_t now=(uint32_t)esp_timer_get_time();
    if(o->last_eof){uint32_t gap=now-o->last_eof;if(gap>atomic_load(&o->max_eof_gap_us))atomic_store(&o->max_eof_gap_us,gap);}
    o->last_eof=now;
    unsigned index=0;
    while(index<o->descriptor_count && o->descriptors[index].address!=event->dma_buf)++index;
    if(index==o->descriptor_count){
        if(index==DMA_DESCRIPTORS){atomic_store(&o->descriptor_errors,atomic_load(&o->descriptor_errors)+1u);atomic_store(&o->failed,1);return false;}
        o->descriptors[index]=(descriptor_t){event->dma_buf,DMA_FRAMES,false};++o->descriptor_count;
    }
    if(index!=o->next_descriptor){atomic_store(&o->descriptor_errors,atomic_load(&o->descriptor_errors)+1u);atomic_store(&o->failed,1);}
    o->next_descriptor=index+1u;if(o->next_descriptor==DMA_DESCRIPTORS)o->next_descriptor=0;
    descriptor_t *descriptor=&o->descriptors[index];
    atomic_store(&o->dma_eofs,atomic_load(&o->dma_eofs)+1u);
    atomic_store(&o->dma_source_frames,atomic_load(&o->dma_source_frames)+descriptor->valid);
    bool stopping=atomic_load_explicit(&o->stop_requested,memory_order_acquire)!=0;
    if(stopping && descriptor->silent)++o->zero_completions;else o->zero_completions=0;
    if(stopping && o->zero_completions>=DMA_DESCRIPTORS)atomic_store_explicit(&o->stop_finished,1,memory_order_release);
    int32_t *buffer=event->dma_buf;
    uint32_t wanted=stopping && o->fade.remaining<DMA_FRAMES?o->fade.remaining:DMA_FRAMES;
    uint32_t taken=wanted?dbt_audio_take(&o->audio,buffer,wanted):0;
    if(stopping){
        for(unsigned i=taken;i<wanted;++i){buffer[2*i]=o->held_left;buffer[2*i+1u]=o->held_right;}
        atomic_store(&o->stop_fill_frames,atomic_load(&o->stop_fill_frames)+wanted-taken);
    }else{
        if(taken<DMA_FRAMES && !atomic_load(&o->underrun_frames)){
            atomic_store(&o->first_underrun_accepted_frames,atomic_load(&o->accepted_frames));
            atomic_store(&o->first_underrun_dma_frames,atomic_load(&o->dma_source_frames));
        }
        atomic_store(&o->underrun_frames,atomic_load(&o->underrun_frames)+DMA_FRAMES-taken);
    }
    if(taken){o->held_left=buffer[2*(taken-1u)];o->held_right=buffer[2*(taken-1u)+1u];}
    if(!stopping && taken<DMA_FRAMES)o->held_left=o->held_right=0;
    for(unsigned i=wanted;i<DMA_FRAMES;++i)buffer[2*i]=buffer[2*i+1u]=0;
    descriptor->silent=stopping && !o->fade.remaining;
    if(stopping)dbt_audio_fade(&o->fade,buffer,DMA_FRAMES);
    descriptor->valid=taken;
    vTaskNotifyGiveFromISR(o->task,&wake);
    return wake==pdTRUE;
}
static void close_output(void){
    if(output.channel){
        if(output.enabled)(void)i2s_channel_disable(output.channel);
        (void)i2s_del_channel(output.channel);output.channel=0;
    }
    /* All I2S operations and its ISR live on core 0. No callback can still be
     * running on another core when this task frees the internal-SRAM rings. */
    dbt_audio_close(&output.audio);output.active=output.enabled=output.stopping=false;
    atomic_store(&output.running,0);atomic_store(&output.requested_bytes,0);atomic_store(&output.ready_frames,0);
    atomic_store(&output.input_ready_frames,0);
}
static esp_err_t create_channel(uint32_t rate,bool apll){
    i2s_chan_config_t channel=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1,I2S_ROLE_MASTER);
    channel.dma_desc_num=DMA_DESCRIPTORS;channel.dma_frame_num=DMA_FRAMES;
    channel.auto_clear_after_cb=false;channel.auto_clear_before_cb=false;channel.intr_priority=1;
    esp_err_t result=i2s_new_channel(&channel,&output.channel,0);if(result!=ESP_OK)return result;
    i2s_std_config_t config={
        .clk_cfg=I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,I2S_SLOT_MODE_STEREO),
        .gpio_cfg={.mclk=I2S_GPIO_UNUSED,.bclk=CONFIG_NATIVE_AUDIO_BCLK,.ws=CONFIG_NATIVE_AUDIO_LRCK,
                   .dout=CONFIG_NATIVE_AUDIO_DATA,.din=I2S_GPIO_UNUSED}};
    if(apll)config.clk_cfg.clk_src=I2S_CLK_SRC_APLL;
    result=i2s_channel_init_std_mode(output.channel,&config);
    if(result==ESP_OK){i2s_event_callbacks_t callbacks={.on_sent=sent};result=i2s_channel_register_event_callback(output.channel,&callbacks,&output);}
    if(result!=ESP_OK){(void)i2s_del_channel(output.channel);output.channel=0;}
    return result;
}
static dbt_result_t start_output(uint32_t rate,uint8_t channels){
    if(output.active)return DBT_BUSY;
    uint32_t target=(rate*64u+999u)/1000u;
    dbt_audio_config_t config={rate,power2((rate*32u+999u)/1000u),power2(target+DMA_DESCRIPTORS*DMA_FRAMES+128u),
        target,DBT_PCM_S16_LE,channels,true};
    dbt_result_t result=dbt_audio_open(&output.audio,config);if(result!=DBT_OK)return result;
    if(create_channel(rate,true)!=ESP_OK && create_channel(rate,false)!=ESP_OK){close_output();return DBT_INVALID;}
    for(unsigned i=0;i<DMA_DESCRIPTORS;++i)output.descriptors[i]=(descriptor_t){0};
    output.descriptor_count=output.next_descriptor=output.zero_completions=output.last_eof=0;
    output.held_left=output.held_right=0;output.clock_at=(uint32_t)(esp_timer_get_time()/1000);
    dbt_audio_fade_init(&output.fade,rate/50u);
    atomic_store(&output.stop_requested,0);atomic_store(&output.stop_finished,0);
    atomic_store(&output.failed,0);
    atomic_store(&output.rate,rate);atomic_store(&output.accepted_frames,0);atomic_store(&output.rendered_frames,0);
    atomic_store(&output.dma_eofs,0);atomic_store(&output.dma_source_frames,0);atomic_store(&output.underrun_frames,0);
    atomic_store(&output.descriptor_errors,0);atomic_store(&output.max_eof_gap_us,0);atomic_store(&output.maximum_render_us,0);
    atomic_store(&output.stop_fill_frames,0);atomic_store(&output.requested_bytes,output.audio.allocated_bytes+DMA_DESCRIPTORS*DMA_FRAMES*8u);
    atomic_store(&output.first_underrun_accepted_frames,0);atomic_store(&output.first_underrun_dma_frames,0);
    output.active=true;return DBT_OK;
}
static dbt_result_t enable_output(void){
    int32_t block[DMA_FRAMES*2];
    for(unsigned i=0;i<DMA_DESCRIPTORS;++i){
        if(dbt_audio_take(&output.audio,block,DMA_FRAMES)!=DMA_FRAMES)return DBT_INVALID;
        size_t loaded=0;
        if(i2s_channel_preload_data(output.channel,block,sizeof(block),&loaded)!=ESP_OK || loaded!=sizeof(block))return DBT_INVALID;
    }
    output.enabled=i2s_channel_enable(output.channel)==ESP_OK;
    atomic_store(&output.running,output.enabled);return output.enabled?DBT_OK:DBT_INVALID;
}
static void render_loop(void *unused){
    (void)unused;
    for(;;){
        request_t request;
        if(xQueueReceive(output.requests,&request,0)==pdTRUE){
            if(output.stopping)report_done(request.id,DBT_BUSY);
            else if(request.start)report_done(request.id,start_output(request.rate,request.channels));
            else if(!output.active || !output.enabled){close_output();report_done(request.id,DBT_OK);}
            else{output.stopping=true;output.stop_request=request.id;output.stop_at=(uint32_t)(esp_timer_get_time()/1000);
                atomic_store_explicit(&output.stop_requested,1,memory_order_release);}
        }
        if(output.active){
            if(output.stopping && atomic_load_explicit(&output.stop_finished,memory_order_acquire)){
                close_output();report_done(output.stop_request,DBT_OK);continue;
            }
            if(output.stopping && (uint32_t)(esp_timer_get_time()/1000)-output.stop_at>500){
                close_output();atomic_store(&output.failed,1);report_done(output.stop_request,DBT_INVALID);continue;
            }
            if(atomic_load(&output.failed)){ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));continue;}
            uint32_t target=output.audio.config.target_frames+(output.enabled?0:DMA_DESCRIPTORS*DMA_FRAMES);
            uint32_t available=dbt_audio_output_frames(&output.audio);
            if(available<target){
                uint32_t began=(uint32_t)esp_timer_get_time();uint32_t budget=target-available;if(budget>128)budget=128;
                uint32_t made=dbt_audio_render(&output.audio,budget);
                if(made){
                    uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
                    if(elapsed>atomic_load(&output.maximum_render_us))atomic_store(&output.maximum_render_us,elapsed);
                    atomic_store(&output.rendered_frames,atomic_load(&output.audio.rendered_frames));
                    if(output.wake_decoder)output.wake_decoder(output.wake_context);
                }
            }
            available=dbt_audio_output_frames(&output.audio);atomic_store(&output.ready_frames,available);
            atomic_store(&output.input_ready_frames,dbt_audio_input_frames(&output.audio));
            if(!output.enabled && available>=target){
                if(enable_output()!=DBT_OK)atomic_store(&output.failed,1);
            }
            uint32_t now=(uint32_t)(esp_timer_get_time()/1000);
            if(output.enabled && !output.stopping && now-output.clock_at>=500){dbt_audio_clock(&output.audio);output.clock_at=now;}
            if(output.active && dbt_audio_output_frames(&output.audio)<target && dbt_audio_input_frames(&output.audio)>1)continue;
        }
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));
    }
}
dbt_result_t probe_output_init(void (*wake_decoder)(void *),void *context){
    output.wake_decoder=wake_decoder;output.wake_context=context;
    dbt_audio_memory_t memory={0,allocate,release};if(dbt_audio_init(&output.audio,memory)!=DBT_OK)return DBT_INVALID;
    output.requests=xQueueCreate(2,sizeof(request_t));output.completed=xQueueCreate(4,sizeof(response_t));
    if(!output.requests || !output.completed)return DBT_NO_SPACE;
    return xTaskCreatePinnedToCore(render_loop,"native_render",6144,0,6,&output.task,0)==pdPASS?DBT_OK:DBT_NO_SPACE;
}
dbt_result_t probe_output_stream(bool start,uint32_t rate,uint8_t channels){
    if(!++output.next_request)++output.next_request;
    request_t request={start,rate,channels,output.next_request};
    response_t response;
    while(xQueueReceive(output.completed,&response,0)==pdTRUE){} /* stale timed-out receipts */
    if(xQueueSend(output.requests,&request,pdMS_TO_TICKS(100))!=pdTRUE)return DBT_BUSY;
    xTaskNotifyGive(output.task);
    uint32_t began=(uint32_t)(esp_timer_get_time()/1000);
    for(;;){
        uint32_t elapsed=(uint32_t)(esp_timer_get_time()/1000)-began;
        if(elapsed>=1500)return DBT_BUSY;
        if(xQueueReceive(output.completed,&response,pdMS_TO_TICKS(1500-elapsed))==pdTRUE && response.id==request.id)return response.result;
    }
}
dbt_result_t probe_output_pcm(const int16_t *samples,uint16_t frames,uint8_t channels,uint32_t rate){
    if(atomic_load(&output.failed) || !output.active || output.audio.config.rate!=rate || output.audio.config.channels!=channels)return DBT_INVALID;
    uint32_t began=(uint32_t)(esp_timer_get_time()/1000);
    for(;;){
        dbt_result_t result=dbt_audio_submit(&output.audio,samples,frames);
        if(result==DBT_OK){atomic_fetch_add(&output.accepted_frames,frames);xTaskNotifyGive(output.task);return result;}
        if(result!=DBT_BUSY || (uint32_t)(esp_timer_get_time()/1000)-began>=100)return result;
        xTaskNotifyGive(output.task);ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(2));
    }
}
void probe_output_stats(probe_output_stats_t *s){
    if(!s)return;
#define LOAD(field) s->field=atomic_load(&output.field)
    LOAD(running);LOAD(failed);LOAD(rate);LOAD(requested_bytes);LOAD(accepted_frames);LOAD(rendered_frames);LOAD(ready_frames);LOAD(input_ready_frames);
    LOAD(dma_eofs);LOAD(dma_source_frames);LOAD(underrun_frames);LOAD(descriptor_errors);LOAD(max_eof_gap_us);
    LOAD(maximum_render_us);LOAD(stop_fill_frames);
    LOAD(first_underrun_accepted_frames);LOAD(first_underrun_dma_frames);
#undef LOAD
}
uint16_t probe_output_delay_100us(uint32_t rate){
    if(!rate)return 0;
    uint32_t frames=atomic_load(&output.ready_frames)+atomic_load(&output.input_ready_frames)+DMA_DESCRIPTORS*DMA_FRAMES+80u;
    if(!atomic_load(&output.running))frames=(rate*64u+999u)/1000u+DMA_DESCRIPTORS*DMA_FRAMES+80u;
    uint32_t value=(uint32_t)(((uint64_t)frames*10000u+rate-1u)/rate);return value>UINT16_MAX?UINT16_MAX:(uint16_t)value;
}
dbt_result_t probe_output_controls(int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags){
    return dbt_audio_controls(&output.audio,bass,mid,treble,volume,flags);
}
