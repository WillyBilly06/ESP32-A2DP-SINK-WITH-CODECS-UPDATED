#include "probe_product_io_owner.h"
#include "probe_gatt.h"
#include "probe_avrcp.h"
#include "probe_output.h"
#include <string.h>

/* Kept independent of the decoder's FreeRTOS-facing interface. */
extern dbt_result_t probe_audio_peer(uint16_t *,uint32_t *);
extern const dbt_link_t *probe_audio_peer_link(uint16_t,uint32_t);
enum { PAIR_BEGIN,PAIR_SUBMIT,PAIR_STATUS,PAIR_DISCONNECT,PAIR_SCAN,PAIR_SCAN_WAIT };
enum { PAIR_PENDING,PAIR_ACCEPTED,PAIR_REJECTED,PAIR_CANCELLED };
static bool retry(dbt_result_t r){return r==DBT_BUSY || r==DBT_NO_SPACE;}
static void pair_done(void *context,uint16_t opcode,dbt_command_outcome_t outcome,
                      uint8_t status,const uint8_t *returns,uint8_t size){
    probe_product_io_owner_t *s=context;(void)status;(void)returns;(void)size;
    if(s->pair_phase!=PAIR_STATUS || opcode!=0x0406)return;
    s->pair_token=0;
    s->pair_status=outcome==DBT_COMMAND_SUCCESS?PAIR_ACCEPTED:
        outcome==DBT_COMMAND_ABORTED && s->pair_gone?PAIR_CANCELLED:PAIR_REJECTED;
}
void probe_product_io_owner_link(probe_product_io_owner_t *s,const dbt_link_change_t *change){
    if(!s || !change || change->kind!=DBT_LINK_REMOVED || !s->pair_generation ||
       change->handle!=s->pair_handle || change->generation!=s->pair_generation)return;
    s->pair_gone=true;
    if(s->pair_phase==PAIR_STATUS && s->pair_token && s->gap && s->gap->host)
        (void)dbt_host_cancel_command(s->gap->host,s->pair_token);
}
static dbt_result_t pairing(probe_product_io_owner_t *s,uint32_t now){
    dbt_gap_t *g=s->gap;
    if(!g || !g->host || g->state==DBT_GAP_FAILED)return DBT_INVALID;
    if(s->pair_phase==PAIR_BEGIN){
        dbt_result_t r=probe_audio_peer(&s->pair_handle,&s->pair_generation);
        if(r==DBT_EMPTY){s->pair_generation=0;s->pair_phase=PAIR_SCAN;}
        else if(r!=DBT_OK)return r;
        else {s->pair_gone=false;s->pair_phase=PAIR_SUBMIT;}
    }
    if(s->pair_phase==PAIR_SUBMIT){
        const dbt_link_t *link=probe_audio_peer_link(s->pair_handle,s->pair_generation);
        if(!link){s->pair_gone=true;s->pair_phase=PAIR_SCAN;}
        else {
            if(g->state!=DBT_GAP_READY)return DBT_BUSY;
            uint8_t bytes[3]={(uint8_t)link->handle,(uint8_t)(link->handle>>8),0x13};
            s->pair_status=PAIR_PENDING;s->pair_phase=PAIR_STATUS;
            dbt_result_t r=dbt_host_command_tracked(g->host,0x0406,bytes,3,DBT_COMMAND_STATUS,
                                                     pair_done,s,now,&s->pair_token);
            if(r!=DBT_OK){s->pair_phase=PAIR_SUBMIT;return r;}
            return DBT_BUSY;
        }
    }
    if(s->pair_phase==PAIR_STATUS){
        if(s->pair_status==PAIR_PENDING)return DBT_BUSY;
        if(s->pair_status==PAIR_REJECTED)return DBT_INVALID;
        if(s->pair_status==PAIR_CANCELLED || s->pair_gone ||
           !probe_audio_peer_link(s->pair_handle,s->pair_generation))s->pair_phase=PAIR_SCAN;
        else {s->pair_deadline=now+5000u;s->pair_phase=PAIR_DISCONNECT;}
    }
    if(s->pair_phase==PAIR_DISCONNECT){
        if(s->pair_gone || !probe_audio_peer_link(s->pair_handle,s->pair_generation))s->pair_phase=PAIR_SCAN;
        else return (int32_t)(now-s->pair_deadline)>=0?DBT_INVALID:DBT_BUSY;
    }
    if(s->pair_phase==PAIR_SCAN_WAIT){
        if(g->classic_pending)return DBT_BUSY;
        if(g->classic_scan!=3)return DBT_INVALID;
        s->pair_phase=PAIR_BEGIN;s->pair_generation=0;return DBT_OK;
    }
    if(s->pair_phase==PAIR_SCAN){
        if(g->state!=DBT_GAP_READY || g->classic_pending || g->rename_active)return DBT_BUSY;
        if(g->classic_scan==3){s->pair_phase=PAIR_BEGIN;s->pair_generation=0;return DBT_OK;}
        dbt_result_t r=dbt_gap_set_classic_scan(g,true,true);
        if(r!=DBT_OK)return r;
        s->pair_phase=PAIR_SCAN_WAIT;return DBT_BUSY;
    }
    return DBT_INVALID;
}
static dbt_result_t input_apply(probe_product_io_owner_t *s,uint32_t now) {
    dbt_input_action_t *a=&s->input;
    if(a->kind>=DBT_INPUT_EQ_BASS && a->kind<=DBT_INPUT_EQ_TREBLE)
        return probe_gatt_control_eq_delta((uint8_t)(a->kind-DBT_INPUT_EQ_BASS),a->delta);
    if(a->kind==DBT_INPUT_VOLUME) {
        probe_avrcp_snapshot_t snapshot;dbt_result_t r=probe_avrcp_snapshot(&snapshot);if(r!=DBT_OK)return r;
        int v=(int)snapshot.volume+a->delta;if(v<0)v=0;if(v>127)v=127;
        r=probe_output_volume((uint8_t)v);if(r==DBT_OK)probe_avrcp_local_volume((uint8_t)v);return r;
    }
    if(a->kind==DBT_INPUT_PLAY_PAUSE || a->kind==DBT_INPUT_NEXT || a->kind==DBT_INPUT_PREVIOUS) {
        if(!s->key_released) {
            uint8_t op=a->kind==DBT_INPUT_NEXT?0x4b:a->kind==DBT_INPUT_PREVIOUS?0x4c:0;
            if(!op){probe_avrcp_snapshot_t view;dbt_result_t r=probe_avrcp_snapshot(&view);if(r!=DBT_OK)return r;op=view.playback_status==1?0x46:0x44;}
            dbt_result_t r=probe_avrcp_key(op,false,now);if(r!=DBT_OK)return r;
            s->key_released=true;s->key_operation=op;s->key_due=now+70u;return DBT_BUSY;
        }
        if((int32_t)(now-s->key_due)<0)return DBT_BUSY;
        dbt_result_t r=probe_avrcp_key(s->key_operation,true,now);
        if(!retry(r))s->key_released=false;
        return r;
    }
    if(a->kind==DBT_INPUT_BRIGHTNESS || a->kind==DBT_INPUT_EFFECT_PREVIEW ||
       a->kind==DBT_INPUT_EFFECT_CONFIRM || a->kind==DBT_INPUT_EFFECT_NEXT) {
        if(s->led_serial==UINT32_MAX)return DBT_INVALID;
        dbt_led_operation_t op=a->kind==DBT_INPUT_BRIGHTNESS?DBT_LED_ADJUST_BRIGHTNESS:
            a->kind==DBT_INPUT_EFFECT_PREVIEW?DBT_LED_PREVIEW_DELTA:
            a->kind==DBT_INPUT_EFFECT_CONFIRM?DBT_LED_CONFIRM_PREVIEW:DBT_LED_ADJUST_EFFECT;
        int8_t delta=a->kind==DBT_INPUT_EFFECT_CONFIRM?0:a->kind==DBT_INPUT_EFFECT_NEXT?1:a->delta;
        uint32_t token=s->led_serial+1u;dbt_result_t r=probe_product_io_local_led_relative(op,delta,token);
        if(r==DBT_OK){s->led_serial=token;s->led_pending_token=token;s->led_pending=true;}
        return r;
    }
    if(a->kind==DBT_INPUT_BASS_BOOST || a->kind==DBT_INPUT_CHANNEL_FLIP || a->kind==DBT_INPUT_BYPASS)
        return probe_gatt_control_toggle(a->kind==DBT_INPUT_BASS_BOOST?1u:a->kind==DBT_INPUT_CHANNEL_FLIP?2u:4u);
    if(a->kind==DBT_INPUT_PAIR)return pairing(s,now);
    return DBT_UNSUPPORTED;
}
void probe_product_io_owner_poll(probe_product_io_owner_t *s,uint32_t now) {
    if(!s)return;
    (void)probe_gatt_control_worker_required(probe_product_io_needs_worker());
    dbt_led_receipt_t led;
    for(unsigned i=0;i<4 && probe_product_io_take_local_led(&led);i++) {
        if(s->led_pending && led.token==s->led_pending_token)s->led_pending=false;
        if(led.result!=DBT_LED_OK)s->failures++;
    }
    probe_product_io_play_result_t play;
    for(unsigned i=0;i<4 && probe_product_io_take_play_result(&play);i++)if(play.result!=DBT_OK)s->failures++;
    dbt_led_settings_t snapshot;if(probe_product_io_led_snapshot(&snapshot))(void)probe_gatt_control_led_applied(snapshot.bytes);
    if(s->sound_enabled)(void)probe_gatt_control_sound_applied(probe_product_io_sound_status());
    for(unsigned i=0;i<4;i++) {
        if(!s->have_notice)s->have_notice=probe_product_io_take_notice(&s->notice_peer,s->notice,&s->notice_size);
        if(!s->have_notice)break;
        dbt_result_t r=probe_gatt_control_notify(&s->notice_peer,s->notice,s->notice_size);
        if(retry(r))break;
        if(r!=DBT_OK && r!=DBT_UNSUPPORTED)s->failures++;
        s->have_notice=false;
    }
    for(unsigned i=0;i<8 && !s->led_pending;i++) {
        if(!s->have_input)s->have_input=probe_product_io_take_input(&s->input);
        if(!s->have_input)break;
        dbt_result_t r=input_apply(s,now);if(retry(r))break;
        if(r==DBT_UNSUPPORTED || r==DBT_EMPTY)s->unsupported++;else if(r!=DBT_OK)s->failures++;
        if(s->input.kind==DBT_INPUT_PAIR){s->pair_phase=PAIR_BEGIN;s->pair_generation=0;}
        s->have_input=false;
    }
    (void)probe_gatt_control_worker_required(probe_product_io_needs_worker());
}
uint32_t probe_product_io_owner_wait_ms(const probe_product_io_owner_t *s,uint32_t now) {
    if(!s)return UINT32_MAX;
    if(s->key_released && (int32_t)(now-s->key_due)<0)return s->key_due-now;
    return s->have_input || s->led_pending || s->have_notice?20u:UINT32_MAX;
}
