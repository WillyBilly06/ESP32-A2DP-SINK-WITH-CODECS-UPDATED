#include "probe_product_io_owner.h"
#include "probe_avrcp.h"
#include <assert.h>
#include <string.h>
static dbt_input_action_t queue[32];static unsigned head,tail,eq_attempts,eq_success,notices,led_sends,sound_updates;
static int8_t steps[8];static bool eq_block,notice_block,required,led_available,notice_available,led_receipt_available;static unsigned flags;
static dbt_led_settings_t settings={{3,50,2,1,2,3,4,5,6,1}};static dbt_led_receipt_t receipt;
static uint8_t volume=50,last_op,brightness_raw=128;static unsigned key_down,key_up;
static uint8_t sound_status=0x85,last_sound;static bool sound_busy,play_result_ready,play_block;
static dbt_result_t sound_result;static unsigned sound_sends,sound_cancels,pair_modes,pair_success,volume_feedback;
bool probe_audio_sound_memory_ok(void){return true;}
bool probe_product_io_sound_busy(void){return sound_busy;}
dbt_result_t probe_product_io_play(uint8_t type,uint32_t rate,bool exclusive){
    assert(rate>=8000 && rate<=192000 && (type!=3 || !exclusive));if(play_block)return DBT_BUSY;
    last_sound=type;sound_sends++;return DBT_OK;
}
void probe_product_io_cancel_sound(void){sound_cancels++;}
void probe_product_io_pairing_mode(bool enabled){if(enabled)pair_modes++;}
void probe_product_io_pairing_success(void){pair_success++;}
void probe_product_io_volume_feedback(uint8_t v){assert(v<=127);volume_feedback++;}
static dbt_link_t active_link;static bool audio_active;static unsigned disconnect_count,scan_count;
static dbt_result_t enqueue_result=DBT_OK,scan_result=DBT_OK;
static dbt_command_done_t done;static void *done_context;static bool sent;
dbt_result_t probe_audio_peer(uint16_t *h,uint32_t *g){if(!audio_active)return DBT_EMPTY;*h=active_link.handle;*g=active_link.generation;return DBT_OK;}
const dbt_link_t *probe_audio_peer_link(uint16_t h,uint32_t g){return active_link.used && active_link.kind==DBT_LINK_CLASSIC && active_link.handle==h && active_link.generation==g?&active_link:0;}
dbt_result_t dbt_host_command_tracked(dbt_host_t *host,uint16_t op,const uint8_t *p,uint8_t n,dbt_command_response_t response,dbt_command_done_t cb,void *ctx,uint32_t now,uint32_t *token){
    (void)host;(void)now;assert(op==0x0406 && n==3 && response==DBT_COMMAND_STATUS && p[0]==(uint8_t)active_link.handle && p[1]==(uint8_t)(active_link.handle>>8) && p[2]==0x13);
    if(enqueue_result!=DBT_OK)return enqueue_result;
    assert(!done);done=cb;done_context=ctx;*token=++disconnect_count;sent=false;return DBT_OK;
}
dbt_command_cancel_result_t dbt_host_cancel_command(dbt_host_t *host,uint32_t token){
    (void)host;assert(token && done);if(sent)return DBT_COMMAND_ALREADY_SUBMITTED;
    dbt_command_done_t cb=done;done=0;cb(done_context,0x0406,DBT_COMMAND_ABORTED,0,0,0);return DBT_COMMAND_CANCELLED;
}
static void status(dbt_command_outcome_t outcome){assert(done);dbt_command_done_t cb=done;done=0;sent=true;cb(done_context,0x0406,outcome,0,0,0);}
dbt_result_t dbt_gap_set_classic_scan(dbt_gap_t *g,bool discoverable,bool connectable){assert(discoverable && connectable);if(scan_result!=DBT_OK)return scan_result;scan_count++;g->classic_pending=true;return DBT_OK;}
bool probe_product_io_needs_worker(void){return led_receipt_available;}
bool probe_product_io_take_input(dbt_input_action_t *a){if(tail==head)return false;*a=queue[tail++];return true;}
bool probe_product_io_take_local_led(dbt_led_receipt_t *out){if(!led_receipt_available)return false;*out=receipt;led_receipt_available=false;return true;}
bool probe_product_io_take_play_result(probe_product_io_play_result_t *p){if(!play_result_ready)return false;play_result_ready=false;*p=(probe_product_io_play_result_t){last_sound,sound_result};return true;}
bool probe_product_io_led_snapshot(dbt_led_settings_t *p){if(!led_available)return false;*p=settings;return true;}
uint8_t probe_product_io_sound_status(void){return sound_status;}
bool probe_product_io_take_notice(dbt_ble_control_peer_t *p,uint8_t b[3],uint8_t *n){if(!notice_available)return false;notice_available=false;*p=(dbt_ble_control_peer_t){7,1};b[0]=0x33;b[1]=0x0b;*n=2;return true;}
dbt_result_t probe_product_io_local_led_relative(dbt_led_operation_t op,int8_t delta,uint32_t token){
    static const uint8_t expected[]={52,54};assert(led_sends<2 && op==DBT_LED_ADJUST_BRIGHTNESS && delta==5 && token);
    brightness_raw=(uint8_t)(brightness_raw+delta);settings.bytes[1]=(uint8_t)(((unsigned)brightness_raw*25u+32u)>>6);
    assert(settings.bytes[1]==expected[led_sends]);led_sends++;receipt.token=token;receipt.result=DBT_LED_OK;
    led_receipt_available=true;return DBT_OK;
}
dbt_result_t probe_gatt_control_worker_required(bool r){required=r;return DBT_OK;}
dbt_result_t probe_gatt_control_eq_delta(uint8_t b,int8_t d){assert(b==0);eq_attempts++;if(eq_block){eq_block=false;return DBT_BUSY;}steps[eq_success++]=d;return DBT_OK;}
dbt_result_t probe_gatt_control_toggle(uint8_t flag){flags^=flag;return DBT_OK;}
dbt_result_t probe_gatt_control_led_applied(const uint8_t v[10]){assert(v[1]==50 || v[1]==52 || v[1]==54);return DBT_OK;}
dbt_result_t probe_gatt_control_sound_applied(uint8_t v){assert(v==sound_status);sound_updates++;return DBT_OK;}
dbt_result_t probe_gatt_control_notify(const dbt_ble_control_peer_t *p,const uint8_t *v,uint8_t n){assert(p->generation==1 && n==2 && v[0]==0x33);if(notice_block){notice_block=false;return DBT_BUSY;}notices++;return DBT_OK;}
dbt_result_t probe_avrcp_snapshot(probe_avrcp_snapshot_t *s){memset(s,0,sizeof(*s));s->volume=volume;s->playback_status=1;return DBT_OK;}
dbt_result_t probe_output_volume(uint8_t v){volume=v;return DBT_OK;}
void probe_avrcp_local_volume(uint8_t v){assert(v==volume);}
dbt_result_t probe_avrcp_key(uint8_t op,bool release,uint32_t now){(void)now;last_op=op;if(release)key_up++;else key_down++;return DBT_OK;}
int main(void){
    assert(!probe_product_io_owner_startup_needed(false,1));
    assert(!probe_product_io_owner_startup_needed(true,0));
    assert(!probe_product_io_owner_startup_needed(true,0x81));
    assert(probe_product_io_owner_startup_needed(true,1));
    probe_product_io_owner_t s={0};s.sound_enabled=true;led_available=true;notice_available=true;notice_block=true;eq_block=true;
    queue[head++]=(dbt_input_action_t){DBT_INPUT_EQ_BASS,1};queue[head++]=(dbt_input_action_t){DBT_INPUT_EQ_BASS,-1};
    queue[head++]=(dbt_input_action_t){DBT_INPUT_BRIGHTNESS,5};queue[head++]=(dbt_input_action_t){DBT_INPUT_BRIGHTNESS,5};queue[head++]=(dbt_input_action_t){DBT_INPUT_VOLUME,3};
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PLAY_PAUSE,0};
    queue[head++]=(dbt_input_action_t){DBT_INPUT_BASS_BOOST,0};queue[head++]=(dbt_input_action_t){DBT_INPUT_BYPASS,0};
    probe_product_io_owner_poll(&s,0);assert(s.have_input && s.have_notice && eq_attempts==1 && !eq_success);
    probe_product_io_owner_poll(&s,20);assert(!s.have_notice && notices==1 && eq_success==2 && steps[0]==1 && steps[1]==-1 && s.led_pending && led_sends==1 && required);
    probe_product_io_owner_poll(&s,40);assert(s.led_pending && led_sends==2 && volume==50);
    probe_product_io_owner_poll(&s,60);assert(!s.led_pending && volume==53 && key_down==1 && last_op==0x46 && s.have_input);
    probe_product_io_owner_poll(&s,129);assert(!key_up && probe_product_io_owner_wait_ms(&s,129)==1);
    probe_product_io_owner_poll(&s,130);assert(key_up==1 && !s.have_input && sound_updates>0 && flags==5);
    dbt_host_t host={0};dbt_gap_t gap={0};gap.host=&host;gap.state=DBT_GAP_READY;s.gap=&gap;
    /* No audio peer: scan is queued once and success requires completion. */
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,200);
    assert(s.have_input && scan_count==1 && !disconnect_count);probe_product_io_owner_poll(&s,220);assert(scan_count==1);
    gap.classic_pending=false;gap.classic_scan=3;probe_product_io_owner_poll(&s,240);assert(!s.have_input && !s.failures);
    /* Preserve the original identity across backpressure and handle reuse. */
    active_link=(dbt_link_t){.used=true,.kind=DBT_LINK_CLASSIC,.handle=42,.generation=7};audio_active=true;
    enqueue_result=DBT_NO_SPACE;queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,300);
    assert(s.have_input && !disconnect_count);active_link.generation=8;enqueue_result=DBT_OK;probe_product_io_owner_poll(&s,320);
    assert(!s.have_input && !disconnect_count);
    /* Removal cancels the queued command before a replacement can be sent. */
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,340);assert(done && disconnect_count==1);
    dbt_link_change_t change={DBT_LINK_REMOVED,42,8};active_link.used=false;probe_product_io_owner_link(&s,&change);assert(!done);
    active_link.used=true;active_link.generation=9;probe_product_io_owner_poll(&s,360);assert(!s.have_input && disconnect_count==1);
    /* Repeated polling never resubmits; accepted command waits for link loss. */
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,400);assert(done && disconnect_count==2);
    probe_product_io_owner_poll(&s,420);assert(disconnect_count==2);status(DBT_COMMAND_SUCCESS);probe_product_io_owner_poll(&s,440);assert(s.have_input);
    change.generation=9;active_link.used=false;probe_product_io_owner_link(&s,&change);probe_product_io_owner_poll(&s,460);assert(!s.have_input);
    /* Rejection and no disconnect event are reported as failures. */
    active_link.used=true;active_link.generation=10;queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,500);
    status(DBT_COMMAND_REJECTED);probe_product_io_owner_poll(&s,520);assert(!s.have_input && s.failures==1);
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,600);status(DBT_COMMAND_SUCCESS);
    probe_product_io_owner_poll(&s,620);probe_product_io_owner_poll(&s,5620);assert(!s.have_input && s.failures==2);
    /* A rejected scan completion is not reported as successful pairing mode. */
    audio_active=false;gap.classic_scan=0;queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,5700);
    assert(gap.classic_pending);gap.classic_pending=false;probe_product_io_owner_poll(&s,5720);assert(!s.have_input && s.failures==3);
    /* Busy scan submission is retried without a disconnect; repeated queued
     * gestures converge on the same visibility and do not resubmit it. */
    scan_result=DBT_NO_SPACE;queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};
    probe_product_io_owner_poll(&s,5800);assert(s.have_input && scan_count==2);scan_result=DBT_OK;
    probe_product_io_owner_poll(&s,5820);assert(gap.classic_pending && scan_count==3);
    gap.classic_scan=3;gap.classic_pending=false;probe_product_io_owner_poll(&s,5840);assert(!s.have_input && scan_count==3);
    /* When already submitted, removal cannot cancel or retarget the command. */
    audio_active=true;active_link=(dbt_link_t){.used=true,.kind=DBT_LINK_CLASSIC,.handle=42,.generation=11};
    queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};probe_product_io_owner_poll(&s,5900);assert(done);sent=true;
    change.generation=11;active_link.used=false;probe_product_io_owner_link(&s,&change);assert(done);
    active_link.used=true;active_link.generation=12;status(DBT_COMMAND_SUCCESS);probe_product_io_owner_poll(&s,5920);
    assert(!s.have_input && disconnect_count==5 && s.failures==3);
    /* Profile/generation fence, delayed connected sound, backpressure and
     * attachment receipt are distinct from completed playback. */
    probe_product_io_owner_t a={0};a.gap=&gap;a.io_ready=true;a.sound_enabled=true;sound_status=15;pair_modes=0;
    probe_product_io_owner_audio(&a,0,42,30,0,100);assert(a.audio_connected && sound_cancels==1);
    probe_product_io_owner_audio(&a,0,42,30,0,101);assert(sound_cancels==1);
    probe_product_io_owner_audio(&a,2,42,29,44100,110);assert(!a.connected_pending);
    probe_product_io_owner_audio(&a,2,42,30,44100,110);assert(a.connected_pending);
    probe_product_io_owner_poll(&a,509);assert(!sound_sends);
    play_block=true;probe_product_io_owner_poll(&a,510);assert(a.sound_pending && !sound_sends);
    play_block=false;probe_product_io_owner_poll(&a,530);assert(a.sound_inflight && sound_sends==1 && last_sound==2 && !a.sound_attached);
    play_result_ready=true;sound_result=DBT_OK;sound_busy=true;probe_product_io_owner_poll(&a,550);assert(a.sound_attached==1 && !a.sound_inflight);
    probe_product_io_owner_audio(&a,3,42,30,0,560);assert(sound_cancels==2);
    sound_busy=false;probe_product_io_owner_audio(&a,1,42,29,0,570);assert(a.audio_connected);
    probe_product_io_owner_audio(&a,1,42,30,0,580);assert(!a.audio_connected && sound_cancels==3);
    probe_product_io_owner_audio(&a,0,42,31,0,600);probe_product_io_owner_audio(&a,2,42,31,48000,610);assert(!a.connected_pending);
    /* Volume grace/cooldown, mute and receipt failure handling. */
    probe_product_io_owner_volume(&a,127,2599);assert(!a.sound_pending);
    probe_product_io_owner_volume(&a,127,2600);assert(a.sound_pending && volume_feedback==2);
    sound_status=0x8f;probe_product_io_owner_poll(&a,2600);assert(!a.sound_pending && sound_sends==1);
    sound_status=15;probe_product_io_owner_volume(&a,127,4500);assert(!a.sound_pending);
    probe_product_io_owner_volume(&a,127,4600);probe_product_io_owner_poll(&a,4600);assert(sound_sends==2 && last_sound==3);
    play_result_ready=true;sound_result=DBT_INVALID;probe_product_io_owner_poll(&a,4620);assert(a.failures==1 && !a.sound_inflight);
    /* Pairing feedback appears only after scan completion; a different
     * generation is not allowed to cancel the current profile. */
    audio_active=false;gap.classic_scan=3;queue[head++]=(dbt_input_action_t){DBT_INPUT_PAIR,0};
    probe_product_io_owner_poll(&a,4700);assert(a.pairing_active && pair_modes==1 && sound_sends==3 && last_sound==1);
    probe_product_io_owner_audio(&a,0,42,32,0,4800);assert(!a.pairing_active && pair_success==1 && sound_cancels==5);
    return 0;
}
