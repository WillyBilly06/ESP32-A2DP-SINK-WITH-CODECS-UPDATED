#include "dbt_led.h"
#include <string.h>

static uint8_t clamp100(uint8_t n) { return n > 100 ? 100 : n; }
static uint8_t to255(uint8_t n) {
  unsigned v = ((unsigned)n * 41u) >> 4;
  return v > 255 ? 255 : (uint8_t)v;
}
void dbt_led_default(dbt_led_settings_t *s) {
  static const uint8_t d[10] = {0, 25, 50, 255, 0, 128, 0, 128, 255, 0};
  if (s)
    memcpy(s->bytes, d, 10);
}
bool dbt_led_effect_supported(uint8_t e) { return e < DBT_LED_EFFECT_COUNT; }
bool dbt_led_settings_valid(const dbt_led_settings_t *s) {
  return s && dbt_led_effect_supported(s->bytes[0]) && s->bytes[1] <= 100 &&
         s->bytes[9] <= 4;
}
void dbt_led_to_legacy(uint8_t o[10], const dbt_led_settings_t *s) {
  o[0] = to255(s->bytes[1]);
  memcpy(o + 1, s->bytes + 3, 6);
  o[7] = s->bytes[9];
  o[8] = s->bytes[2];
  o[9] = s->bytes[0];
}
void dbt_led_from_legacy(dbt_led_settings_t *s, const uint8_t p[10]) {
  s->bytes[0] = p[9];
  s->bytes[1] = (uint8_t)(((unsigned)p[0] * 100u + 127u) / 255u);
  s->bytes[2] = p[8];
  memcpy(s->bytes + 3, p + 1, 6);
  s->bytes[9] = p[7];
}
bool dbt_led_init(dbt_led_t *l, uint16_t w, uint16_t h, uint8_t *f, size_t n,
                  const dbt_led_io_t *io, const uint8_t legacy[10]) {
  if (!l || !f || !io || !io->show || !io->commit || !w || !h || w > 32 ||
      h > 32 || n < (size_t)w * h * 3)
    return false;
  memset(l, 0, sizeof(*l));
  l->width = w;
  l->height = h;
  l->frame = f;
  l->frame_bytes = n;
  l->io = *io;
  l->active_effect = 255;
  l->random_state=1337;
  atomic_init(&l->meter, 0);
  atomic_init(&l->audio_active, false);
  atomic_init(&l->pairing_event, 0);
  atomic_init(&l->volume_event, 0);
  atomic_init(&l->beat_event, 0);
  atomic_init(&l->request_head, 0);
  atomic_init(&l->request_tail, 0);
  atomic_init(&l->receipt_head, 0);
  atomic_init(&l->receipt_tail, 0);
  dbt_led_default(&l->settings);
  dbt_led_to_legacy(l->legacy, &l->settings);
  if (legacy) {
    dbt_led_settings_t saved;
    dbt_led_from_legacy(&saved, legacy);
    if (!dbt_led_settings_valid(&saved))
      return false;
    l->settings = saved;
    memcpy(l->legacy, legacy, 10);
  }
  return true;
}
bool dbt_led_set_allocator(dbt_led_t *l,void *(*allocate)(void *,size_t),
                           void (*release)(void *,void *),void *context) {
  if(!l || l->active_effect!=255 || !allocate || !release)return false;
  l->effect_allocate=allocate;l->effect_release=release;l->effect_alloc_context=context;return true;
}
void dbt_led_deinit(dbt_led_t *l) {
  if(l && l->effect_dynamic && l->effect_release)l->effect_release(l->effect_alloc_context,l->effect_dynamic);
  if(l){l->effect_dynamic=NULL;l->active_effect=255;}
}
void dbt_led_meter(dbt_led_t *l, uint8_t b, uint8_t m, uint8_t t) {
  if (l)
    atomic_store_explicit(&l->meter,
                          (unsigned)clamp100(b) | ((unsigned)clamp100(m) << 8) |
                              ((unsigned)clamp100(t) << 16),
                          memory_order_release);
}
void dbt_led_audio_active(dbt_led_t *l, bool active) {
  if (l)
    atomic_store_explicit(&l->audio_active, active, memory_order_release);
}
static void publish_event(atomic_uint *event,unsigned payload) {
  unsigned old=atomic_load_explicit(event,memory_order_relaxed),next;
  do {next=((old+256u)&~255u)|payload;}
  while(!atomic_compare_exchange_weak_explicit(event,&old,next,memory_order_release,memory_order_relaxed));
}
void dbt_led_volume(dbt_led_t *l,uint8_t volume) {
  if(l)publish_event(&l->volume_event,volume>127?127:volume);
}
void dbt_led_pairing_mode(dbt_led_t *l,bool active) {
  if(l)publish_event(&l->pairing_event,active?1u:0u);
}
void dbt_led_pairing_success(dbt_led_t *l) {
  if(l)publish_event(&l->pairing_event,2u);
}
void dbt_led_beat(dbt_led_t *l,uint8_t intensity) {
  if(l)publish_event(&l->beat_event,clamp100(intensity));
}
dbt_led_result_t dbt_led_submit(dbt_led_t *l, const dbt_led_request_t *r) {
  if (!l || !r || !r->token || r->operation < DBT_LED_SET_ALL ||
      r->operation > DBT_LED_CONFIRM_PREVIEW)
    return DBT_LED_INVALID;
  if (r->operation == DBT_LED_SET_ALL && !dbt_led_settings_valid(&r->settings))
    return DBT_LED_INVALID;
  if ((r->operation == DBT_LED_SET_EFFECT ||
       r->operation == DBT_LED_PREVIEW_EFFECT) &&
      !dbt_led_effect_supported(r->settings.bytes[0]))
    return DBT_LED_INVALID;
  if (r->operation == DBT_LED_SET_BRIGHTNESS && r->settings.bytes[1] > 100)
    return DBT_LED_INVALID;
  if ((r->operation == DBT_LED_ADJUST_BRIGHTNESS && r->delta != 5 &&
       r->delta != -5) ||
      ((r->operation == DBT_LED_ADJUST_EFFECT ||
        r->operation == DBT_LED_PREVIEW_DELTA) &&
       r->delta != 1 && r->delta != -1) ||
      (r->operation == DBT_LED_CONFIRM_PREVIEW && r->delta != 0))
    return DBT_LED_INVALID;
  unsigned h = atomic_load_explicit(&l->request_head, memory_order_relaxed),
           t = atomic_load_explicit(&l->request_tail, memory_order_acquire);
  if (h - t >= DBT_LED_QUEUE_CAPACITY)
    return DBT_LED_BUSY;
  l->requests[h % DBT_LED_QUEUE_CAPACITY] = *r;
  atomic_store_explicit(&l->request_head, h + 1, memory_order_release);
  return DBT_LED_OK;
}
bool dbt_led_receipt(dbt_led_t *l, dbt_led_receipt_t *out) {
  if (!l || !out)
    return false;
  unsigned t = atomic_load_explicit(&l->receipt_tail, memory_order_relaxed),
           h = atomic_load_explicit(&l->receipt_head, memory_order_acquire);
  if (t == h)
    return false;
  *out = l->receipts[t % DBT_LED_QUEUE_CAPACITY];
  atomic_store_explicit(&l->receipt_tail, t + 1, memory_order_release);
  return true;
}
/* Integer parabolic sine matching the baseline's sin8 shape. */
static unsigned wave(unsigned v) {
  unsigned x = v & 127u;
  unsigned y = (x * (128u - x)) >> 5;
  if (y > 127)
    y = 127;
  return (v & 128u) ? 128u - y : 128u + y;
}
static unsigned absdiff(unsigned a, unsigned b) {
  return a > b ? a - b : b - a;
}
static unsigned root(unsigned v) {
  unsigned r = 0, bit = 1u << 30;
  while (bit > v)
    bit >>= 2;
  while (bit) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else
      r >>= 1;
    bit >>= 2;
  }
  return r;
}
static uint8_t mix(uint8_t a, uint8_t b, unsigned t) {
  return (uint8_t)(((unsigned)a * (255u - t) + (unsigned)b * t + 127u) / 255u);
}
typedef struct {
  uint8_t r, g, b;
} colour_t;
typedef struct { int16_t x,y,vx,vy;uint8_t hue,life; } particle_t;
typedef struct { int16_t x,y,vy;uint8_t hue,active; } rocket_t;
typedef struct { particle_t particles[100];rocket_t rockets[3]; } fireworks_t;
static uint8_t random8(dbt_led_t *l) {
  l->random_state=(uint16_t)(l->random_state*2053u+13849u);return (uint8_t)(l->random_state>>8);
}
static unsigned random_below(dbt_led_t *l,unsigned n){return (unsigned)random8(l)*n>>8;}
static bool activate_effect(dbt_led_t *l,unsigned effect) {
  if(l->active_effect==effect)return true;
  dbt_led_deinit(l);
  size_t n=effect==9?sizeof(fireworks_t):effect==11?80u*sizeof(particle_t):0u;
  if(n){
    if(!l->effect_allocate || !l->effect_release)return false;
    l->effect_dynamic=l->effect_allocate(l->effect_alloc_context,n);
    if(!l->effect_dynamic)return false;
    memset(l->effect_dynamic,0,n);
  }
  memset(l->effect_state,0,sizeof(l->effect_state));
  memset(l->frame,0,(size_t)l->width*l->height*3u);
  if(effect==7)for(unsigned i=0;i<40;i++){
    uint8_t *s=l->effect_state+i*4u;s[0]=(uint8_t)random_below(l,16);s[1]=(uint8_t)random_below(l,16);
    s[2]=(uint8_t)(50u+random_below(l,150));int delta=(int)(2u+random_below(l,6));
    s[3]=(uint8_t)(int8_t)((random8(l)&1u)?-delta:delta);
  }
  if(effect==19)for(unsigned i=0;i<5;i++){
    uint8_t *s=l->effect_state+i*6u;s[0]=(uint8_t)((4u+random_below(l,8))*16u);
    s[1]=(uint8_t)((4u+random_below(l,8))*16u);s[2]=(uint8_t)(int8_t)((int)random_below(l,9)-4);
    s[4]=random8(l);s[5]=(uint8_t)(1u+random_below(l,2));
  }
  if(effect==18)memset(l->effect_state,8,16);
  if(effect==20)for(unsigned i=0;i<4;i++){
    uint8_t *s=l->effect_state+i*6u;s[0]=(uint8_t)((4u+random_below(l,8))*16u);
    s[1]=(uint8_t)((4u+random_below(l,8))*16u);s[2]=(uint8_t)(int8_t)((int)random_below(l,7)-3);
    s[3]=(uint8_t)(int8_t)((int)random_below(l,7)-3);s[4]=(uint8_t)((3u+random_below(l,3))*16u);s[5]=random8(l);
  }
  l->active_effect=(uint8_t)effect;l->frame_number=0;l->effect_hue=0;l->pulse=0;
  return true;
}
static void particles_step(dbt_led_t *l,unsigned effect,unsigned bass,bool beat) {
  fireworks_t *fire=effect==9?(fireworks_t *)l->effect_dynamic:NULL;
  particle_t *p=fire?fire->particles:(particle_t *)l->effect_dynamic;
  unsigned count=fire?100u:80u;
  if(beat && fire)for(unsigned i=0;i<3;i++)if(!fire->rockets[i].active){
    rocket_t *r=&fire->rockets[i];r->active=1;r->x=(int16_t)((4u+random_below(l,8))*256u);
    r->y=15*256;r->vy=(int16_t)(-128-(int)random8(l)/2);r->hue=random8(l);break;
  }
  if(beat && !fire){
    unsigned n=20u+bass*20u/100u;l->effect_hue=(uint8_t)(l->effect_hue+40u);
    for(unsigned i=0;i<count && n;i++)if(!p[i].life){
      unsigned angle=random8(l),speed=(51u+random_below(l,218))*(100u+bass)/100u;
      p[i].x=p[i].y=7*256+128;
      p[i].vx=(int16_t)(((int)wave(angle+64u)-128)*(int)speed/128);
      p[i].vy=(int16_t)(((int)wave(angle)-128)*(int)speed/128);
      p[i].hue=(uint8_t)(l->effect_hue+random_below(l,40));p[i].life=50;n--;
    }
  }
  if(fire)for(unsigned i=0;i<3;i++)if(fire->rockets[i].active){
    rocket_t *r=&fire->rockets[i];r->y=(int16_t)(r->y+r->vy);r->vy=(int16_t)(r->vy+3);
    if(r->y<4*256 || r->vy>-26){
      unsigned n=20;
      for(unsigned j=0;j<count && n;j++)if(!p[j].life){
        unsigned angle=random8(l),speed=77u+random_below(l,164);
        p[j].x=r->x;p[j].y=r->y;
        p[j].vx=(int16_t)(((int)wave(angle+64u)-128)*(int)speed/128);
        p[j].vy=(int16_t)(((int)wave(angle)-128)*(int)speed/128);
        p[j].hue=(uint8_t)(r->hue+random_below(l,30));p[j].life=(uint8_t)(40u+random_below(l,30));n--;
      }
      r->active=0;
    }
  }
  for(unsigned i=0;i<count;i++)if(p[i].life){
    int x=p[i].x+p[i].vx,y=p[i].y+p[i].vy;
    if(fire)p[i].vy=(int16_t)(p[i].vy+5);
    if(!(--p[i].life) || x<0 || x>=16*256 || y<0 || y>=16*256){p[i].life=0;continue;}
    p[i].x=(int16_t)x;p[i].y=(int16_t)y;
  }
}
static colour_t hsv(unsigned h, unsigned sat, unsigned val) {
  h &= 255u;
  unsigned region = (h * 6u) >> 8, rem = (h * 6u) & 255u;
  unsigned p = (val * (255u - sat) + 128u) >> 8,
           q = (val * (255u - ((sat * rem + 128u) >> 8)) + 128u) >> 8,
           t = (val * (255u - ((sat * (255u - rem) + 128u) >> 8)) + 128u) >> 8;
  switch (region) {
  case 0:
    return (colour_t){val, t, p};
  case 1:
    return (colour_t){q, val, p};
  case 2:
    return (colour_t){p, val, t};
  case 3:
    return (colour_t){p, q, val};
  case 4:
    return (colour_t){t, p, val};
  default:
    return (colour_t){val, p, q};
  }
}
static unsigned noise(unsigned x, unsigned y, unsigned t) {
  uint32_t z =
      (x + 1u) * 0x9e3779b9u ^ (y + 17u) * 0x85ebca6bu ^ (t + 3u) * 0xc2b2ae35u;
  z ^= z >> 16;
  z *= 0x7feb352du;
  z ^= z >> 15;
  return z & 255u;
}
/* probe_output_meter is a linear mapping of -60..0 dB, not amplitude. */
static const uint8_t meter_linear[101] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
    3,  4,  4,  4,  4,  5,  5,  5,  6,  6,  7,  7,  8,  8,  9,  10, 10,
    11, 12, 13, 13, 14, 15, 17, 18, 19, 20, 22, 23, 25, 27, 29, 31, 33,
    35, 38, 41, 44, 47, 50, 54, 58, 62, 66, 71, 76, 81, 87, 93, 100};
static bool overlay(dbt_led_t *l,uint8_t brightness) {
  unsigned pair=atomic_load_explicit(&l->pairing_event,memory_order_acquire);
  unsigned vol=atomic_load_explicit(&l->volume_event,memory_order_acquire);
  uint32_t now=l->render_now_ms;
  if(pair!=l->pairing_seen){l->pairing_seen=pair;l->pairing_state=(uint8_t)(pair&255u);l->pairing_started=now;}
  if(vol!=l->volume_seen){l->volume_seen=vol;l->volume_target=(uint8_t)(vol&255u);l->volume_started=now;l->volume_visible=true;}
  uint32_t pair_elapsed=now-l->pairing_started,vol_elapsed=now-l->volume_started;
  if(l->pairing_state==2 && pair_elapsed>=500u)l->pairing_state=0;
  if(l->volume_visible && vol_elapsed>=2500u)l->volume_visible=false;
  if(!l->pairing_state && !l->volume_visible)return false;
  unsigned w=l->width,h=l->height,bright=brightness;
  if(l->pairing_state){
    unsigned strength;
    if(l->pairing_state==2) {
      strength=(pair_elapsed<150u || (pair_elapsed>=250u && pair_elapsed<400u))?255u:0u;
      if(!bright)bright=60;
    } else {
      strength=51u+wave((pair_elapsed%2000u)*256u/2000u)*204u/255u;
      if(!bright)bright=40;
    }
    unsigned green=strength*(l->pairing_state==2?200u:30u)/255u;
    for(unsigned i=0;i<w*h;i++){
      l->frame[3*i]=(uint8_t)(green*bright/255u);l->frame[3*i+1]=0;
      l->frame[3*i+2]=(uint8_t)(strength*bright/255u);
    }
  } else {
    int32_t target=(int32_t)((uint32_t)l->volume_target*65535u/127u);
    l->volume_smooth=(uint32_t)((int32_t)l->volume_smooth+(target-(int32_t)l->volume_smooth)*3/10);
    unsigned rows=(l->volume_smooth*h+32767u)/65535u;
    unsigned fade=vol_elapsed<1500u?255u:(2500u-vol_elapsed)*255u/1000u;
    unsigned pulse=217u+wave(l->overlay_frame*6u)*38u/255u;
    if(bright<10u)bright=10u;
    for(unsigned y=0;y<h;y++)for(unsigned x=0;x<w;x++) {
      unsigned row=h-1u-y,pos=(y*w+((y&1u)?w-1u-x:x))*3u;
      unsigned edge=w>1?absdiff(2u*x,w-1u)*77u/(w-1u):0u;
      unsigned intensity=row<rows?pulse*fade/255u*(255u-edge)/255u*bright/255u:0u;
      unsigned white=h>1?(h-1u-row)*255u/(h-1u):255u;
      l->frame[pos]=(uint8_t)(white*intensity/255u);l->frame[pos+1]=(uint8_t)intensity;
      l->frame[pos+2]=(uint8_t)(white*intensity/255u);
    }
  }
  ++l->overlay_frame;
  return true;
}
static bool frame(dbt_led_t *l, const dbt_led_settings_t *s, uint8_t bright) {
  unsigned e=s->bytes[0];
  if(overlay(l,bright))return l->io.show(l->io.context,l->frame,(size_t)l->width*l->height*3u);
  if(!activate_effect(l,e))return false;
  unsigned m = atomic_load_explicit(&l->meter, memory_order_acquire),
           bass = m & 255u, mid = (m >> 8) & 255u, high = (m >> 16) & 255u;
  bass = meter_linear[bass];
  mid = meter_linear[mid];
  high = meter_linear[high];
  unsigned w = l->width, h = l->height;
  unsigned f = ++l->frame_number;
  if (atomic_load_explicit(&l->audio_active, memory_order_acquire))l->idle_timer_started=false;
  else if(!l->idle_timer_started){l->idle_timer_started=true;l->inactive_since_ms=l->render_now_ms;}
  bool demo=l->idle_timer_started && (uint32_t)(l->render_now_ms-l->inactive_since_ms)>5000u;
  if (demo) {
    bass = 30u + wave(f * 2u) * 30u / 255u;
    mid = 20u + wave(f * 3u + 85u) * 20u / 255u;
    high = 10u + wave(f * 4u + 170u) * 20u / 255u;
    if(e==0){mid=30u+wave(f*3u+85u)*20u/255u;high=20u+wave(f*4u+170u)*20u/255u;}
    else if(e==3){bass=30u+wave(f*2u)*20u/255u;mid=high=0;}
    else if(e==5){bass=30;mid=20;high=20;}
    else if(e==7){bass=10;mid=high=0;}
    else if(e==9){bass=mid=high=0;}
    else if(e==11 || e==19){bass=30;mid=high=0;}
    else if(e==14){bass=wave(f*3u)>200?80:40;mid=high=0;}
    else if(e==15){bass=f%20u==0?80:10;mid=high=0;}
  }
  bool beat =
      !demo && bass > l->previous_bass + 8u && bass > 35u;
  if(demo){unsigned period=e==2 || e==19?25u:e==9?45u:e==7 || e==16?40u:
                          e==1 || e==11 || e==15 || e==20?30u:0u;
    beat=period && f%period==0;}
  unsigned beat_event=atomic_load_explicit(&l->beat_event,memory_order_acquire);
  if(beat_event && !demo) {
    beat=beat_event!=l->beat_seen;
    if(beat)l->beat_seen=beat_event;
  }
  unsigned beat_intensity=beat_event && !demo?(beat_event&255u):demo && beat?70u:bass;
  if (beat) {
    l->pulse = (uint8_t)(demo && e==1?200:255);
    if(e!=11)l->effect_hue = (uint8_t)(l->effect_hue + 32u);
  } else
    l->pulse = (uint8_t)(l->pulse * (demo && e==1?230u:217u) / 256u);
  l->previous_bass = (uint8_t)bass;
  if(e==9 || e==11)particles_step(l,e,beat_intensity,beat);
  if(e==0)for(unsigned x=0;x<16;x++){
    unsigned band=x<5?(bass*(5u-x)+mid*x)/5u:x<10?(mid*(10u-x)+high*(x-5u))/5u:high;
    band=band*(204u+wave(x*16u+f)*102u/256u)*3u/512u;
    if(band>100)band=100;
    unsigned db=0;for(unsigned k=0;k<=100;k++)if(meter_linear[k]<=band)db=k;
    if(!band)db=0;
    unsigned target=db*15u/100u,old=l->effect_state[x];
    l->effect_state[x]=(uint8_t)(target*16u>old?target*16u:old*217u/256u);
    unsigned height=l->effect_state[x]/16u;
    if(height>l->effect_state[16+x]){l->effect_state[16+x]=(uint8_t)height;l->effect_state[32+x]=30;}
    else if(l->effect_state[32+x])--l->effect_state[32+x];
    else if(l->effect_state[16+x])--l->effect_state[16+x];
  }
  if(e==2){
    if(beat)for(unsigned i=0;i<5;i++){uint8_t *s=l->effect_state+i*4u;if(!s[3]){
      s[0]=0;s[1]=(uint8_t)(5u+beat_intensity*5u/100u);s[2]=random8(l);s[3]=1;break;}}
    for(unsigned i=0;i<5;i++){uint8_t *s=l->effect_state+i*4u;if(s[3]){
      unsigned radius=s[0]+s[1];if(radius>240u)s[3]=0;else s[0]=(uint8_t)radius;}}
  }
  if(e==5)for(unsigned x=0;x<16;x++){
    uint8_t *s=l->effect_state+x*4u;
    if(!s[3] && random8(l)<30u+high*40u/100u){s[3]=1;s[0]=(uint8_t)(64u-(1u+random_below(l,7))*8u);
      s[1]=(uint8_t)(2u+random_below(l,5));s[2]=(uint8_t)(3u+random_below(l,8));}
    if(s[3]){unsigned step=s[1]*(30u+bass/2u+mid*3u/10u)/100u;if(!step)step=1;
      unsigned y=s[0]+step;if(y>64u+(16u+s[2])*8u || y>255u)s[3]=0;else s[0]=(uint8_t)y;}
  }
  if(e==6){
    unsigned target=(bass*6u+mid*3u+high)*2u*15u*8u/1000u;
    if(target>255u)target=255u;
    unsigned target2=target*(230u+wave(f*3u)*51u/255u)/256u;
    if(target2>255u)target2=255u;
    for(unsigned i=0;i<2;i++){
      int t=(int)(i?target2:target),v=l->effect_state[i];v+=(t-v)*3/10;
      l->effect_state[i]=(uint8_t)v;
      if(v>l->effect_state[2+i]){l->effect_state[2+i]=(uint8_t)v;l->effect_state[4]=30;}
    }
    if(l->effect_state[4])--l->effect_state[4];
    else for(unsigned i=2;i<4;i++)l->effect_state[i]=l->effect_state[i]>2?l->effect_state[i]-2:0;
  }
  if(e==7)for(unsigned i=0;i<40;i++){
    uint8_t *s=l->effect_state+i*4u;int v=s[2]+(int8_t)s[3];
    if(v>250 || v<30){s[3]=(uint8_t)(int8_t)(-(int8_t)s[3]);v=s[2]+(int8_t)s[3];}
    if(v<0)v=0;
    if(v>255)v=255;
    s[2]=(uint8_t)v;
  }
  if(e==15){
    if(bass>40u || (beat && beat_intensity>30u))for(unsigned i=0;i<8;i++){
      uint8_t *s=l->effect_state+i*6u;if(!s[5]){s[0]=0;s[1]=(uint8_t)random_below(l,16);
        s[2]=(uint8_t)(4u+bass*12u/100u);s[3]=random8(l);s[4]=(uint8_t)(4u+random_below(l,6));s[5]=1;break;}}
    for(unsigned i=0;i<8;i++){uint8_t *s=l->effect_state+i*6u;if(s[5]){
      unsigned y=s[0]+s[2];if(y>=(16u+s[4])*8u || y>255u)s[5]=0;else s[0]=(uint8_t)y;}}
  }
  if(e==19)for(unsigned i=0;i<5;i++){
    uint8_t *s=l->effect_state+i*6u;int x=s[0],y=s[1],vx=(int8_t)s[2],vy=(int8_t)s[3],r=s[5];
    vy+=2+(int)bass*3/100;if(beat)vy-=(int)beat_intensity*32/100;
    if(vy>100)vy=100;
    if(vy< -100)vy= -100;
    x+=vx;y+=vy;
    if(x<r*16){x=r*16;vx= -vx*8/10;}if(x>=(16-r)*16){x=(15-r)*16;vx= -vx*8/10;}
    if(y<r*16){y=r*16;vy= -vy*7/10;}if(y>=(16-r)*16){y=(15-r)*16;vy= -vy*7/10;}
    s[0]=(uint8_t)x;s[1]=(uint8_t)y;s[2]=(uint8_t)(int8_t)vx;s[3]=(uint8_t)(int8_t)vy;s[4]++;
  }
  if(e==20)for(unsigned i=0;i<4;i++){
    uint8_t *s=l->effect_state+i*6u;int x=s[0],y=s[1],vx=(int8_t)s[2],vy=(int8_t)s[3];
    vx+=((int)wave(f+i*41u)-128)/64+((int)bass-50)/25;
    vy+=((int)wave(f+64u+i*81u)-128)/64+((int)high-50)/25;
    vx=vx*95/100;vy=vy*95/100;
    if(vx>24)vx=24;
    if(vx< -24)vx= -24;
    if(vy>24)vy=24;
    if(vy< -24)vy= -24;
    x+=vx*(5+(int)mid*15/100)/10;y+=vy*(5+(int)mid*15/100)/10;
    if(x<32){x=32;vx= -vx;}if(x>208){x=208;vx= -vx;}
    if(y<32){y=32;vy= -vy;}if(y>208){y=208;vy= -vy;}
    s[0]=(uint8_t)x;s[1]=(uint8_t)y;s[2]=(uint8_t)(int8_t)vx;s[3]=(uint8_t)(int8_t)vy;
    s[4]=(uint8_t)(48u+bass*32u/100u+wave(f*2u+i*40u)*8u/255u);s[5]+=(uint8_t)(beat?5:1);
  }
  if (e == 14) {
    for (unsigned k = 7; k > 0; k--)
      l->effect_state[k] = (uint8_t)(l->effect_state[k - 1] * 230u / 256u);
    l->effect_state[0] = (uint8_t)(bass * 255u / 100u);
  }
  if (e == 18) {
    for (unsigned k = 0; k < 15; k++)
      l->effect_state[k] = l->effect_state[k + 1];
    int sample=(int)(bass*5u+mid*3u+high*2u)*7/1000+
        ((int)random8(l)-128)*(int)high*21/12800;
    if(sample< -8)sample= -8;
    if(sample>7)sample=7;
    l->effect_state[15]=(uint8_t)(8+sample);
  }
  if (e == 16) {
    unsigned target = (bass * 5u + mid * 3u + high * 2u) * 255u / 1000u;
    unsigned v = l->effect_state[0];
    if (v < target)
      v += (target - v + 3u) / 4u;
    else
      v -= (v - target + 7u) / 8u;
    l->effect_state[0] = (uint8_t)v;
  }
  if (e == 3) {
    for (unsigned yy = 0; yy < 15; yy++)
      for (unsigned xx = 0; xx < 16; xx++) {
        unsigned a = l->effect_state[(yy + 1) * 16 + xx],
                 b = l->effect_state[(yy + 1) * 16 + (xx + 1) % 16],
                 cool = noise(xx, yy, f) % 20;
        unsigned heat = (a * 2u + b) / 3u;
        l->effect_state[yy * 16 + xx] =
            (uint8_t)(heat > cool ? heat - cool : 0);
      }
    for (unsigned xx = 0; xx < 16; xx++)
      if (noise(xx, 15, f) < 80u + bass * 120u / 100u)
        l->effect_state[240 + xx] = (uint8_t)(160u + noise(xx, 14, f) % 96u);
  }
  for (unsigned y = 0; y < h; y++)
    for (unsigned x = 0; x < w; x++) {
      unsigned X = x * 16u / w, Y = y * 16u / h, dx = absdiff(2u * X, 15u),
               dy = absdiff(2u * Y, 15u), dist = root(dx * dx + dy * dy) / 2u;
      colour_t c = {0, 0, 0};
      unsigned hue = 0, value = 0;
      switch (e) {
      case 0: {
        unsigned bar=l->effect_state[X]/16u,peak=l->effect_state[16u+X],row=15u-Y;
        if(row<bar)c=hsv(row<5?96:row<10?64-(row-5u)*12u:0,255,255);
        if(peak && peak<16u && row==peak)c=(colour_t){255,255,255};
        break;
      }
      case 1:
        c=hsv(l->effect_hue+128u,demo?200u:255u,demo?20u:bass*30u/100u);
        if(l->pulse>10u){colour_t p=hsv(l->effect_hue,255,l->pulse);
          unsigned r=c.r+p.r,g=c.g+p.g,b=c.b+p.b;
          c=(colour_t){r>255?255:r,g>255?255:g,b>255?255:b};}
        break;
      case 2: {
        unsigned distance=root((dx*dx+dy*dy)*64u);
        for(unsigned k=0;k<5;k++){uint8_t *s=l->effect_state+k*4u;
          unsigned delta=absdiff(distance,s[0]);
          if(s[3] && delta<16u){unsigned v=(255u-s[0]*15u/16u)*(16u-delta)/16u;
            colour_t z=hsv(s[2],255,v);unsigned r=c.r+z.r,g=c.g+z.g,b=c.b+z.b;
            c=(colour_t){r>255?255:r,g>255?255:g,b>255?255:b};}}
        break;
      }
      case 3: {
        unsigned heat = l->effect_state[Y * 16u + X];
        c = (colour_t){heat < 85 ? heat * 3u : 255u,
                       heat < 85    ? 0
                       : heat < 170 ? (heat - 85u) * 3u
                                    : 255u,
                       heat < 170 ? 0 : (heat - 170u) * 3u};
        break;
      }
      case 4:
        hue = (wave(X * 16u + f) + wave(Y * 16u + 2u * f) +
               wave((X + Y) * 8u + f) + wave(dist * 8u - f)) /
              4u;
        c = hsv(hue, 255, 128u + wave(hue + f) / 2u);
        break;
      case 5: {
        uint8_t *s=l->effect_state+X*4u;int head=((int)s[0]-64)/8,trail=head-(int)Y;
        if(s[3] && trail>=0 && trail<s[2]){
          unsigned value=255u-(unsigned)trail*255u/s[2];
          c=trail==0?(colour_t){255,255,255}:(colour_t){0,value,0};}
        break;
      }
      case 6: {
        if (X == 7 || X == 8)
          c = hsv(f + Y * 16u + (X == 8 ? 128 : 0), 255, 100);
        else {
          unsigned side=X<7?0u:1u,row=15u-Y,level=l->effect_state[side]/8u,peak=l->effect_state[2u+side]/8u;
          if(row<level)c=hsv(row<10?96u-row*8u:0u,255,255);
          if(peak && peak<16u && row==peak)c=(colour_t){255,255,255};
        }
        break;
      }
      case 7: {
        c=hsv(160,255,bass/5u);
        for(unsigned k=0;k<40;k++){uint8_t *s=l->effect_state+k*4u;
          if(s[0]==X && s[1]==Y){unsigned v=s[2]+l->pulse/5u;if(v>255)v=255;c=(colour_t){v,v,v};}}
        break;
      }
      case 8: {
        int yy = (int)Y;
        int y1 = 8 + ((int)wave(f * 4u + X * 16u) - 128) *
                         (3 + (int)bass * 4 / 100) / 128;
        int y2 = 8 + ((int)wave(f * 6u + X * 24u + 40u) - 128) *
                         (2 + (int)mid * 3 / 100) / 128;
        int y3 = 8 + ((int)wave(f * 8u + X * 32u + 80u) - 128) *
                         (1 + (int)high * 2 / 100) / 128;
        if (yy == y1)
          c = (colour_t){255, 0, 0};
        if (yy == y2)
          c = (colour_t){0, 255, 0};
        if (yy == y3)
          c = (colour_t){0, 0, 255};
        break;
      }
      case 9: {
        fireworks_t *fire=l->effect_dynamic;
        for(unsigned k=0;k<3;k++)if(fire->rockets[k].active &&
          fire->rockets[k].x/256==(int)X && fire->rockets[k].y/256==(int)Y)c=(colour_t){255,255,255};
        for(unsigned k=0;k<100;k++)if(fire->particles[k].life &&
          fire->particles[k].x/256==(int)X && fire->particles[k].y/256==(int)Y) {
            unsigned life=fire->particles[k].life;
            c=hsv(fire->particles[k].hue,255,life>30?255:life*8u);
          }
        break;
      }
      case 10:
        hue = f + X * 8u + Y * 8u + wave(f + X * 16u) / 4u;
        c = hsv(hue, 255, 128u + bass * 127u / 100u);
        break;
      case 11: {
        particle_t *p=l->effect_dynamic;
        for(unsigned k=0;k<80;k++)if(p[k].life && p[k].x/256==(int)X && p[k].y/256==(int)Y)
          c=hsv(p[k].hue,255,p[k].life*255u/50u);
        if (absdiff(X,7u)<=1 && absdiff(Y,7u)<=1)
          c = hsv(l->effect_hue, 255, 50u + bass);
        break;
      }
      case 12: {
        unsigned xx = X < 8 ? 7 - X : X - 8, yy = Y < 8 ? 7 - Y : Y - 8;
        unsigned d = root(xx * xx + yy * yy);
        hue = f + mid / 2u + d * 20u + xx * 9u + yy * 5u;
        value = 100u + wave(d * 30u - f * 2u) / 2u + bass * 80u / 100u;
        if (value > 255)
          value = 255;
        c = hsv(hue, 255, value);
        break;
      }
      case 13: {
        const unsigned levels[3]={bass,mid,high};
        for(unsigned arm=0;arm<3;arm++)for(unsigned radius=0;radius<512u+levels[arm]*1536u/100u;radius+=77u){
          unsigned angle=f*2u+arm*85u+radius*20u/256u;
          int px=(1920+((int)wave(angle+64u)-128)*(int)radius/128)/256;
          int py=(1920+((int)wave(angle)-128)*(int)radius/128)/256;
          if(px==(int)X && py==(int)Y){colour_t z=hsv(arm*85u+radius*10u/256u,255,255u-radius*30u/256u);
            unsigned r=c.r+z.r,g=c.g+z.g,b=c.b+z.b;c=(colour_t){r>255?255:r,g>255?255:g,b>255?255:b};}
        }
        break;
      }
      case 14:
        if (dist < 8)
          c = hsv(f + dist * 20u, 255,
                  l->effect_state[dist] *
                      (200u + wave(f * 3u + dist * 30u) / 5u) / 256u);
        break;
      case 15: {
        for(unsigned k=0;k<8;k++){uint8_t *s=l->effect_state+k*6u;int trail=(int)(s[0]/8u)-(int)Y;
          if(s[5] && s[1]==X && trail>=0 && trail<s[4])
            c=hsv(s[3],trail?255:200,255u-(unsigned)trail*255u/s[4]);}
        break;
      }
      case 16:
        value =
            l->effect_state[0] * (dist < 11u ? 255u - dist * 22u : 25u) / 255u;
        c = hsv(f + (beat ? 15u : 0u) + dist * 3u, 220, value);
        break;
      case 17: {
        int amp = 4 + (int)bass * 3 / 100;
        int x1 =
            8 + ((int)wave(f * (2u + mid / 20u) + Y * 20u) - 128) * amp / 128;
        int x2 = 16 - x1;
        if ((int)X == x1)
          c = hsv(bass * 60u / 100u, 255, 180u + bass * 75u / 100u);
        else if ((int)X == x2)
          c = hsv(160u + high * 60u / 100u, 255, 180u + high * 75u / 100u);
        else if ((Y + f / 4u) % 4u == 0 && (int)X > (x1 < x2 ? x1 : x2) &&
                 (int)X < (x1 > x2 ? x1 : x2))
          c = hsv(96, 200, 60u + mid);
        break;
      }
      case 18: {
        unsigned yy=l->effect_state[X],previous=X?l->effect_state[X-1u]:yy;
        if(Y >= (yy<previous?yy:previous) && Y <= (yy>previous?yy:previous))
          c = hsv(160u - absdiff(Y, 8u) * 15u, 255,
                  150u + absdiff(Y, 8u) * 13u);
        else if (Y == 8)
          c = (colour_t){15, 15, 15};
        break;
      }
      case 19: {
        for(unsigned k=0;k<5;k++){uint8_t *s=l->effect_state+k*6u;
          int x=(int)X-s[0]/16,y=(int)Y-s[1]/16;
          if(x*x+y*y<=s[5]*s[5])c=hsv(s[4],255,200u+bass*55u/100u);}
        break;
      }
      case 20: {
        unsigned influence = 0, hues = 0;
        for (unsigned k = 0; k < 4; k++) {
          uint8_t *s=l->effect_state+k*6u;unsigned dx=absdiff(X*16u,s[0]),dy=absdiff(Y*16u,s[1]);
          unsigned dd=root(dx*dx+dy*dy);
          unsigned q=(unsigned)s[4]*256u/(dd+8u);
          influence += q;
          hues += q * s[5];
        }
        if (influence > 256u)
          c = hsv(hues / (influence ? influence : 1), 200,
                  influence > 512u ? 255u : (influence - 256u)*255u/256u);
        break;
      }
      case 21: {
        unsigned g = 0;
        switch (s->bytes[9]) {
        case 1:
          g = w > 1 ? x * 255u / (w - 1u) : 0;
          break;
        case 2:
          g = h > 1 ? y * 255u / (h - 1u) : 0;
          break;
        case 3: {
          unsigned xx = absdiff(2 * x, w - 1), yy = absdiff(2 * y, h - 1);
          unsigned max = root((w - 1) * (w - 1) + (h - 1) * (h - 1));
          g = max ? root(xx * xx + yy * yy) * 255u / max : 0;
          break;
        }
        case 4:
          g = (x + y) * 255u / (w + h - 2u + (w + h == 2));
          break;
        default:
          break;
        }
        if (s->bytes[9])
          g = (g + (f * s->bytes[2] / 10u)) % 255u;
        c = (colour_t){mix(s->bytes[3], s->bytes[6], g),
                       mix(s->bytes[4], s->bytes[7], g),
                       mix(s->bytes[5], s->bytes[8], g)};
        break;
      }
      default:
        break;
      }
      unsigned pos = (y * w + ((y & 1u) ? w - 1u - x : x)) * 3u;
      unsigned trail=e==2?200u:e==5 || e==9 || e==15?180u:e==11?220u:e==13?230u:e==8?200u:0u;
      unsigned g=(unsigned)c.g*bright/255u,r=(unsigned)c.r*bright/255u,b=(unsigned)c.b*bright/255u;
      if(trail && !g && !r && !b){g=l->frame[pos]*trail/256u;r=l->frame[pos+1]*trail/256u;b=l->frame[pos+2]*trail/256u;}
      l->frame[pos]=(uint8_t)g;l->frame[pos+1]=(uint8_t)r;l->frame[pos+2]=(uint8_t)b;
    }
  return l->io.show(l->io.context, l->frame, (size_t)w * h * 3u);
}
bool dbt_led_render(dbt_led_t *l) {
  return l && dbt_led_render_at(l,l->render_now_ms+33u);
}
bool dbt_led_render_at(dbt_led_t *l,uint32_t now_ms) {
  if(!l)return false;
  l->render_now_ms=now_ms;
  return frame(l,&l->settings,l->legacy[0]);
}
bool dbt_led_work(dbt_led_t *l) {
  if (!l)
    return false;
  unsigned rt = atomic_load_explicit(&l->request_tail, memory_order_relaxed),
           rh = atomic_load_explicit(&l->request_head, memory_order_acquire);
  unsigned wh = atomic_load_explicit(&l->receipt_head, memory_order_relaxed),
           wt = atomic_load_explicit(&l->receipt_tail, memory_order_acquire);
  if (rt == rh || wh - wt >= DBT_LED_QUEUE_CAPACITY)
    return false;
  dbt_led_request_t r = l->requests[rt % DBT_LED_QUEUE_CAPACITY];
  uint8_t legacy[10];
  memcpy(legacy, l->legacy, 10);
  if (r.operation == DBT_LED_SET_ALL)
    dbt_led_to_legacy(legacy, &r.settings);
  else if (r.operation == DBT_LED_SET_BRIGHTNESS)
    legacy[0] = to255(r.settings.bytes[1]);
  else if (r.operation == DBT_LED_ADJUST_BRIGHTNESS) {
    int b = (int)legacy[0] + r.delta;
    if (b < 0)
      b = 0;
    if (b > 255)
      b = 255;
    legacy[0] = (uint8_t)b;
  } else if (r.operation == DBT_LED_ADJUST_EFFECT ||
             r.operation == DBT_LED_PREVIEW_DELTA) {
    int e = (int)l->settings.bytes[0] + r.delta;
    if (e < 0)
      e += DBT_LED_EFFECT_COUNT;
    if (e >= (int)DBT_LED_EFFECT_COUNT)
      e -= DBT_LED_EFFECT_COUNT;
    legacy[9] = (uint8_t)e;
  } else if (r.operation == DBT_LED_CONFIRM_PREVIEW)
    legacy[9] = l->settings.bytes[0];
  else
    legacy[9] = r.settings.bytes[0];
  dbt_led_settings_t s;
  dbt_led_from_legacy(&s, legacy);
  if (r.operation == DBT_LED_SET_BRIGHTNESS ||
      r.operation == DBT_LED_ADJUST_BRIGHTNESS)
    s.bytes[0] = l->settings.bytes[0];
  bool preview = r.operation == DBT_LED_PREVIEW_EFFECT ||
                 r.operation == DBT_LED_PREVIEW_DELTA;
  dbt_led_receipt_t result = {0};
  result.token = r.token;
  if (!preview && !l->io.commit(l->io.context, legacy))
    result.result = DBT_LED_STORAGE_FAILED;
  else {
    result.committed = !preview;
    /* A failed show can follow a successful durable commit; retain that state.
     */
    if (result.committed) {
      l->settings = s;
      memcpy(l->legacy, legacy, 10);
    }
    if (!activate_effect(l,s.bytes[0]))
      result.result=DBT_LED_NO_MEMORY;
    else if (frame(l, &s, legacy[0])) {
      result.result = DBT_LED_OK;
      if (!result.committed)
        l->settings = s;
    } else
      result.result = DBT_LED_TRANSFER_FAILED;
  }
  result.settings = l->settings;
  l->receipts[wh % DBT_LED_QUEUE_CAPACITY] = result;
  atomic_store_explicit(&l->receipt_head, wh + 1, memory_order_release);
  atomic_store_explicit(&l->request_tail, rt + 1, memory_order_release);
  return true;
}
