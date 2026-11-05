#include "fixed_dsp.h"
#ifndef FIXED_DSP_STAGE_MARK
#define FIXED_DSP_STAGE_MARK(stage) ((void)0)
#endif
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define DSP_HOT IRAM_ATTR
#else
#define DSP_HOT
#endif

#ifndef FIXED_DSP_TABLES_HEADER
#define FIXED_DSP_TABLES_HEADER "fixed_dsp_tables.inc"
#endif
#include FIXED_DSP_TABLES_HEADER

#define COEFF_ONE 268435456
#define AUDIO_ONE 134217728
#define UNITY_GAIN INT32_MAX
#define CLIP_THRESHOLD 114085069
#define CLIP_RANGE (AUDIO_ONE - CLIP_THRESHOLD)

/* abs(sample)<=2^31. This generated component box bounds every legal target
 * and monotone coefficient ramp. With the prior <2^28 fraction, every partial
 * five-product sum then fits int64. Unproven/custom tables keep the general
 * split path; the diagnostic reference can force it for exact comparisons. */
#if defined(FDSP_COEFF_BOX_BOUND) && FDSP_COEFF_BOX_BOUND <= 4294967295ULL && !defined(FIXED_DSP_FORCE_SPLIT_SUM)
#define FDSP_PROVEN_SUM_ENABLED 1
#else
#define FDSP_PROVEN_SUM_ENABLED 0
#endif

#if defined(FIXED_DSP_WORD_RAMP) && !defined(FIXED_DSP_WORD_QUANT)
#error Word ramp quantization requires word quantization
#endif
#if defined(FIXED_DSP_WORD_QUANT) && FDSP_PROVEN_SUM_ENABLED
#ifdef FIXED_DSP_TRANSPOSED_BLOCK
#error Select one exact block-kernel experiment
#endif
/* For sum>>28 to fit int32_t, the signed upper word must fit signed28.
 * This is a full-width test: multiplication, feedback and the Q28 remainder
 * retain their original precision. Unsigned offset arithmetic cannot overflow
 * with undefined behavior. The scalar use is separately opt-in; the conservative split sum is unchanged. */
static inline __attribute__((always_inline)) int32_t block_word_quantize(int64_t sum,uint32_t *fraction){
    uint32_t low=(uint32_t)(uint64_t)sum,high=(uint32_t)((uint64_t)sum>>32);
    if(high+UINT32_C(0x08000000)<=UINT32_C(0x0fffffff)){
        *fraction=low&0x0fffffffu;
        return (int32_t)((high<<4)|(low>>28));
    }
    *fraction=0;
    return (int32_t)high<0?INT32_MIN:INT32_MAX;
}
#endif

static inline int32_t clamp32(int64_t value) {
    return value > INT32_MAX ? INT32_MAX : value < INT32_MIN ? INT32_MIN : (int32_t)value;
}
static inline int8_t clamp_gain(int8_t value) {
    return value < -12 ? -12 : value > 12 ? 12 : value;
}

static void coefficient_target(fdsp_coefficients_t *state, const int32_t target[5],
                                 uint32_t frames, bool immediate) {
    bool changed = false;
    for (unsigned i = 0; i < 5; ++i) if (target[i] != state->target[i]) changed = true;
    if (!changed && !immediate) return;
    state->remaining = immediate ? 0 : frames;
    state->duration = frames;
    for (unsigned i = 0; i < 5; ++i) {
        state->target[i] = target[i];
        if (immediate) state->value[i] = target[i];
        int64_t difference = (int64_t)target[i] - state->value[i];
        state->step[i] = (int32_t)(difference / frames);
        state->remainder[i] = (int32_t)(difference % frames);
        state->error[i] = 0;
    }
}

static inline void coefficients_advance(fdsp_coefficients_t *state) {
    if (state->remaining == 0) return;
    for (unsigned i = 0; i < 5; ++i) {
        state->value[i] += state->step[i];
        state->error[i] += state->remainder[i];
        if (state->error[i] >= (int32_t)state->duration) {
            ++state->value[i]; state->error[i] -= (int32_t)state->duration;
        } else if (state->error[i] <= -(int32_t)state->duration) {
            --state->value[i]; state->error[i] += (int32_t)state->duration;
        }
    }
    if (--state->remaining == 0) {
        for (unsigned i = 0; i < 5; ++i) state->value[i] = state->target[i];
    }
}

static void ramp_target(fdsp_ramp_t *state, int32_t target, uint32_t frames, bool immediate) {
    if (state->target == target && !immediate) return;
    state->target = target;
    state->remaining = immediate ? 0 : frames;
    state->duration = frames;
    if (immediate) state->value = target;
    int64_t difference = (int64_t)target - state->value;
    state->step = (int32_t)(difference / frames);
    state->remainder = (int32_t)(difference % frames);
    state->error = 0;
}

static inline void ramp_advance(fdsp_ramp_t *state) {
    if (state->remaining == 0) return;
    state->value += state->step;
    state->error += state->remainder;
    if (state->error >= (int32_t)state->duration) {
        ++state->value; state->error -= (int32_t)state->duration;
    } else if (state->error <= -(int32_t)state->duration) {
        --state->value; state->error += (int32_t)state->duration;
    }
    if (--state->remaining == 0) state->value = state->target;
}

#ifdef FIXED_DSP_FORCE_FILTER_INLINE
#define FILTER_INLINE __attribute__((always_inline))
#else
#define FILTER_INLINE
#endif
static inline int32_t FILTER_INLINE filter(const int32_t c[5],
                              fdsp_history_t *history, int32_t input) {
    int32_t output;
    if (c[0] == COEFF_ONE && c[1] == 0 && c[2] == 0 && c[3] == 0 && c[4] == 0) {
        output = input;
        history->fraction = 0;
    } else {
#if FDSP_PROVEN_SUM_ENABLED
        int64_t sum = history->fraction;
        sum += (int64_t)c[0] * input;
        sum += (int64_t)c[1] * history->x1;
        sum += (int64_t)c[2] * history->x2;
        sum -= (int64_t)c[3] * history->y1;
        sum -= (int64_t)c[4] * history->y2;
#ifdef FIXED_DSP_WORD_RAMP
        output = block_word_quantize(sum,&history->fraction);
#else
        int64_t integer = sum >> 28;
        history->fraction = (uint32_t)((uint64_t)sum & 0x0fffffffu);
        output = clamp32(integer);
        if ((int64_t)output != integer) history->fraction = 0;
#endif
#else
        /* Split the Q55 sum into integer and fractional parts to avoid a
         * five-product int64 overflow. Carry the discarded fraction forward
         * instead of adding five rounding errors to a low-frequency IIR. */
        int64_t accumulator = 0;
        uint32_t fraction = history->fraction;
#define ACCUMULATE(coefficient, sample, sign) do { \
        int64_t product = (int64_t)(coefficient) * (sample); \
        if ((sign) < 0) product = -product; \
        accumulator += product >> 28; \
        fraction += (uint32_t)((uint64_t)product & 0x0fffffffu); \
    } while (0)
        ACCUMULATE(c[0], input, 1);
        ACCUMULATE(c[1], history->x1, 1);
        ACCUMULATE(c[2], history->x2, 1);
        ACCUMULATE(c[3], history->y1, -1);
        ACCUMULATE(c[4], history->y2, -1);
#undef ACCUMULATE
        accumulator += fraction >> 28;
        history->fraction = fraction & 0x0fffffffu;
        output = clamp32(accumulator);
        if ((int64_t)output != accumulator) history->fraction = 0;
#endif
    }
    history->x2 = history->x1; history->x1 = input;
    history->y2 = history->y1; history->y1 = output;
    return output;
}

static inline int32_t blend(int32_t left, int32_t right, int32_t fraction) {
    if (fraction == 0) return left;
    if (fraction == UNITY_GAIN) return right;
    return clamp32((int64_t)left + (((int64_t)right - left) * fraction >> 31));
}

void fixed_dsp_zero_analysis(fixed_dsp_t *dsp) {
    dsp->analysis_sum = 0;
    dsp->analysis_count = dsp->window_count = 0;
    for (unsigned i = 0; i < 3; ++i) {
        dsp->resonators[i][0] = dsp->resonators[i][1] = 0;
        dsp->meter_states[i] = 0; dsp->envelopes[i] = 0;
        dsp->goertzel_levels[i] = 0;
        dsp->meter_levels[i] = 0;
    }
}

bool fixed_dsp_init(fixed_dsp_t *dsp, uint32_t rate, uint32_t low_pass_hz,
                     uint32_t high_pass_hz, uint32_t reference_frames) {
    if (dsp == 0 || low_pass_hz != FDSP_TABLE_LP_HZ || high_pass_hz != FDSP_TABLE_HP_HZ) return false;
    int index = -1;
    for (unsigned i = 0; i < FDSP_RATE_COUNT; ++i) if (fdsp_rates[i] == rate) index = (int)i;
    if (index < 0) return false;
    uint8_t *bytes = (uint8_t *)dsp;
    for (unsigned i = 0; i < sizeof(*dsp); ++i) bytes[i] = 0;
    dsp->sample_rate = rate;
    dsp->rate_index = index;
    dsp->fade_frames = rate / 50u; /* 20 ms, independent of codec rate. */
    if (dsp->fade_frames == 0) dsp->fade_frames = 1;
    dsp->decimation = (uint16_t)((rate + 5512u) / 11025u);
    if (dsp->decimation == 0) dsp->decimation = 1;
    uint32_t effective_rate = rate / dsp->decimation;
    uint32_t window = (reference_frames * effective_rate + 22050u) / 44100u;
    dsp->window_frames = (uint16_t)(window < 64 ? 64 : window > 4096 ? 4096 : window);
    for (unsigned i = 0; i < 13; ++i) dsp->analysis_coefficients[i] = fdsp_analysis[index][i];
    for (unsigned band = 0; band < 2; ++band)
        for (unsigned coefficient = 0; coefficient < 5; ++coefficient)
            dsp->cross_coefficients[band][coefficient] = fdsp_cross[index][band][coefficient];
    fixed_dsp_controls(dsp, 0, 0, 0, 127, FDSP_ANALYSIS, true);
    return true;
}

void fixed_dsp_controls(fixed_dsp_t *dsp, int8_t bass, int8_t mid, int8_t treble,
                         uint8_t volume, uint8_t flags, bool immediate) {
    static const int32_t identity[5] = {COEFF_ONE, 0, 0, 0, 0};
    int8_t gains[3] = {clamp_gain(bass), clamp_gain(mid), clamp_gain(treble)};
    if (volume > 127) volume = 127;
    for (unsigned i = 0; i < 3; ++i) {
        dsp->gains[i] = gains[i];
        coefficient_target(&dsp->filters[i], fdsp_eq[dsp->rate_index][i][gains[i] + 12], dsp->fade_frames, immediate);
    }
    coefficient_target(&dsp->filters[3], fdsp_comp[dsp->rate_index][volume], dsp->fade_frames, immediate);
    coefficient_target(&dsp->filters[4], flags & FDSP_BASS_BOOST ? fdsp_boost[dsp->rate_index] : identity,
                        dsp->fade_frames, immediate);
    int32_t gain = (int32_t)((int64_t)volume * UNITY_GAIN / 127);
    ramp_target(&dsp->volume, gain, dsp->fade_frames, immediate);
    ramp_target(&dsp->bypass, flags & FDSP_BYPASS ? UNITY_GAIN : 0, dsp->fade_frames, immediate);
    ramp_target(&dsp->flip, flags & FDSP_CHANNEL_FLIP ? UNITY_GAIN : 0, dsp->fade_frames, immediate);
    if (!(flags & FDSP_ANALYSIS) && (dsp->flags & FDSP_ANALYSIS)) fixed_dsp_zero_analysis(dsp);
    dsp->flags = flags;
    dsp->requested_volume = volume;
}

static uint64_t integer_sqrt(uint64_t value) {
    uint64_t result = 0, bit = (uint64_t)1 << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) { value -= result + bit; result = (result >> 1) + bit; }
        else result >>= 1;
        bit >>= 2;
    }
    return result;
}

static void analysis(fixed_dsp_t *dsp, int32_t mono) {
    if (!(dsp->flags & FDSP_ANALYSIS)) return;
    dsp->analysis_sum += mono;
    if (++dsp->analysis_count < dsp->decimation) return;
    int32_t sample = (int32_t)(dsp->analysis_sum / dsp->decimation);
    dsp->analysis_sum = 0; dsp->analysis_count = 0;
    int32_t q15 = sample >> 16;
    for (unsigned i = 0; i < 3; ++i) {
        int64_t s0 = q15 + (((int64_t)dsp->analysis_coefficients[i] * dsp->resonators[i][0]) >> 29) - dsp->resonators[i][1];
        const int64_t bound = (int64_t)1 << 31;
        if (s0 > bound) s0 = bound;
        if (s0 < -bound) s0 = -bound;
        dsp->resonators[i][1] = dsp->resonators[i][0]; dsp->resonators[i][0] = s0;
        int64_t delta = (int64_t)sample - dsp->meter_states[i];
        dsp->meter_states[i] = clamp32((int64_t)dsp->meter_states[i] + (delta * dsp->analysis_coefficients[9 + i] >> 31));
        uint32_t amplitude = (uint32_t)(dsp->meter_states[i] < 0 ? -(int64_t)dsp->meter_states[i] : dsp->meter_states[i]);
        if (amplitude >= dsp->envelopes[i]) dsp->envelopes[i] = amplitude;
        else dsp->envelopes[i] = amplitude + (uint32_t)(((uint64_t)(dsp->envelopes[i] - amplitude) * (uint32_t)dsp->analysis_coefficients[12]) >> 31);
        dsp->meter_levels[i] = dsp->envelopes[i];
    }
    if (++dsp->window_count < dsp->window_frames) return;
    for (unsigned i = 0; i < 3; ++i) {
        int64_t real = dsp->resonators[i][0] - ((dsp->resonators[i][1] * dsp->analysis_coefficients[3 + i]) >> 30);
        int64_t imaginary = (dsp->resonators[i][1] * dsp->analysis_coefficients[6 + i]) >> 30;
        unsigned shift = 0;
        while (real > (1 << 30) || real < -(1 << 30) || imaginary > (1 << 30) || imaginary < -(1 << 30)) {
            real >>= 1; imaginary >>= 1; ++shift;
        }
        uint64_t power = (uint64_t)(real * real) + (uint64_t)(imaginary * imaginary);
        uint64_t magnitude = integer_sqrt(power) << shift;
        uint64_t normalized = (magnitude << 17) / dsp->window_frames;
        dsp->goertzel_levels[i] = normalized > 2147483648u ? 2147483648u : (uint32_t)normalized;
        dsp->resonators[i][0] = dsp->resonators[i][1] = 0;
    }
    dsp->window_count = 0;
}

static inline int32_t limit(fixed_dsp_t *dsp, int32_t value) {
    uint32_t magnitude = (uint32_t)(value < 0 ? -(int64_t)value : value);
    if (magnitude > dsp->peak_q27) dsp->peak_q27 = magnitude;
    if (magnitude <= CLIP_THRESHOLD) return value;
    ++dsp->limited;
    if (magnitude >= AUDIO_ONE) ++dsp->over_full_scale;
    uint32_t excess = magnitude - CLIP_THRESHOLD;
    int32_t output = CLIP_THRESHOLD + (int32_t)((uint64_t)CLIP_RANGE * excess / (CLIP_RANGE + excess));
    return value < 0 ? -output : output;
}

void DSP_HOT fixed_dsp_process(fixed_dsp_t *dsp, int32_t *left, int32_t *right) {
    FIXED_DSP_STAGE_MARK(0);
    analysis(dsp, (int32_t)(((int64_t)*left + *right) / 2));
    FIXED_DSP_STAGE_MARK(1);
    for (unsigned i = 0; i < 5; ++i) coefficients_advance(&dsp->filters[i]);
    ramp_advance(&dsp->volume); ramp_advance(&dsp->bypass); ramp_advance(&dsp->flip);
    int32_t l = *left >> 4, r = *right >> 4;
    FIXED_DSP_STAGE_MARK(2);
    for (unsigned i = 0; i < 4; ++i) {
        l = filter(dsp->filters[i].value, &dsp->channel_history[i][0], l);
        r = filter(dsp->filters[i].value, &dsp->channel_history[i][1], r);
    }
    FIXED_DSP_STAGE_MARK(3);
    int32_t lp = filter(dsp->cross_coefficients[0], &dsp->cross_history[0], l);
    int32_t hp = filter(dsp->cross_coefficients[1], &dsp->cross_history[1], r);
    lp = filter(dsp->filters[4].value, &dsp->boost_history[2], lp);
    l = filter(dsp->filters[4].value, &dsp->boost_history[0], l);
    r = filter(dsp->filters[4].value, &dsp->boost_history[1], r);
    lp = clamp32(((int64_t)lp * 189246996) >> 27); /* 1.41 crossover makeup */
    hp = clamp32(((int64_t)hp * 189246996) >> 27);
    FIXED_DSP_STAGE_MARK(4);
    int32_t split_l = blend(lp, hp, dsp->flip.value);
    int32_t split_r = blend(hp, lp, dsp->flip.value);
    l = blend(split_l, l, dsp->bypass.value);
    r = blend(split_r, r, dsp->bypass.value);
    if (dsp->volume.value != UNITY_GAIN) {
        l = clamp32(((int64_t)l * dsp->volume.value) >> 31);
        r = clamp32(((int64_t)r * dsp->volume.value) >> 31);
    }
    *left = clamp32((int64_t)limit(dsp, l) * 16);
    *right = clamp32((int64_t)limit(dsp, r) * 16);
    FIXED_DSP_STAGE_MARK(5);
}

#ifdef FIXED_DSP_BLOCK_API
#if defined(__GNUC__) && !defined(__clang__)
#define BLOCK_KERNEL __attribute__((optimize("O3"),noinline))
#else
#define BLOCK_KERNEL
#endif
/* One channel at a time keeps coefficient/history reuse within LX6's register
 * budget. It changes neither arithmetic nor the carried Q28 fraction. */
static BLOCK_KERNEL void filter_steady_block(const int32_t c[5],fdsp_history_t *h,
                                             int32_t *pcm,unsigned frames) {
    if(!frames)return;
    if(c[0]==COEFF_ONE && !c[1] && !c[2] && !c[3] && !c[4]){
        /* Identity still advances all history, including a one-frame block
         * immediately following a nonidentity filter. */
        int32_t last=pcm[2u*(frames-1u)];
        h->x2=frames>1u?pcm[2u*(frames-2u)]:h->x1;
        h->y2=frames>1u?pcm[2u*(frames-2u)]:h->y1;
        h->x1=h->y1=last;h->fraction=0;return;
    }
#if FDSP_PROVEN_SUM_ENABLED
    int32_t c0=c[0],c1=c[1],c2=c[2],c3=c[3],c4=c[4];
#ifdef FIXED_DSP_TRANSPOSED_BLOCK
    /* Exact product-domain state, rebuilt from canonical history for each
     * stable block. No state quantization: z1/z2 exclude the carried fraction.
     * The same coefficient-box bound covers these four/two-product states.
     * Ramping samples never enter this helper. */
    int32_t last_input=pcm[2u*(frames-1u)],previous_input=frames>1u?pcm[2u*(frames-2u)]:h->x1;
    int32_t previous_output=h->y1;
    int64_t z1=(int64_t)c1*h->x1+(int64_t)c2*h->x2-(int64_t)c3*h->y1-(int64_t)c4*h->y2;
    int64_t z2=(int64_t)c2*h->x1-(int64_t)c4*h->y1;
    uint32_t fraction=h->fraction;
    for(unsigned i=0;i<frames;++i){
        int32_t input=pcm[2u*i];int64_t sum=(int64_t)c0*input+z1+fraction;
        int64_t integer=sum>>28;int32_t output=clamp32(integer);
        fraction=(int64_t)output==integer?(uint32_t)((uint64_t)sum&0x0fffffffu):0;
        z1=(int64_t)c1*input-(int64_t)c3*output+z2;
        z2=(int64_t)c2*input-(int64_t)c4*output;
        pcm[2u*i]=output;
    }
    h->x1=last_input;h->x2=previous_input;h->y2=frames>1u?pcm[2u*(frames-2u)]:previous_output;
    h->y1=pcm[2u*(frames-1u)];h->fraction=fraction;
#else
    int32_t x1=h->x1,x2=h->x2,y1=h->y1,y2=h->y2;uint32_t fraction=h->fraction;
    for(unsigned i=0;i<frames;++i){
        int32_t input=pcm[2u*i];int64_t sum=fraction;
        sum+=(int64_t)c0*input;sum+=(int64_t)c1*x1;sum+=(int64_t)c2*x2;
        sum-=(int64_t)c3*y1;sum-=(int64_t)c4*y2;
#ifdef FIXED_DSP_WORD_QUANT
        int32_t output=block_word_quantize(sum,&fraction);
#else
        int64_t integer=sum>>28;int32_t output=clamp32(integer);
        fraction=(int64_t)output==integer?(uint32_t)((uint64_t)sum&0x0fffffffu):0;
#endif
        x2=x1;x1=input;y2=y1;y1=output;pcm[2u*i]=output;
    }
    h->x1=x1;h->x2=x2;h->y1=y1;h->y2=y2;h->fraction=fraction;
#endif
#else
    /* Custom/unproven tables retain the original overflow-safe split sum. */
    for(unsigned i=0;i<frames;++i)pcm[2u*i]=filter(c,h,pcm[2u*i]);
#endif
}
bool fixed_dsp_process_block(fixed_dsp_t *dsp,int32_t *pcm,uint16_t frames){
    if(!dsp || frames>FIXED_DSP_BLOCK_FRAMES || (frames && (!pcm || (uintptr_t)pcm%4u)))return false;
    if(!frames)return true;
    /* Target measurements favor the canonical kernel during coefficient
     * ramps. Process exactly that prefix, then use block reuse only once all
     * coefficients are stable. Volume/flip/bypass still advance per sample. */
    uint32_t prefix=0;
    for(unsigned f=0;f<5;++f)if(dsp->filters[f].remaining>prefix)prefix=dsp->filters[f].remaining;
    if(prefix>frames)prefix=frames;
    for(unsigned i=0;i<prefix;++i)fixed_dsp_process(dsp,&pcm[2u*i],&pcm[2u*i+1u]);
    pcm+=2u*prefix;frames=(uint16_t)(frames-prefix);
    if(!frames)return true;
    int32_t split[FIXED_DSP_BLOCK_FRAMES*2u];
    for(unsigned i=0;i<frames;++i){
        analysis(dsp,(int32_t)(((int64_t)pcm[2u*i]+pcm[2u*i+1u])/2));
        pcm[2u*i]>>=4;pcm[2u*i+1u]>>=4;
    }
    for(unsigned f=0;f<4;++f){
        filter_steady_block(dsp->filters[f].value,&dsp->channel_history[f][0],pcm,frames);
        filter_steady_block(dsp->filters[f].value,&dsp->channel_history[f][1],pcm+1,frames);
    }
    for(unsigned i=0;i<2u*frames;++i)split[i]=pcm[i];
    filter_steady_block(dsp->cross_coefficients[0],&dsp->cross_history[0],split,frames);
    filter_steady_block(dsp->cross_coefficients[1],&dsp->cross_history[1],split+1,frames);
    filter_steady_block(dsp->filters[4].value,&dsp->boost_history[2],split,frames);
    filter_steady_block(dsp->filters[4].value,&dsp->boost_history[0],pcm,frames);
    filter_steady_block(dsp->filters[4].value,&dsp->boost_history[1],pcm+1,frames);
    for(unsigned i=0;i<frames;++i){
        ramp_advance(&dsp->volume);ramp_advance(&dsp->bypass);ramp_advance(&dsp->flip);
        int32_t lp=clamp32(((int64_t)split[2u*i]*189246996)>>27);
        int32_t hp=clamp32(((int64_t)split[2u*i+1u]*189246996)>>27);
        int32_t split_l=blend(lp,hp,dsp->flip.value),split_r=blend(hp,lp,dsp->flip.value);
        int32_t l=blend(split_l,pcm[2u*i],dsp->bypass.value);
        int32_t r=blend(split_r,pcm[2u*i+1u],dsp->bypass.value);
        if(dsp->volume.value!=UNITY_GAIN){l=clamp32(((int64_t)l*dsp->volume.value)>>31);r=clamp32(((int64_t)r*dsp->volume.value)>>31);}
        pcm[2u*i]=clamp32((int64_t)limit(dsp,l)*16);
        pcm[2u*i+1u]=clamp32((int64_t)limit(dsp,r)*16);
    }
    return true;
}
#undef BLOCK_KERNEL
#endif

int32_t fixed_dsp_level_db_q8(uint32_t level) {
    if (level == 0) return -120 * 256;
    unsigned msb = 31u - (unsigned)__builtin_clz(level);
    uint64_t normalized = (uint64_t)level << (31u - msb);
    unsigned fraction = (unsigned)(normalized >> 23) & 255u;
    int32_t db = ((int32_t)msb - 31) * 1541 + fdsp_db_fraction_q8[fraction];
    return db < -120 * 256 ? -120 * 256 : db > 0 ? 0 : db;
}
