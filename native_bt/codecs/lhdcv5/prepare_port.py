"""Generate bounded audited LHDC units without modifying the sibling decoder."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct

PINNED={
 "lhdc_dec.c":"b770f56d9b31a1b4b5272fd9ba6b3006195ceed68ef8949ec44e07631b20a7b0",
 "lhdc_imdct.c":"5eb7a01fe6e4c3e44b78129a841ab6bf5f667651111d617a6e110ddf073868de",
 "lhdc_entropy_dec.c":"698a55c083697aa9a98578415cfdf24cf0a6197f2e23c1b7f025d16a87802f70",
 "lhdc_sns_synth.c":"b6afd0bde63a78c0a10fac06dd27f26f567f11ef88e343ff4057dad29cf97715",
}

PINNED_HEADERS={
 "lhdc_bit_reader.h":"9148efdbb01b6767cadbf1f3ed5b5b7592b931db05937539ecfa54ea4aa2a21d",
 "lhdc_dec.h":"cd59bb9ba2a099a3d26cf526676b4513493fc68edd84609a3c4cd0d3128efaa6",
 "lhdc_dec_internal.h":"dece279c95755c35e3f31fce1cbc6164183830db7e1c7d95aedf34bec0c66941",
 "lhdc_dec_slot.h":"0dfa37032f39f3cb48ba2fb4eb0c479c58ba2680410def7eb0af15a40140e042",
 "lhdc_diag_config.h":"ae2224a4dd86bed16ecd4b08f8ca05b3344cec42298d44114e818ebc7605cfcd",
 "lhdc_entropy_dec.h":"b84717ce9a6f91446f349eac9809ccebac064745d67198c78e704555f19ef982",
 "lhdc_imdct.h":"6bb2d9f77ebfb8c30109423bc0d90dc1f9078fc9b36335ca9a817ad645b5c724",
 "lhdc_sns_synth.h":"848094a1254ae48313403db555d6e4e7e11125fe0f419281a2cf0afd8b8681fa",
 "lhdc_tables.h":"980d7442c3466ec44545a0be2bead28e74c4d584688dbb0129d7e980cf60ef62",
}


def once(text,old,new):
    if text.count(old)!=1:raise RuntimeError("LHDC port site changed: "+old[:80])
    return text.replace(old,new)


def generate(source,output):
    output.mkdir(parents=True,exist_ok=True);receipt={};header_receipt={}
    for name,expected in {**PINNED,**PINNED_HEADERS}.items():
        raw=(source/name).read_bytes().replace(b'\r\n',b'\n')
        if hashlib.sha256(raw).hexdigest()!=expected:raise RuntimeError("Unreviewed sibling source: "+name)
        text=raw.decode("utf-8")
        if name in ("lhdc_dec.c","lhdc_imdct.c"):
            anchor='#include <stdio.h>   /* snprintf used in g_lhdc_trace diagnostics */' if name=="lhdc_dec.c" else '#include <string.h>'
            text=once(text,anchor,anchor+'\n#include "dbt_lhdc_alloc.h"\n#define malloc dbt_lhdc_mem_alloc\n#define free dbt_lhdc_mem_free')
        if name=="lhdc_imdct.c":
            text=once(text,'static float *s480_mem = NULL;',
                      'static float *s480_mem = NULL;\nstatic float *s480_scr = NULL; /* one active codec worker */')
            text=once(text,'    if (s480_mem) { free(s480_mem); s480_mem = NULL; }',
                      '    if (s480_mem) { free(s480_mem); s480_mem = NULL; }\n    if (s480_scr) { free(s480_scr); s480_scr = NULL; }')
            text=once(text,'    {\n        float *p = s480_mem;',
                      '''    if (!s480_scr) {
        s480_scr = (float *)malloc(8u * IMDCT_P * sizeof(float));
        if (!s480_scr) { lhdc_imdct_free_480(); return; }
    }
    {
        float *p = s480_mem;''')
            text=once(text,'    float Gr[IMDCT_P], Gi[IMDCT_P];   /* G[k1*15 + n2] */',
                      '    float *Gr = s480_scr + 6u * IMDCT_P, *Gi = s480_scr + 7u * IMDCT_P;')
            text=once(text,'''    float zr[IMDCT_P], zi[IMDCT_P];
    float Xr[IMDCT_P], Xi[IMDCT_P];
    float reZ[IMDCT_P], imZ[IMDCT_P];''',
            '''    float *zr = s480_scr, *zi = zr + IMDCT_P;
    float *Xr = zi + IMDCT_P, *Xi = Xr + IMDCT_P;
    float *reZ = Xi + IMDCT_P, *imZ = reZ + IMDCT_P;''')
            for tag,width in (("960",8),("1920",16)):
                tw_name=f's{tag}_w{width*2}'
                text=once(text,f'static float {tw_name}r[{width}], {tw_name}i[{width}];',
                          f'static float *{tw_name}r, *{tw_name}i;')
                anchor=f'    s{tag}_rot_i = p; p += IMDCT_P_{tag};'
                text=once(text,anchor,anchor+f'\n    {tw_name}r = p; p += {width}; {tw_name}i = p;')
                anchor=f'    s{tag}_built = 0;\n    s{tag}_ok = 0;'
                text=once(text,anchor,f'    {tw_name}r = {tw_name}i = NULL;\n'+anchor)
                # The existing table block contains 450 unused float slots.
                # The added pointers use 2*width of those slots, without growth.
            text=once(text,'#define S960_TBL_FLOATS (2*IMDCT_N2_960*IMDCT_N2_960 \\\n                         + 2*IMDCT_N2_960*IMDCT_N1_960 + 2*IMDCT_P_960)',
                      '#define S960_TBL_FLOATS (2*IMDCT_N2_960*IMDCT_N1_960 + 2*IMDCT_P_960 + 16)')
            text=once(text,'#define S1920_TBL_FLOATS (2*IMDCT_N2_1920*IMDCT_N2_1920 \\\n                          + 2*IMDCT_N2_1920*IMDCT_N1_1920 + 2*IMDCT_P_1920)',
                      '#define S1920_TBL_FLOATS (2*IMDCT_N2_1920*IMDCT_N1_1920 + 2*IMDCT_P_1920 + 32)')
        if name=="lhdc_dec.c":
            # Classic ESP32 already compiles out the selector retry that is
            # the only reader of ov_save. Keep the portable retry/storage and
            # an explicit retained-buffer reference for paired validation.
            text=once(text,'    dec->ov_save        = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));',
                '''#if CONFIG_IDF_TARGET_ESP32 && !defined(DBT_LHDC_KEEP_UNUSED_OV_SAVE)
    dec->ov_save = NULL;
#else
    dec->ov_save        = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
#endif''')
            text=once(text,'''        !dec->pcm_r) {
#else
        !dec->ov_save) {''','''        !dec->pcm_r) {
#elif CONFIG_IDF_TARGET_ESP32 && !defined(DBT_LHDC_KEEP_UNUSED_OV_SAVE)
        0) {
#else
        !dec->ov_save) {''')
            # Six timestamps per channel were executed even with the stage
            # profiler disabled. Keep the two frame-latency timestamps, and
            # enable these stage marks only for an explicit diagnostic build.
            anchor='#define LHDC_NOW_US() esp_timer_get_time()\n#endif'
            text=once(text,anchor,anchor+"""
#if defined(DBT_LHDC_STAGE_CLOCK_CYCLES) && DBT_LHDC_STAGE_CLOCK_CYCLES
#if !defined(ESP_PLATFORM) || LHDCV5_DEC_PROFILE
#error Cycle stage profiling requires the ESP32 diagnostic without the legacy logging profiler
#endif
#include "esp_cpu.h"
/* Subtractions are cast to uint32_t at recording, so a single CCOUNT wrap
 * between adjacent marks is handled without converting cycles to time. */
#define DBT_LHDC_STAGE_NOW() ((int64_t)esp_cpu_get_cycle_count())
#elif LHDCV5_DEC_PROFILE || (defined(DBT_LHDC_STAGE_PROFILE) && DBT_LHDC_STAGE_PROFILE)
#define DBT_LHDC_STAGE_NOW() LHDC_NOW_US()
#else
#define DBT_LHDC_STAGE_NOW() ((int64_t)0)
#endif
#if defined(DBT_LHDC_STAGE_PROFILE) && DBT_LHDC_STAGE_PROFILE
extern void dbt_lhdc_stage_record(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
#endif
""")
            for variable in ('t_chan0','t_ent0','t_ent1','t_mant1','t_im0','t_im1'):
                text=once(text,variable+' = LHDC_NOW_US();',variable+' = DBT_LHDC_STAGE_NOW();')
            anchor='''    /* Steady-state stage profile. Disabled: the periodic ESP_LOGI blocks the'''
            instrument='''#if defined(DBT_LHDC_STAGE_PROFILE) && DBT_LHDC_STAGE_PROFILE
    {
        const int64_t stamp = DBT_LHDC_STAGE_NOW();
        dbt_lhdc_stage_record((uint32_t)(t_ent0-t_chan0),(uint32_t)(t_ent1-t_ent0),
            (uint32_t)(t_mant1-t_ent1),(uint32_t)(t_im0-t_mant1),
            (uint32_t)(t_im1-t_im0),(uint32_t)(stamp-t_im1));
    }
#endif
'''
            text=once(text,anchor,instrument+anchor)
            # Frame size/rate/depth are stable across most stream packets.
            # Cache the exact existing float results, never an approximation;
            # target is single-owner (LHDC_NSLOTS==1). Host diagnostic builds
            # keep their environment overrides unless explicitly testing target math.
            cache_if='#if !defined(LHDC_HOST_BUILD) || defined(DBT_LHDC_TARGET_MATH)'
            anchor='    const int OFFSET_MAX = 504;   /* (1<<9) - 8 */\n'
            text=once(text,anchor,anchor+cache_if+"""
    static int cache_l = -1, cache_depth = -1, cache_start;
    static uint32_t cache_rate;
    static float cache_jump;
    int start; float jump;
    if (L == cache_l && bit_depth == cache_depth && sample_rate == cache_rate) {
        start = cache_start; jump = cache_jump;
    } else {
#else
    int start; float jump;
#endif
""")
            text=once(text,'    int start = (int)(offset_max / divisor);',
                      '    start = (int)(offset_max / divisor);')
            text=once(text,'    float jump = (deno != 0) ? (offset_max - (float)start) / (float)deno : 0.0f;',
                      '    jump = (deno != 0) ? (offset_max - (float)start) / (float)deno : 0.0f;')
            anchor='''#endif

    float e;
    if (gain_idx <= start) {'''
            text=once(text,anchor,'''#endif
'''+cache_if+'''
        cache_l=L; cache_depth=bit_depth; cache_rate=sample_rate; cache_start=start; cache_jump=jump;
    }
#endif

    float e;
    if (gain_idx <= start) {''')
            anchor='static float lhdc_dec_rate_norm(int mdct_size)\n{\n'
            text=once(text,anchor,anchor+cache_if+'''
    if (mdct_size == 480) return 1.0f;
    if (mdct_size == 960) return 2.0f;
    if (mdct_size == 1920) return 4.0f;
#endif
''')
            anchor='static float lhdc_dec_level_cal(uint32_t sample_rate, int bit_depth)\n{\n'
            text=once(text,anchor,anchor+cache_if+'''
    static uint32_t cache_rate;
    static int cache_depth = -1;
    static float cache_level;
    if (cache_rate == sample_rate && cache_depth == bit_depth) return cache_level;
#endif
''')
            anchor='    return exp2f(db / 6.020599913f);   /* 10^(db/20) */'
            text=once(text,anchor,'''    float level = exp2f(db / 6.020599913f);   /* 10^(db/20) */
'''+cache_if+'''
    cache_rate = sample_rate; cache_depth = bit_depth; cache_level = level;
#endif
    return level;''')
            text=once(text,'#define LHDC_DEC_MALLOC(n) heap_caps_malloc((n), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)',
                      '#define LHDC_DEC_MALLOC(n) dbt_lhdc_mem_alloc(n)')
            text=once(text,'#define LHDC_DEC_FREE(p)   heap_caps_free(p)','#define LHDC_DEC_FREE(p)   dbt_lhdc_mem_free(p)')
            text=once(text,'#if defined(LHDC_HOST_BUILD)\n    /* Host-only A/B hook:',
                      '#if defined(LHDC_HOST_BUILD) && !defined(DBT_LHDC_NO_HOST_WINFILE)\n    /* Host-only A/B hook:')
            text=once(text,'''        ESP_LOGW("LHDCV5_DEC", "ch%d: sns_decode ret=%d gain=%d sfb=%d",
                 channel, ret, (int)slt->sns_params.global_gain, num_sfb);''',
                      '''        if (g_lhdc_trace > 0)
            ESP_LOGW("LHDCV5_DEC", "ch%d: sns_decode ret=%d gain=%d sfb=%d",
                     channel, ret, (int)slt->sns_params.global_gain, num_sfb);''')
            text=once(text,'        if (conceal && (s_rl++ % 16) == 0) {',
                      '        if (conceal && g_lhdc_trace > 0 && (s_rl++ % 16) == 0) {')
        if name=="lhdc_entropy_dec.c":
            text=once(text,'#include <stdlib.h>   /* malloc/free for the lazily-heaped FAC models */',
                      '#include <stdlib.h>\n#include "dbt_lhdc_alloc.h"\n#include "dbt_lhdc_bits.h"')
            text=once(text,'''    int nb = 0;
    while (lhdc_bit_reader_remaining(br) >= 8 && nb < scratch_fac_cap) {
        facbuf[nb++] = (uint8_t)lhdc_bit_reader_read(br, 8);
    }''','''    const int nb = dbt_lhdc_read_fac_bytes(br, facbuf, scratch_fac_cap);''')
            start=text.index('static uint8_t s_fac_hist1[LHDC_NSLOTS][512];')
            end=text.index('\n/* The FAC alphabet is always ternary',start)
            replacement='''typedef struct {
    uint8_t hist1[LHDC_NSLOTS][512];
    uint8_t hist2[LHDC_NSLOTS][3072];
    fac_model_t models[LHDC_NSLOTS][2];
} dbt_fac_state_t;
static dbt_fac_state_t *s_fac_state;
#define s_fac_hist1 (s_fac_state->hist1)
#define s_fac_hist2 (s_fac_state->hist2)
#define s_fac_m (s_fac_state->models)
int dbt_lhdc_entropy_ready(void) { return s_fac_state != NULL; }
void lhdc_entropy_alloc(void) {
    if (!s_fac_state) {
        s_fac_state = dbt_lhdc_mem_alloc(sizeof(*s_fac_state));
        if (s_fac_state) memset(s_fac_state, 0, sizeof(*s_fac_state));
    }
}
void lhdc_entropy_free(void) { if (s_fac_state) dbt_lhdc_mem_free(s_fac_state); s_fac_state = NULL; }
void lhdc_entropy_alloc_internal(void) {
    const int s = LHDC_DEC_SLOT();
    s_fac_m[s][0].hist = s_fac_hist1[s]; s_fac_m[s][0].hist_cap = (int)sizeof(s_fac_hist1[s]);
    s_fac_m[s][1].hist = s_fac_hist2[s]; s_fac_m[s][1].hist_cap = (int)sizeof(s_fac_hist2[s]);
}
'''
            text=text[:start]+replacement+text[end:]
            text=once(text,'    const int slot = LHDC_DEC_SLOT();       /* this core\'s private model set */',
                      '    if (!s_fac_state) return LHDC_DEC_ERROR;\n    const int slot = LHDC_DEC_SLOT();       /* this core\'s private model set */')
        if name=="lhdc_sns_synth.c":
            match=re.search(r'static const int32_t LHDC_POW2_MANT\[161\] = \{(.*?)\};',text,re.S)
            if not match:raise RuntimeError("LHDC mantissa table changed")
            values=[int(x) for x in re.findall(r'\d+',match.group(1))]
            if len(values)!=161:raise RuntimeError("Unexpected LHDC mantissa count")
            f32=lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
            table=',\n    '.join(f32(1.0/f32(v)).hex()+'f' for v in values)
            old='static float s_pow2_mant_recip[sizeof(LHDC_POW2_MANT) / sizeof(int32_t)];\nstatic int   s_pow2_mant_recip_built = 0;'
            text=once(text,old,'static const float s_pow2_mant_recip[161] = {\n    '+table+'\n};')
            old='''    if (!s_pow2_mant_recip_built) {
        for (int i = 0; i < n; i++) s_pow2_mant_recip[i] = 1.0f / (float)LHDC_POW2_MANT[i];
        s_pow2_mant_recip_built = 1;
    }
'''
            text=once(text,old,'')
            text+='''\n#ifdef DBT_LHDC_PORT_SELFTEST
int dbt_lhdc_recip_port_test(void) {
    for (unsigned i=0;i<161;++i) {
        volatile float denominator=(float)LHDC_POW2_MANT[i];
        float expected=1.0f/denominator;
        if (expected!=s_pow2_mant_recip[i]) return 0;
    }
    return 1;
}
#endif
'''
        if name=="lhdc_dec.c":
            text+="""
#ifdef DBT_LHDC_MATH_SELFTEST
#if defined(_WIN32)
#define DBT_LHDC_TEST_EXPORT __declspec(dllexport)
#else
#define DBT_LHDC_TEST_EXPORT __attribute__((visibility("default")))
#endif
DBT_LHDC_TEST_EXPORT float dbt_lhdc_test_gain(int gain,int length,int depth,uint32_t rate) {
    return lhdc_dec_gain_exponent(gain,length,depth,rate);
}
DBT_LHDC_TEST_EXPORT float dbt_lhdc_test_level(uint32_t rate,int depth) {
    return lhdc_dec_level_cal(rate,depth);
}
DBT_LHDC_TEST_EXPORT float dbt_lhdc_test_norm(int length) {
    return lhdc_dec_rate_norm(length);
}
#endif
"""
        if name=="lhdc_dec_internal.h":
            text=once(text,'#define LHDC_DEC_MAX_FRAME_BYTES     1536   /* 48k/5ms HR payload <=~660B */',
                      '#define LHDC_DEC_MAX_FRAME_BYTES     2048   /* Up to 2046 payload bytes from the 10-bit word count. */')
        data=text.encode("utf-8")
        if len(data)>262144:raise RuntimeError("Unexpected generated LHDC unit size")
        (output/name).write_bytes(data)
        records=header_receipt if name.endswith(".h") else receipt
        records[name]={"source_sha256":expected,"generated_sha256":hashlib.sha256(data).hexdigest()}
    (output/"lhdc_port.json").write_text(json.dumps(receipt,indent=2)+'\n')
    (output/"lhdc_port_headers.json").write_text(json.dumps(header_receipt,indent=2)+'\n')
    return receipt


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(json.dumps(generate(args.source,args.output),indent=2))
