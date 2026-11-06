/* Compile the existing LGPL SDK implementation directly from its original
 * directory. Do not compile freeaptx.c a second time in the same component. */
#include "freeaptx.c"
#include <stdbool.h>

size_t dbt_aptx_backend_size(void){return sizeof(struct aptx_context);}
struct aptx_context *dbt_aptx_backend_init(void *memory,int hd){
    struct aptx_context *ctx=memory;
    if(!ctx)return NULL;
    ctx->hd=hd?1:0;aptx_reset(ctx);return ctx;
}

/* At most one codeword of new input and eight output samples per call. The
 * public sync API can inspect its <=5 cached bytes as well. Feeding only one
 * codeword avoids its local input-size precedence bug: output capacity cannot
 * be mistaken for a parity failure. Never pass it an entire A2DP packet.
 * Reuse the existing PCM scratch; expand packed S24 backwards into S32 Q31. */
size_t dbt_aptx_backend_recover32(struct aptx_context *ctx,const uint8_t *input,
    size_t input_size,uint8_t *output,size_t *written,uint8_t *run,bool *ready,size_t *discarded){
    const size_t quantum=ctx->hd?6u:4u;
    if(input_size>quantum)input_size=quantum;
    size_t packed=0;int synced=0;
    size_t used=aptx_decode_sync(ctx,input,input_size,output,24,&packed,&synced,discarded);
    if(!synced || ctx->decode_dropped)*run=0;
    else if(*discarded)*run=(LATENCY_SAMPLES+3)/4;
    else if(*run<(LATENCY_SAMPLES+3)/4)++*run;
    *ready=*run>=(LATENCY_SAMPLES+3)/4 && !ctx->decode_sync_buffer_len && !ctx->decode_dropped;
    if(packed>24 || packed%6){*written=0;*ready=false;return 0;}
    for(size_t samples=packed/3;samples;--samples){
        size_t src=3u*(samples-1u),dst=4u*(samples-1u);
        uint8_t lo=output[src],mid=output[src+1u],hi=output[src+2u];
        output[dst]=0;output[dst+1u]=lo;output[dst+2u]=mid;output[dst+3u]=hi;
    }
    *written=(packed/3u)*4u;return used;
}

#ifdef DBT_APTX_SNAPSHOT
#include <stddef.h>
uint32_t dbt_aptx_backend_snapshot_schema(void){
    uint32_t h=2166136261u;
#define H(v) h=(h^(uint32_t)(v))*16777619u
#define T(t) H(sizeof(t))
#define F(t,f) H(offsetof(t,f));H(sizeof(((t*)0)->f))
    T(size_t);T(struct aptx_context);T(struct aptx_channel);T(struct aptx_QMF_analysis);T(struct aptx_filter_signal);
    T(struct aptx_quantize);T(struct aptx_invert_quantize);T(struct aptx_prediction);
    F(struct aptx_context,decode_sync_packets);F(struct aptx_context,decode_dropped);F(struct aptx_context,channels);F(struct aptx_context,hd);F(struct aptx_context,sync_idx);
    F(struct aptx_context,encode_remaining);F(struct aptx_context,decode_skip_leading);F(struct aptx_context,decode_sync_buffer_len);F(struct aptx_context,decode_sync_buffer);
    F(struct aptx_channel,codeword_history);F(struct aptx_channel,dither_parity);F(struct aptx_channel,dither);F(struct aptx_channel,qmf);F(struct aptx_channel,quantize);F(struct aptx_channel,invert_quantize);F(struct aptx_channel,prediction);
    F(struct aptx_QMF_analysis,outer_filter_signal);F(struct aptx_QMF_analysis,inner_filter_signal);F(struct aptx_filter_signal,buffer);F(struct aptx_filter_signal,pos);
    F(struct aptx_quantize,quantized_sample);F(struct aptx_quantize,quantized_sample_parity_change);F(struct aptx_quantize,error);
    F(struct aptx_invert_quantize,quantization_factor);F(struct aptx_invert_quantize,factor_select);F(struct aptx_invert_quantize,reconstructed_difference);
    F(struct aptx_prediction,prev_sign);F(struct aptx_prediction,s_weight);F(struct aptx_prediction,d_weight);F(struct aptx_prediction,pos);F(struct aptx_prediction,reconstructed_differences);
    F(struct aptx_prediction,previous_reconstructed_sample);F(struct aptx_prediction,predicted_difference);F(struct aptx_prediction,predicted_sample);
#undef F
#undef T
#undef H
    return h;
}
#endif
