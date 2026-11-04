/* Compile the existing LGPL SDK implementation directly from its original
 * directory. Do not compile freeaptx.c a second time in the same component. */
#include "freeaptx.c"

size_t dbt_aptx_backend_size(void){return sizeof(struct aptx_context);}
struct aptx_context *dbt_aptx_backend_init(void *memory,int hd){
    struct aptx_context *ctx=memory;
    if(!ctx)return NULL;
    ctx->hd=hd?1:0;aptx_reset(ctx);return ctx;
}
