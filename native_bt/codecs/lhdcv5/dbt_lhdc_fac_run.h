#ifndef DBT_LHDC_FAC_RUN_H
#define DBT_LHDC_FAC_RUN_H
/* Included after the pinned FAC types and single-symbol implementation. The
 * decoder owns disjoint FAC input, history and pass-1 output allocations. */
_Static_assert(FAC_TOTAL_BITS==15 && FAC_NSYM==3,"FAC run specialization");
#ifdef DBT_LHDC_FAC_RUN_TEST
static unsigned dbt_run_entered,dbt_run_symbols,dbt_run_padding;
#endif
static LHDC_HOT LHDC_NOINLINE void dbt_lhdc_fac_run(fac_dec_t *d,fac_model_t *m,uint8_t *out,int count)
{
#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace;
    if(g_lhdc_trace>0){
        for(int i=0;i<count;++i)out[i]=(uint8_t)fac_decode(d,m);
        return;
    }
#endif
    int i=0;
    while(i<count){
        if(m->count>=m->window && m->freq[0]>=2 && m->freq[0]<m->thresh[0]){
            const uint8_t *const hist=m->hist,*const data=d->data;
            const int window=m->window,len=d->len;
            const uint32_t c1=m->cum[1];
            uint32_t range=d->range,code=d->code;
            int pos=m->pos,p=d->p;
#ifdef DBT_LHDC_FAC_RUN_TEST
            ++dbt_run_entered;
#endif
            while(i<count && hist[pos]==0){
                uint32_t next=(range>>FAC_TOTAL_BITS)*c1;
                if(next<=code)break;
                range=next;
                if(range<(1u<<24)){
                    do{
#ifdef DBT_LHDC_FAC_RUN_TEST
                        if(p>=len)++dbt_run_padding;
#endif
                        code=(code<<8)|((p<len)?data[p]:0);
                        ++p;range<<=8;
                    }while(range<(1u<<24));
                }
                out[i++]=0;
                if(++pos>=window)pos=0;
#ifdef DBT_LHDC_FAC_RUN_TEST
                ++dbt_run_symbols;
#endif
            }
            /* Each accepted zero replaces a zero and cancels a permitted
             * decrement/increment. Frequencies/cumulative tables stay fixed;
             * the strict threshold gate preserves every rescale boundary. */
            d->range=range;d->code=code;d->p=p;m->pos=pos;
        }
        if(i<count)out[i++]=(uint8_t)fac_decode(d,m);
    }
}
#endif
