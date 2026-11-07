#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static uint32_t model_rescales;
#include "lhdc_model_under_test.h"
static uint32_t random_word(uint32_t *s){uint32_t x=*s;x^=x<<13;x^=x>>17;x^=x<<5;return *s=x;}
int main(void){
    const int windows[]={1,2,3,31,32,64,65,96,114,126,128,512,3072};
    uint8_t left_hist[3074],right_hist[3074];
    uint32_t checked=0,cancelled=0,cancelled_rescales=0;
    for(unsigned wi=0;wi<sizeof(windows)/sizeof(windows[0]);++wi)for(unsigned initial=0;initial<2;++initial)for(unsigned pattern=0;pattern<8;++pattern){
        memset(left_hist,0xa5,sizeof(left_hist));memset(right_hist,0xa5,sizeof(right_hist));
        fac_model_t a={0},b={0};a.hist=left_hist+1;b.hist=right_hist+1;a.hist_cap=b.hist_cap=3072;
        const uint32_t f[2][3]={{54,12,1},{1,1,1}};
        fac_model_init(&a,f[initial],windows[wi]);fac_model_init(&b,f[initial],windows[wi]);uint32_t seed=0x53917325u;
        for(unsigned i=0;i<50000;++i){
            uint32_t rnd=random_word(&seed);int sym=pattern<3?(int)pattern:pattern==3?(int)(i%3):pattern==4?(rnd%10?0:(int)(rnd%3)):pattern==5?(int)(rnd%3):pattern==6?(int)((i/200)%3):(i%97?0:(int)(rnd%3));
            /* Force threshold equality periodically: a cancelled frequency
             * update must still refresh stale cumulative frequencies. */
            if((i%257)==0)for(unsigned k=0;k<3;++k)a.thresh[k]=b.thresh[k]=a.freq[k];
            bool cancel=a.count>=a.window && a.hist[a.pos]==sym && a.freq[sym]>=2;
            uint32_t before=model_rescales;reference_fac_model_update(&a,sym);uint32_t calls=model_rescales-before;
            before=model_rescales;candidate_fac_model_update(&b,sym);assert(model_rescales-before==calls);
            uint8_t *saved=b.hist;b.hist=a.hist;assert(!memcmp(&a,&b,sizeof(a)));b.hist=saved;
            assert(!memcmp(left_hist,right_hist,sizeof(left_hist)));assert(left_hist[0]==0xa5 && left_hist[3073]==0xa5);
            cancelled+=cancel;cancelled_rescales+=cancel && calls;checked++;
        }
    }
    assert(checked==10400000 && cancelled>0 && cancelled_rescales>0);
    printf("PASS %u complete FAC model/history transitions; cancelled=%u; still_rescaled=%u\n",checked,cancelled,cancelled_rescales);
    return 0;
}
