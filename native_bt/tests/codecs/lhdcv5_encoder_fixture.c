/* Test-only deterministic fixture generator using the separate C encoder. */
#include "lhdcv5_enc.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc,char **argv){
    if(argc!=5)return 2;
    unsigned rate=(unsigned)strtoul(argv[1],0,10),depth=(unsigned)strtoul(argv[2],0,10),quality=(unsigned)strtoul(argv[3],0,10);
    if((rate!=44100 && rate!=48000 && rate!=96000 && rate!=192000) || (depth!=16 && depth!=24) || quality>8)return 2;
    FILE *out=fopen(argv[4],"wb");if(!out)return 3;
    lhdcv5_enc_t *enc=lhdcv5_enc_new(LHDC_VERSION_1);if(!enc){fclose(out);return 4;}
    if(lhdcv5_enc_init_encoder(enc,rate,depth,quality,LHDC_MTU_3MBPS,LHDC_ENC_INTERVAL_20MS)!=0){lhdcv5_enc_free(enc);fclose(out);return 5;}
    uint32_t samples=0;if(lhdcv5_enc_get_block_size(enc,&samples)!=0 || !samples || samples>960){lhdcv5_enc_free(enc);fclose(out);return 6;}
    uint8_t input[960*2*3],packet[4096];unsigned packets=0,total_frames=0;
    for(unsigned frame=0;frame<96;++frame){
        unsigned width=depth/8;
        for(unsigned i=0;i<samples;++i)for(unsigned ch=0;ch<2;++ch){
            double a=2.0*3.14159265358979323846*(ch?1300.0:997.0)*(double)(frame*samples+i)/(double)rate;
            int32_t v=(int32_t)(sin(a)*(depth==16?18000.0:4608000.0));
            unsigned at=(i*2+ch)*width;
            for(unsigned j=0;j<width;++j)input[at+j]=(uint8_t)((uint32_t)v>>(8*j));
        }
        uint32_t written=0,count=0;
        if(lhdcv5_enc_encode(enc,input,samples*2u*width,packet,sizeof(packet),&written,&count)!=0 || written>2046u || count>15u){
            lhdcv5_enc_free(enc);fclose(out);return 7;
        }
        if(written){
            uint8_t header[4]={(uint8_t)written,(uint8_t)(written>>8),(uint8_t)count,(uint8_t)(count>>8)};
            if(fwrite(header,1,4,out)!=4 || fwrite(packet,1,written,out)!=written){lhdcv5_enc_free(enc);fclose(out);return 8;}
            ++packets;total_frames+=count;
        }
    }
    lhdcv5_enc_free(enc);if(fclose(out)!=0 || !packets || !total_frames)return 9;
    printf("packets=%u frames=%u samples=%u\n",packets,total_frames,samples);return 0;
}
