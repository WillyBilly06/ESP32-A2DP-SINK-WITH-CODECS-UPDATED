/* Test-only deterministic fixture generator using the separate C encoder. */
#include "lhdcv5_enc.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc,char **argv){
    if(argc!=5 && argc!=6 && argc!=7)return 2;
    unsigned rate=(unsigned)strtoul(argv[1],0,10),depth=(unsigned)strtoul(argv[2],0,10),quality=(unsigned)strtoul(argv[3],0,10);
    if((rate!=44100 && rate!=48000 && rate!=96000 && rate!=192000) || (depth!=16 && depth!=24) || quality>9)return 2;
    double amplitude=argc>=6?strtod(argv[5],0):0.0;
    unsigned pattern=argc==7?(unsigned)strtoul(argv[6],0,10):0u;
    if(pattern>1u || (argc>=6 && !(amplitude>0.0 && amplitude<=1.0)))return 2;
    FILE *out=fopen(argv[4],"wb");if(!out)return 3;
    lhdcv5_enc_t *enc=lhdcv5_enc_new(LHDC_VERSION_1);if(!enc){fclose(out);return 4;}
    if(lhdcv5_enc_init_encoder(enc,rate,depth,quality==9?5:quality,LHDC_MTU_3MBPS,LHDC_ENC_INTERVAL_20MS)!=0){lhdcv5_enc_free(enc);fclose(out);return 5;}
    uint32_t samples=0;if(lhdcv5_enc_get_block_size(enc,&samples)!=0 || !samples || samples>960){lhdcv5_enc_free(enc);fclose(out);return 6;}
    uint8_t input[960*2*3],packet[4096];unsigned packets=0,total_frames=0;
    for(unsigned frame=0;frame<96;++frame){
        /* Test-only index 9 alternates 400/1000 kbps every 16 frames. */
        if(quality==9 && frame%16==0 && lhdcv5_enc_set_bitrate_index(enc,(frame/16)%2?8:5,true)!=0){lhdcv5_enc_free(enc);fclose(out);return 10;}
        unsigned width=depth/8;
        for(unsigned i=0;i<samples;++i)for(unsigned ch=0;ch<2;++ch){
            double a=2.0*3.14159265358979323846*(ch?1300.0:997.0)*(double)(frame*samples+i)/(double)rate;
            double level=argc>=6?(depth==16?32767.0:8388607.0)*amplitude:(depth==16?18000.0:4608000.0);
            double unit=sin(a);
            if(pattern){
                uint32_t bits=(frame*samples+i)*747796405u+ch*2891336453u+0x632be59bu;
                bits^=bits>>16;bits*=2246822519u;bits^=bits>>13;
                unit=((double)(bits&65535u)-32768.0)/32768.0;
            }
            int32_t v=(int32_t)(unit*level);
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
