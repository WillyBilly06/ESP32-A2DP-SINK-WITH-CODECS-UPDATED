/* Test-only fixture generator using the separately sourced AOSP encoder. */
#include "ldacBT.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc,char **argv){
    if(argc!=7)return 2;
    int rate=atoi(argv[1]),mode=atoi(argv[2]),quality=atoi(argv[3]);
    int depth=atoi(argv[5]);double amplitude=strtod(argv[6],NULL);
    if((depth!=16 && depth!=24) || amplitude<=0 || amplitude>=1)return 2;
    if((rate!=44100 && rate!=48000 && rate!=88200 && rate!=96000) ||
       (mode!=1 && mode!=2 && mode!=4) || quality<0 || quality>2)return 2;
    FILE *file=fopen(argv[4],"wb");if(!file)return 3;
    HANDLE_LDAC_BT h=ldacBT_get_handle();if(!h){fclose(file);return 4;}
    if(ldacBT_init_handle_encode(h,679,quality,mode,depth==16?LDACBT_SMPL_FMT_S16:LDACBT_SMPL_FMT_S24,rate)!=0){
        fprintf(stderr,"encoder init error %d\n",ldacBT_get_error_code(h));
        ldacBT_free_handle(h);fclose(file);return 5;
    }
    unsigned channels=mode==4?1u:2u,frame_total=0,bytes_total=0;
    for(unsigned block=0;block<128;++block){
        uint8_t pcm[768];unsigned char stream[LDACBT_MAX_NBYTES];int used=0,size=0,frames=0;
        for(unsigned i=0;i<128;++i)for(unsigned ch=0;ch<channels;++ch){
            unsigned n=block*128u+i,width=(unsigned)depth/8u;
            int32_t sample=(int32_t)llround(ldexp(amplitude,depth-1)*sin(6.283185307179586476925286766559*(ch?1300.0:997.0)*n/rate));
            for(unsigned byte=0;byte<width;++byte)pcm[(i*channels+ch)*width+byte]=(uint8_t)((uint32_t)sample>>(8u*byte));
        }
        if(ldacBT_encode(h,pcm,&used,stream,&size,&frames)!=0 || used!=(int)(128u*channels*(unsigned)(depth/8)) ||
           size<0 || size>LDACBT_MAX_NBYTES || frames<0 || (size && fwrite(stream,1,(size_t)size,file)!=(size_t)size)){
            fprintf(stderr,"encoder error %d at block %u\n",ldacBT_get_error_code(h),block);
            ldacBT_free_handle(h);fclose(file);return 6;
        }
        frame_total+=(unsigned)frames;bytes_total+=(unsigned)size;
    }
    for(unsigned i=0;i<4;++i){
        unsigned char stream[LDACBT_MAX_NBYTES];int used=0,size=0,frames=0;
        if(ldacBT_encode(h,NULL,&used,stream,&size,&frames)!=0 || size<0 || size>LDACBT_MAX_NBYTES ||
           (size && fwrite(stream,1,(size_t)size,file)!=(size_t)size)){
            ldacBT_free_handle(h);fclose(file);return 7;
        }
        frame_total+=(unsigned)frames;bytes_total+=(unsigned)size;if(!size)break;
    }
    ldacBT_close_handle(h);ldacBT_free_handle(h);
    if(fclose(file)!=0 || !frame_total || !bytes_total)return 8;
    printf("frames=%u bytes=%u\n",frame_total,bytes_total);return 0;
}
