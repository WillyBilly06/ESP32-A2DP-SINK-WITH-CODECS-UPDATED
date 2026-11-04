/* Test-only fixture generator using the separately sourced AOSP encoder. */
#include "ldacBT.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc,char **argv){
    if(argc!=5)return 2;
    int rate=atoi(argv[1]),mode=atoi(argv[2]),quality=atoi(argv[3]);
    if((rate!=44100 && rate!=48000 && rate!=88200 && rate!=96000) ||
       (mode!=1 && mode!=2 && mode!=4) || quality<0 || quality>2)return 2;
    FILE *file=fopen(argv[4],"wb");if(!file)return 3;
    HANDLE_LDAC_BT h=ldacBT_get_handle();if(!h){fclose(file);return 4;}
    if(ldacBT_init_handle_encode(h,679,quality,mode,LDACBT_SMPL_FMT_S16,rate)!=0){
        fprintf(stderr,"encoder init error %d\n",ldacBT_get_error_code(h));
        ldacBT_free_handle(h);fclose(file);return 5;
    }
    unsigned channels=mode==4?1u:2u,frame_total=0,bytes_total=0;
    for(unsigned block=0;block<128;++block){
        int16_t pcm[256];unsigned char stream[LDACBT_MAX_NBYTES];int used=0,size=0,frames=0;
        for(unsigned i=0;i<128;++i)for(unsigned ch=0;ch<channels;++ch){
            unsigned n=block*128u+i,phase=(n*(ch?137u:97u))%48000u;
            pcm[i*channels+ch]=(int16_t)((int)phase-24000);
        }
        if(ldacBT_encode(h,pcm,&used,stream,&size,&frames)!=0 || used!=(int)(128u*channels*2u) ||
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
