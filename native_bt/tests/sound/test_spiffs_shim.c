/* Compile the actual ESP32 provider against an in-memory SPIFFS/NVS shim. */
#include "dbt_sound_esp32.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "nvs.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdio.h>
#include <setjmp.h>
#include <string.h>
#define ASSERT(x) do {if(!(x)){fprintf(stderr,"assertion %d: %s\n",__LINE__,#x);return 1;}}while(0)
typedef struct {char name[64];uint8_t data[256];size_t len,pos;bool exists;} fake_file;
static fake_file files[16];static int fail_new_rename,fail_old_rename,fail_fsync,fail_mute_commit,fail_intent_write,fail_backup_unlink,fail_intent_unlink;static uint8_t saved_mute;static bool mute_exists;
static jmp_buf crash;static int cut_at,cut_count;
static void cutpoint(void){if(cut_at && ++cut_count==cut_at)longjmp(crash,1);}
static int find(const char *name){for(int i=0;i<16;i++)if(files[i].exists && !strcmp(files[i].name,name))return i;return -1;}
static int fake_open(const char *name,int flags,...){
    int i=find(name);if(i>=0 && (flags&O_EXCL)){errno=EEXIST;return -1;}
    if(i<0){for(i=0;i<16 && files[i].exists;i++){}if(i==16 || strlen(name)>=sizeof(files[i].name)){errno=ENOSPC;return -1;}memset(&files[i],0,sizeof(files[i]));files[i].exists=true;memcpy(files[i].name,name,strlen(name)+1);}
    cutpoint();return i+3;
}
static int fake_close(int fd){(void)fd;return 0;}
static FILE *fake_fdopen(int fd,const char *mode){(void)mode;return (FILE *)&files[fd-3];}
static FILE *fake_fopen(const char *name,const char *mode){(void)mode;int i=find(name);if(i<0)return 0;files[i].pos=0;return (FILE *)&files[i];}
static int fake_fseek(FILE *fp,long offset,int whence){fake_file *f=(fake_file*)fp;if(whence!=SEEK_SET || offset<0 || (size_t)offset>sizeof(f->data))return -1;f->pos=(size_t)offset;return 0;}
static size_t fake_fread(void *buf,size_t size,size_t n,FILE *fp){fake_file *f=(fake_file*)fp;size_t bytes=size*n;if(bytes>f->len-f->pos)return 0;memcpy(buf,f->data+f->pos,bytes);f->pos+=bytes;return n;}
static size_t fake_fwrite(const void *buf,size_t size,size_t n,FILE *fp){fake_file *f=(fake_file*)fp;size_t bytes=size*n;if(fail_intent_write && strstr(f->name,".txn")){bytes=8;memcpy(f->data,buf,bytes);f->len=bytes;return 0;}if(bytes>sizeof(f->data)-f->pos)return 0;memcpy(f->data+f->pos,buf,bytes);f->pos+=bytes;if(f->pos>f->len)f->len=f->pos;cutpoint();return n;}
static int fake_fflush(FILE *fp){(void)fp;cutpoint();return 0;}
static int fake_fileno(FILE *fp){return (int)((fake_file*)fp-files)+3;}
static int fake_fsync(int fd){(void)fd;if(fail_fsync)return -1;cutpoint();return 0;}
static int fake_fclose(FILE *fp){(void)fp;cutpoint();return 0;}
static int fake_stat(const char *path,struct stat *st){int i=find(path);if(i<0){errno=ENOENT;return -1;}memset(st,0,sizeof(*st));st->st_size=(long)files[i].len;return 0;}
static int fake_unlink(const char *path){int i=find(path);if(i<0 || (fail_backup_unlink && strstr(path,".bak")) || (fail_intent_unlink && strstr(path,".txn"))){errno=ENOENT;return -1;}files[i].exists=false;cutpoint();return 0;}
static int fake_rename(const char *a,const char *b){int i=find(a);if(i<0 || find(b)>=0 || strlen(b)>=sizeof(files[0].name) || (fail_new_rename && strstr(a,".tmp")) || (fail_old_rename && strstr(b,".bak"))){errno=EEXIST;return -1;}memcpy(files[i].name,b,strlen(b)+1);cutpoint();return 0;}
esp_err_t esp_spiffs_info(const char *n,size_t *t,size_t *u){(void)n;*t=4096;*u=0;return ESP_OK;}
int64_t esp_timer_get_time(void){return 0;}
esp_err_t nvs_open(const char *name,int flags,nvs_handle_t *h){(void)flags;if(strcmp(name,"audio"))return 1;*h=1;return 0;}
esp_err_t nvs_get_u8(nvs_handle_t h,const char *key,uint8_t *p){(void)h;if(strcmp(key,"sound_muted"))return 1;if(!mute_exists)return ESP_ERR_NVS_NOT_FOUND;*p=saved_mute;return 0;}
esp_err_t nvs_set_u8(nvs_handle_t h,const char *key,uint8_t p){(void)h;if(strcmp(key,"sound_muted"))return 1;saved_mute=p;mute_exists=true;return 0;}
esp_err_t nvs_commit(nvs_handle_t h){(void)h;return fail_mute_commit?1:0;}
void nvs_close(nvs_handle_t h){(void)h;}
#define open fake_open
#define close fake_close
#define fdopen fake_fdopen
#define fopen fake_fopen
#define fseek fake_fseek
#define fread fake_fread
#define fwrite fake_fwrite
#define fflush fake_fflush
#define fileno fake_fileno
#define fsync fake_fsync
#define fclose fake_fclose
#define stat(path, st) fake_stat(path, st)
#define unlink fake_unlink
#define rename fake_rename
#include "../../src/esp32/dbt_sound_spiffs.c"
static bool peer_check(void *v,dbt_sound_peer_t peer){(void)v;return peer.generation!=0;}
static void wav(uint8_t *b,uint8_t sample){
    memset(b,0,48);memcpy(b,"RIFF",4);b[4]=40;memcpy(b+8,"WAVEfmt ",8);b[16]=16;b[20]=1;b[22]=2;
    b[24]=0x80;b[25]=0xbb;b[28]=0x00;b[29]=0xee;b[30]=2;b[32]=4;b[34]=16;memcpy(b+36,"data",4);b[40]=4;
    b[44]=sample;b[45]=1;b[46]=sample;b[47]=2;
}
static bool same_persisted(const fake_file *a,const fake_file *b){
    for(unsigned i=0;i<16;i++)if(a[i].exists!=b[i].exists || (a[i].exists &&
        (strcmp(a[i].name,b[i].name) || a[i].len!=b[i].len || memcmp(a[i].data,b[i].data,a[i].len))))return false;
    return true;
}
static int crash_cuts(void){
    static fake_file saved[16];static uint8_t old[48],new_data[48];wav(old,3);wav(new_data,7);
    static int replacing,cut,completed;
    for(replacing=0;replacing<2;replacing++){
        completed=0;
        for(cut=1;cut<100;cut++){
            memset(files,0,sizeof(files));
            if(replacing){files[0].exists=true;memcpy(files[0].name,"/spiffs/startup.wav",20);memcpy(files[0].data,old,48);files[0].len=48;}
            dbt_sound_esp32_t ctx;dbt_sound_store_t s;bool muted;dbt_sound_recovery_t r;
            ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK);
            cut_count=0;cut_at=cut;
            if(setjmp(crash)==0){
                ASSERT(s.begin(s.context,0,48) && s.write(s.context,0,new_data,48) && s.seal(s.context) && s.commit(s.context,0));
                completed=1;
            }
            cut_at=0;
            ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK);
            ASSERT(dbt_sound_esp32_inspect(&ctx,0,&r)==DBT_OK);
            if(r.state==DBT_SOUND_RECOVERY_NEEDED){
                ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&r)==DBT_OK && r.state==DBT_SOUND_RECOVERY_CLEAN);
            }else if(r.state==DBT_SOUND_RECOVERY_BLOCKED){
                memcpy(saved,files,sizeof(files));ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&r)==DBT_INVALID);
                ASSERT(same_persisted(saved,files));
            }
            int final=find("/spiffs/startup.wav"),bak=find("/spiffs/.dbt_snd0.bak");
            if(r.state==DBT_SOUND_RECOVERY_CLEAN && replacing){
                ASSERT(final>=0 && files[final].len==48 && (!memcmp(files[final].data,old,48) || !memcmp(files[final].data,new_data,48)));
            }
            if(r.state==DBT_SOUND_RECOVERY_BLOCKED && replacing){
                ASSERT((final>=0 && files[final].len==48 && !memcmp(files[final].data,old,48)) ||
                       (bak>=0 && files[bak].len==48 && !memcmp(files[bak].data,old,48)));
            }
            if(completed)break;
        }
        ASSERT(completed && cut>10);
        printf("modeled %s crash boundaries: %d\n",replacing?"replacement":"creation",cut-1);
    }
    puts("creation and replacement crash cut points passed");return 0;
}
int main(void){
    dbt_sound_esp32_t ctx;dbt_sound_store_t s;bool muted=true;
    ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK && !muted);
    uint8_t old[48],new_data[48],out[48];wav(old,3);wav(new_data,7);uint32_t n;dbt_sound_recovery_t recovery;
    ASSERT(s.begin(s.context,0,48) && s.write(s.context,0,old,48) && s.seal(s.context));
    ASSERT(s.read_temp(s.context,0,out,48) && !memcmp(out,old,48) && s.commit(s.context,0));
    ASSERT(s.size(s.context,0,&n)==DBT_OK && n==48 && s.read(s.context,0,0,out,48) && !memcmp(out,old,48));
    for(unsigned stage=0;stage<4;stage++){
        const uint8_t *next=(stage&1)?old:new_data;
        ASSERT(s.begin(s.context,0,48) && s.write(s.context,0,next,48) && s.seal(s.context));
        if(stage==0)fail_old_rename=1;
        if(stage==1)fail_new_rename=1;
        if(stage==2)fail_backup_unlink=1;
        if(stage==3)fail_intent_unlink=1;
        ASSERT(!s.commit(s.context,0));s.abort(s.context);
        fail_old_rename=fail_new_rename=fail_backup_unlink=fail_intent_unlink=0;
        ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK);
        ASSERT(dbt_sound_esp32_inspect(&ctx,0,&recovery)==DBT_OK && recovery.state==DBT_SOUND_RECOVERY_NEEDED);
        if(stage==0)fail_old_rename=1;
        if(stage==1)fail_new_rename=1;
        if(stage==2)fail_backup_unlink=1;
        if(stage==3)fail_intent_unlink=1;
        ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&recovery)==DBT_INVALID);
        fail_old_rename=fail_new_rename=fail_backup_unlink=fail_intent_unlink=0;
        ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&recovery)==DBT_OK && recovery.state==DBT_SOUND_RECOVERY_CLEAN);
        ASSERT(s.read(s.context,0,0,out,48) && !memcmp(out,next,48));
    }
    ASSERT(s.begin(s.context,1,48) && s.write(s.context,0,old,48));fail_fsync=1;ASSERT(!s.seal(s.context));s.abort(s.context);fail_fsync=0;
    ASSERT(find("/spiffs/.dbt_snd1.tmp")<0 && find("/spiffs/pairing.wav")<0);
    ASSERT(s.begin(s.context,1,48) && s.write(s.context,0,old,48) && s.seal(s.context));
    fail_intent_write=1;ASSERT(!s.commit(s.context,1));s.abort(s.context);fail_intent_write=0;
    ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK);
    ASSERT(dbt_sound_esp32_inspect(&ctx,1,&recovery)==DBT_OK && recovery.state==DBT_SOUND_RECOVERY_BLOCKED);
    ASSERT(dbt_sound_esp32_reconcile(&ctx,1,&recovery)==DBT_INVALID && find("/spiffs/.dbt_snd1.tmp")>=0);
    /* A valid intent does not authorize mutation of a CRC-mismatched temp. */
    ASSERT(s.begin(s.context,0,48) && s.write(s.context,0,new_data,48) && s.seal(s.context));
    fail_old_rename=1;ASSERT(!s.commit(s.context,0));s.abort(s.context);fail_old_rename=0;
    int candidate=find("/spiffs/.dbt_snd0.tmp");ASSERT(candidate>=0);files[candidate].data[44]^=1;
    ASSERT(dbt_sound_esp32_inspect(&ctx,0,&recovery)==DBT_OK && recovery.state==DBT_SOUND_RECOVERY_BLOCKED);
    ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&recovery)==DBT_INVALID && find("/spiffs/startup.wav")>=0);
    files[candidate].data[44]^=1;ASSERT(dbt_sound_esp32_reconcile(&ctx,0,&recovery)==DBT_OK);
    ASSERT(s.set_mute(s.context,true));ASSERT(dbt_sound_esp32_init(&ctx,0,peer_check,&s,&muted)==DBT_OK && muted);
    fail_mute_commit=1;ASSERT(!s.set_mute(s.context,false));fail_mute_commit=0;
    ASSERT(s.remove(s.context,0) && s.size(s.context,0,&n)==DBT_EMPTY);
    ASSERT(crash_cuts()==0);
    puts("ESP32 SPIFFS/NVS provider passed in-memory shim tests");return 0;
}
