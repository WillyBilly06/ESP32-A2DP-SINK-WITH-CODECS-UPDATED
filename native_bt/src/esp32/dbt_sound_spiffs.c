#include "dbt_sound_esp32.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "nvs.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>

static const char *const paths[4]={"/spiffs/startup.wav","/spiffs/pairing.wav","/spiffs/connected.wav","/spiffs/volumemax.wav"};
static const char *const temp[4]={"/spiffs/.dbt_snd0.tmp","/spiffs/.dbt_snd1.tmp","/spiffs/.dbt_snd2.tmp","/spiffs/.dbt_snd3.tmp"};
static const char *const backup[4]={"/spiffs/.dbt_snd0.bak","/spiffs/.dbt_snd1.bak","/spiffs/.dbt_snd2.bak","/spiffs/.dbt_snd3.bak"};
static const char *const intent[4]={"/spiffs/.dbt_snd0.txn","/spiffs/.dbt_snd1.txn","/spiffs/.dbt_snd2.txn","/spiffs/.dbt_snd3.txn"};
static int present(const char *path,struct stat *st){if(stat(path,st)==0)return 1;return errno==ENOENT?0:-1;}
static uint32_t crc_add(uint32_t c,const uint8_t *p,unsigned n){while(n--){c^=*p++;for(unsigned i=0;i<8;i++)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));}return c;}
static uint32_t get32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void put32(uint8_t *p,uint32_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
static bool parse_read(void *v,uint8_t type,uint32_t at,uint8_t *b,uint16_t n){
    (void)type;FILE *f=v;return fseek(f,(long)at,SEEK_SET)==0 && fread(b,1,n,f)==n;
}
static bool verify(const char *path,uint32_t *size,uint32_t *crc){
    struct stat st;if(present(path,&st)!=1 || st.st_size<44 || (uint64_t)st.st_size>0xffffffu)return false;
    FILE *f=fopen(path,"rb");if(!f)return false;
    uint8_t bytes[256];uint32_t length=(uint32_t)st.st_size,at=0,c=0xffffffffu;bool ok=true;
    while(at<length){size_t n=length-at>sizeof(bytes)?sizeof(bytes):length-at;
        if(fread(bytes,1,n,f)!=n){ok=false;break;}c=crc_add(c,bytes,(unsigned)n);at+=(uint32_t)n;}
    dbt_sound_t sound;memset(&sound,0,sizeof(sound));sound.store.context=f;sound.store.read=parse_read;
    dbt_sound_wav_t wav;
    if(ok && dbt_sound_wav(&sound,0,false,length,&wav)!=DBT_OK)ok=false;
    if(fclose(f)!=0)ok=false;
    if(ok){*size=length;*crc=~c;}return ok;
}
typedef struct {bool old;uint32_t old_size,old_crc,new_size,new_crc;} transaction;
static bool read_intent(uint8_t type,transaction *t){
    struct stat st;if(present(intent[type],&st)!=1 || st.st_size!=28)return false;
    FILE *f=fopen(intent[type],"rb");if(!f)return false;uint8_t b[28];bool ok=fread(b,1,sizeof(b),f)==sizeof(b);
    if(fclose(f)!=0)ok=false;
    if(!ok || memcmp(b,"DBTS",4) || b[4]!=1 || b[5]!=type || b[6]>1 || b[7] || get32(b+24)!=~crc_add(0xffffffffu,b,24))return false;
    t->old=b[6]!=0;t->old_size=get32(b+8);t->old_crc=get32(b+12);t->new_size=get32(b+16);t->new_crc=get32(b+20);
    return t->new_size>=44 && t->new_size<=0xffffffu &&
        (t->old?(t->old_size>=44 && t->old_size<=0xffffffu):(!t->old_size && !t->old_crc));
}
static bool write_intent(dbt_sound_esp32_t *s,uint8_t type,const transaction *t){
    uint8_t b[28]={0};memcpy(b,"DBTS",4);b[4]=1;b[5]=type;b[6]=t->old?1:0;
    put32(b+8,t->old_size);put32(b+12,t->old_crc);put32(b+16,t->new_size);put32(b+20,t->new_crc);put32(b+24,~crc_add(0xffffffffu,b,24));
    int fd=open(intent[type],O_CREAT|O_EXCL|O_RDWR,0600);if(fd<0)return false;
    s->journaled=true;FILE *f=fdopen(fd,"w+b");if(!f){close(fd);return false;}
    bool ok=fwrite(b,1,sizeof(b),f)==sizeof(b) && fflush(f)==0 && fsync(fileno(f))==0;
    if(fclose(f)!=0)ok=false;
    return ok;
}
static bool begin(void *v,uint8_t type,uint32_t n){
    dbt_sound_esp32_t *s=v;if(type>3 || s->temp || s->created)return false;
    struct stat st;size_t total=0,used=0;
    if(present(backup[type],&st)!=0 || present(intent[type],&st)!=0 || esp_spiffs_info("spiffs",&total,&used)!=ESP_OK || used>total || n>total-used)return false;
    int fd=open(temp[type],O_CREAT|O_EXCL|O_RDWR,0600);if(fd<0)return false;
    s->type=type;s->created=true;s->temp=fdopen(fd,"w+b");
    if(!s->temp){close(fd);unlink(temp[type]);s->created=false;return false;}return true;
}
static bool write_temp(void *v,uint32_t at,const uint8_t *b,uint16_t n){
    dbt_sound_esp32_t *s=v;return s->temp && fseek(s->temp,(long)at,SEEK_SET)==0 && fwrite(b,1,n,s->temp)==n;
}
static bool seal(void *v){dbt_sound_esp32_t *s=v;return s->temp && fflush(s->temp)==0 && fsync(fileno(s->temp))==0;}
static bool read_temp(void *v,uint32_t at,uint8_t *b,uint16_t n){
    dbt_sound_esp32_t *s=v;return s->temp && fseek(s->temp,(long)at,SEEK_SET)==0 && fread(b,1,n,s->temp)==n;
}
static void abort_temp(void *v){
    dbt_sound_esp32_t *s=v;if(s->temp){fclose(s->temp);s->temp=0;}if(s->created){if(!s->journaled)unlink(temp[s->type]);s->created=false;}
}
static bool commit(void *v,uint8_t type){
    dbt_sound_esp32_t *s=v;if(type>3 || !s->created || !s->temp || s->type!=type)return false;
    if(fclose(s->temp)!=0){s->temp=0;return false;}s->temp=0;
    struct stat st;int old=present(paths[type],&st);
    if(old<0 || present(backup[type],&st)!=0 || present(intent[type],&st)!=0)return false;
    transaction t={0};t.old=old!=0;
    if(!verify(temp[type],&t.new_size,&t.new_crc) || (t.old && !verify(paths[type],&t.old_size,&t.old_crc)))return false;
    if(!write_intent(s,type,&t))return false;
    if(t.old && rename(paths[type],backup[type])!=0)return false;
    if(rename(temp[type],paths[type])!=0)return false;
    s->created=false;
    uint32_t length=0,crc=0;
    if(!verify(paths[type],&length,&crc) || length!=t.new_size || crc!=t.new_crc)return false;
    if(t.old && (!verify(backup[type],&length,&crc) || length!=t.old_size || crc!=t.old_crc || unlink(backup[type])!=0))return false;
    if(unlink(intent[type])!=0)return false;
    s->journaled=false;return true;
}
static dbt_result_t size_of(void *v,uint8_t type,uint32_t *n){
    (void)v;if(type>3 || !n)return DBT_INVALID;struct stat st;
    if(present(backup[type],&st)!=0 || present(intent[type],&st)!=0)return DBT_INVALID;
    int found=present(paths[type],&st);if(found<0 || (found && (st.st_size<0 || (uint64_t)st.st_size>UINT32_MAX)))return DBT_INVALID;
    if(!found)return DBT_EMPTY;
    *n=(uint32_t)st.st_size;return DBT_OK;
}
static bool read_file(void *v,uint8_t type,uint32_t at,uint8_t *b,uint16_t n){
    (void)v;if(type>3)return false;FILE *f=fopen(paths[type],"rb");if(!f)return false;
    bool ok=fseek(f,(long)at,SEEK_SET)==0 && fread(b,1,n,f)==n;return fclose(f)==0 && ok;
}
static bool remove_file(void *v,uint8_t type){(void)v;struct stat st;return type<4 && present(backup[type],&st)==0 && present(intent[type],&st)==0 && unlink(paths[type])==0;}
static bool mute(void *v,bool muted){
    (void)v;nvs_handle_t h;if(nvs_open("audio",NVS_READWRITE,&h)!=ESP_OK)return false;
    esp_err_t r=nvs_set_u8(h,"sound_muted",muted?1:0);if(r==ESP_OK)r=nvs_commit(h);nvs_close(h);return r==ESP_OK;
}
static uint32_t now_ms(void *v){(void)v;return (uint32_t)((uint64_t)esp_timer_get_time()/1000u);}
static bool valid(void *v,dbt_sound_peer_t peer){dbt_sound_esp32_t *s=v;return s->peer_valid(s->peer_context,peer);}
static bool matches(const char *path,uint32_t want_size,uint32_t want_crc){uint32_t size=0,crc=0;return verify(path,&size,&crc) && size==want_size && crc==want_crc;}
dbt_result_t dbt_sound_esp32_inspect(dbt_sound_esp32_t *s,uint8_t type,dbt_sound_recovery_t *r){
    if(!s || !r || type>3)return DBT_INVALID;
    if(s->temp || s->created)return DBT_BUSY;
    memset(r,0,sizeof(*r));struct stat st;
    int fi=present(paths[type],&st),te=present(temp[type],&st),ba=present(backup[type],&st),in=present(intent[type],&st);
    if(fi<0 || te<0 || ba<0 || in<0){r->state=DBT_SOUND_RECOVERY_BLOCKED;return DBT_INVALID;}
    r->final_file=fi!=0;r->temp=te!=0;r->backup=ba!=0;r->intent=in!=0;
    if(!in){r->state=(te || ba)?DBT_SOUND_RECOVERY_BLOCKED:DBT_SOUND_RECOVERY_CLEAN;return DBT_OK;}
    transaction t;
    if(!read_intent(type,&t) || (te && !matches(temp[type],t.new_size,t.new_crc)) ||
       (ba && (!t.old || !matches(backup[type],t.old_size,t.old_crc)))){
        r->state=DBT_SOUND_RECOVERY_BLOCKED;return DBT_OK;
    }
    bool old=fi && t.old && matches(paths[type],t.old_size,t.old_crc);
    bool new_file=fi && matches(paths[type],t.new_size,t.new_crc);
    bool possible;
    if(fi && te)possible=t.old && old && !ba;
    else if(!fi && te)possible=t.old?ba:!ba;
    else if(fi && !te)possible=ba?new_file:(new_file || old);
    else possible=t.old?ba:!ba;
    r->state=possible?DBT_SOUND_RECOVERY_NEEDED:DBT_SOUND_RECOVERY_BLOCKED;return DBT_OK;
}
dbt_result_t dbt_sound_esp32_reconcile(dbt_sound_esp32_t *s,uint8_t type,dbt_sound_recovery_t *r){
    dbt_result_t result=dbt_sound_esp32_inspect(s,type,r);if(result!=DBT_OK)return result;
    if(r->state==DBT_SOUND_RECOVERY_CLEAN){s->journaled=false;return DBT_OK;}
    if(r->state!=DBT_SOUND_RECOVERY_NEEDED)return DBT_INVALID;
    transaction t;if(!read_intent(type,&t))return DBT_INVALID;
    if(r->final_file && r->temp){
        if(rename(paths[type],backup[type])!=0)return DBT_INVALID;
        r->final_file=false;r->backup=true;
    }
    if(!r->final_file && r->temp){
        if(rename(temp[type],paths[type])!=0)return DBT_INVALID;
        r->temp=false;r->final_file=true;
    }
    if(!r->final_file && r->backup){
        if(rename(backup[type],paths[type])!=0)return DBT_INVALID;
        r->backup=false;r->final_file=true;
    }
    if(r->backup){
        if(!matches(paths[type],t.new_size,t.new_crc) || !matches(backup[type],t.old_size,t.old_crc) || unlink(backup[type])!=0)return DBT_INVALID;
        r->backup=false;
    }
    if(unlink(intent[type])!=0)return DBT_INVALID;
    s->journaled=false;return dbt_sound_esp32_inspect(s,type,r);
}
dbt_result_t dbt_sound_esp32_init(dbt_sound_esp32_t *s,void *context,bool (*peer_valid)(void *,dbt_sound_peer_t),dbt_sound_store_t *store,bool *muted){
    if(!s || !peer_valid || !store || !muted)return DBT_INVALID;
    memset(s,0,sizeof(*s));s->peer_context=context;s->peer_valid=peer_valid;
    nvs_handle_t h;esp_err_t r=nvs_open("audio",NVS_READONLY,&h);uint8_t value=0;
    if(r==ESP_ERR_NVS_NOT_FOUND)*muted=false;
    else if(r!=ESP_OK)return DBT_INVALID;
    else {r=nvs_get_u8(h,"sound_muted",&value);nvs_close(h);if(r!=ESP_OK && r!=ESP_ERR_NVS_NOT_FOUND)return DBT_INVALID;*muted=value!=0;}
    *store=(dbt_sound_store_t){s,begin,write_temp,seal,read_temp,commit,abort_temp,size_of,read_file,remove_file,mute,now_ms,valid};return DBT_OK;
}
