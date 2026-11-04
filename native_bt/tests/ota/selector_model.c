/* Power-cut model of SDK otadata selection, not a flash or bootloader test. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
typedef struct {uint32_t sequence;bool crc_valid;} entry_t;
typedef struct {entry_t records[2];uint8_t running[16],inactive[16],recovery[16],data[16];} store_t;
static int chosen(const store_t *s) {
    int chosen_index=-1;
    for(int i=0;i<2;i++) if(s->records[i].crc_valid && s->records[i].sequence!=UINT32_MAX &&
       (chosen_index<0 || s->records[i].sequence>s->records[chosen_index].sequence))chosen_index=i;
    return chosen_index;
}
static int slot(const store_t *s) {int i=chosen(s);return i<0?-1:(int)((s->records[i].sequence-1u)%3u);}
int main(void) {
    store_t s={0};s.records[0]=(entry_t){4,true};s.records[1]=(entry_t){2,true};
    memset(s.running,0x11,sizeof(s.running));memset(s.inactive,0x22,sizeof(s.inactive));
    memset(s.recovery,0x33,sizeof(s.recovery));memset(s.data,0x44,sizeof(s.data));
    store_t original=s;assert(slot(&s)==0 && s.records[1].sequence==2); /* ota_0 selected, ota_1 historical */
    /* Resume(0) changes no selectors. An interrupted inactive erase/write
     * leaves the selected record and the running, recovery and data bytes. */
    for(unsigned cut=0;cut<=sizeof(s.inactive);cut++) {
        store_t image=original;memset(image.inactive,0xff,cut);
        assert(slot(&image)==0 && !memcmp(image.records,original.records,sizeof(image.records)));
        assert(!memcmp(image.running,original.running,sizeof(image.running)));
        assert(!memcmp(image.recovery,original.recovery,sizeof(image.recovery)) && !memcmp(image.data,original.data,sizeof(image.data)));
    }
    /* SDK chooses the next sequence mapping to OTA1 with three OTA slots:
     * 4 is current; 5 maps to OTA1. It erases and rewrites the other record. */
    store_t before=s;int active=chosen(&s),other=1-active;
    s.records[other]=(entry_t){UINT32_MAX,false};assert(slot(&s)==0);
    s.records[other]=(entry_t){5,false};assert(slot(&s)==0); /* torn/CRC-invalid write */
    s.records[other]=(entry_t){5,true};assert(slot(&s)==1);
    assert(!memcmp(&s.records[active],&before.records[active],sizeof(entry_t)));
    assert(!memcmp(s.running,before.running,sizeof(s.running)) && !memcmp(s.recovery,before.recovery,sizeof(s.recovery)) &&
           !memcmp(s.data,before.data,sizeof(s.data)));
    return 0;
}
