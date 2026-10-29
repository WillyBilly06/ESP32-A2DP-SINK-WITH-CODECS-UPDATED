#include "dbt_host.h"
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
static void yield(void) { Sleep(0); }
#else
#include <pthread.h>
#include <sched.h>
static void yield(void) { sched_yield(); }
#endif
#define COUNT 20000u
typedef struct { dbt_host_t host; _Atomic unsigned done,error; unsigned received; } fixture_t;
static fixture_t f;
static bool ready(void *p) { (void)p; return false; }
static bool transport_send(void *p,uint8_t *data,uint16_t n) { (void)p;(void)data;(void)n;return false; }
static void observe(void *p,const dbt_hci_packet_t *packet) {
    fixture_t *x=p;
    if(packet->type!=DBT_H4_ACL || packet->size!=32) { atomic_store(&x->error,1); return; }
    uint32_t sequence=(uint32_t)packet->payload[0] | ((uint32_t)packet->payload[1]<<8) |
                      ((uint32_t)packet->payload[2]<<16) | ((uint32_t)packet->payload[3]<<24);
    if(sequence!=x->received++) atomic_store(&x->error,2);
    for(unsigned i=4;i<32;i++) if(packet->payload[i]!=0xa5) atomic_store(&x->error,3);
}
#ifdef _WIN32
static DWORD WINAPI produce(LPVOID unused)
#else
static void *produce(void *unused)
#endif
{
    (void)unused; uint8_t packet[37]={2,1,0x20,32,0};
    for(uint32_t i=0;i<COUNT && !atomic_load(&f.error);i++) {
        while(dbt_queue_used(&f.host.ingress)>2048u && !atomic_load(&f.error)) yield();
        packet[5]=(uint8_t)i; packet[6]=(uint8_t)(i>>8); packet[7]=(uint8_t)(i>>16); packet[8]=(uint8_t)(i>>24);
        memset(packet+9,0xa5,28);
        if(dbt_host_receive(&f.host,packet,sizeof(packet))!=DBT_OK) { atomic_store(&f.error,4); break; }
    }
    atomic_store(&f.done,1);
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}
int main(void) {
    memset(&f,0,sizeof(f)); dbt_transport_t transport={NULL,ready,transport_send};
    if(dbt_host_init(&f.host,transport,observe,&f,2000)!=DBT_OK) return 1;
    f.host.state=DBT_HOST_IDENTIFIED; atomic_init(&f.done,0); atomic_init(&f.error,0);
#ifdef _WIN32
    HANDLE thread=CreateThread(NULL,0,produce,NULL,0,NULL); if(!thread) return 2;
#else
    pthread_t thread; if(pthread_create(&thread,NULL,produce,NULL)) return 2;
#endif
    while((!atomic_load(&f.done) || dbt_queue_used(&f.host.ingress)) && !atomic_load(&f.error)) {
        dbt_host_poll(&f.host,1,8);
        for(unsigned i=0;i<37;i++) if(f.host.rx_packet[i]) atomic_store(&f.error,5);
        yield();
    }
#ifdef _WIN32
    WaitForSingleObject(thread,INFINITE); CloseHandle(thread);
#else
    pthread_join(thread,NULL);
#endif
    if(atomic_load(&f.error) || f.received!=COUNT || dbt_queue_used(&f.host.ingress)) return 3;
    for(unsigned i=0;i<DBT_QUEUE_BYTES;i++) if(f.host.ingress.bytes[i]==0xa5) return 4;
    printf("ACL SPSC sensitive ingress: %u ordered packets, wrapped ring wiped\n",COUNT); return 0;
}
