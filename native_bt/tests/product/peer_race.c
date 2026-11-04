#include "dbt_product_peer.h"
#include <assert.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#endif
static dbt_product_peer_t shared;
static _Atomic bool done;
static _Atomic unsigned reads;
#ifdef _WIN32
static DWORD WINAPI reader(void *unused)
#else
static void *reader(void *unused)
#endif
{
    (void)unused;
    do {
        dbt_product_peer_value_t v;
        if(dbt_product_peer_read(&shared,&v)) {
            uint8_t marker=v.peer.generation==1?0x11:0x22;
            assert((v.peer.generation==1 || v.peer.generation==2) && v.peer.handle==v.peer.generation &&
                   v.encrypted && v.bonded && v.allow_ota && v.ota.trusted_digest &&
                   v.ota.length==(v.peer.generation==1?111u:222u));
            for(unsigned i=0;i<32;i++)assert(v.ota.sha256[i]==marker && (uint8_t)v.ota.project[i]==marker);
            atomic_fetch_add_explicit(&reads,1,memory_order_relaxed);
        }
    } while(!atomic_load_explicit(&done,memory_order_acquire));
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}
int main(void) {
    dbt_product_peer_init(&shared);atomic_init(&done,false);atomic_init(&reads,0);
#ifdef _WIN32
    HANDLE t=CreateThread(NULL,0,reader,NULL,0,NULL);assert(t);
#else
    pthread_t t;assert(!pthread_create(&t,NULL,reader,NULL));
#endif
    for(unsigned i=0;i<30000;i++) {
        dbt_product_peer_value_t v={0};uint8_t marker=(i&1)?0x11:0x22;v.peer.generation=(i&1)?1u:2u;v.peer.handle=(uint16_t)v.peer.generation;
        v.encrypted=v.bonded=v.allow_ota=v.ota.trusted_digest=true;v.ota.length=(i&1)?111u:222u;
        memset(v.ota.sha256,marker,32);memset(v.ota.project,marker,32);assert(dbt_product_peer_publish(&shared,&v));
    }
    atomic_store_explicit(&done,true,memory_order_release);
#ifdef _WIN32
    assert(WaitForSingleObject(t,INFINITE)==WAIT_OBJECT_0);CloseHandle(t);
#else
    assert(!pthread_join(t,NULL));
#endif
    assert(atomic_load_explicit(&reads,memory_order_relaxed)>0);return 0;
}
