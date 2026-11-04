#include "dbt_product_peer.h"
#include <string.h>
void dbt_product_peer_init(dbt_product_peer_t *s) {
    atomic_init(&s->sequence,0);atomic_init(&s->handle,0);atomic_init(&s->generation,0);
    atomic_init(&s->flags,0);atomic_init(&s->length,0);atomic_init(&s->crc,0);
    for(unsigned i=0;i<8;i++){atomic_init(&s->sha[i],0);atomic_init(&s->project[i],0);}
}
bool dbt_product_peer_publish(dbt_product_peer_t *s,const dbt_product_peer_value_t *v) {
    if(!s || !v || (v->peer.generation && v->peer.handle>0xeffu) ||
       (v->ota.trusted_digest && (!v->ota.length || !v->allow_ota)))return false;
    uint32_t seq=atomic_load_explicit(&s->sequence,memory_order_seq_cst);
    if((seq&1u) || seq>UINT32_MAX-2u) {atomic_store_explicit(&s->sequence,UINT32_MAX,memory_order_seq_cst);return false;}
    atomic_store_explicit(&s->sequence,seq+1u,memory_order_seq_cst);
    uint32_t flags=(v->encrypted?1u:0u)|(v->bonded?2u:0u)|(v->allow_name?4u:0u)|
                   (v->allow_ota?8u:0u)|(v->ota.trusted_digest?16u:0u);
    atomic_store_explicit(&s->handle,v->peer.handle,memory_order_seq_cst);
    atomic_store_explicit(&s->generation,v->peer.generation,memory_order_seq_cst);
    atomic_store_explicit(&s->flags,flags,memory_order_seq_cst);
    atomic_store_explicit(&s->length,v->ota.length,memory_order_seq_cst);
    atomic_store_explicit(&s->crc,v->ota.crc32,memory_order_seq_cst);
    for(unsigned i=0;i<8;i++) {uint32_t a,b;memcpy(&a,v->ota.sha256+4*i,4);memcpy(&b,v->ota.project+4*i,4);
        atomic_store_explicit(&s->sha[i],a,memory_order_seq_cst);atomic_store_explicit(&s->project[i],b,memory_order_seq_cst);}
    atomic_store_explicit(&s->sequence,seq+2u,memory_order_seq_cst);return true;
}
bool dbt_product_peer_read(const dbt_product_peer_t *s,dbt_product_peer_value_t *v) {
    if(!s || !v)return false;
    for(unsigned attempt=0;attempt<3;attempt++) {
        uint32_t seq=atomic_load_explicit(&s->sequence,memory_order_seq_cst);if(seq&1u)continue;
        dbt_product_peer_value_t n={0};
        n.peer.handle=(uint16_t)atomic_load_explicit(&s->handle,memory_order_seq_cst);
        n.peer.generation=atomic_load_explicit(&s->generation,memory_order_seq_cst);
        uint32_t flags=atomic_load_explicit(&s->flags,memory_order_seq_cst);
        n.encrypted=(flags&1u)!=0;n.bonded=(flags&2u)!=0;n.allow_name=(flags&4u)!=0;n.allow_ota=(flags&8u)!=0;
        n.ota.trusted_digest=(flags&16u)!=0;n.ota.length=atomic_load_explicit(&s->length,memory_order_seq_cst);
        n.ota.crc32=atomic_load_explicit(&s->crc,memory_order_seq_cst);
        for(unsigned i=0;i<8;i++){uint32_t a=atomic_load_explicit(&s->sha[i],memory_order_seq_cst),b=atomic_load_explicit(&s->project[i],memory_order_seq_cst);
            memcpy(n.ota.sha256+4*i,&a,4);memcpy(n.ota.project+4*i,&b,4);}
        if(seq==atomic_load_explicit(&s->sequence,memory_order_seq_cst)){*v=n;return n.peer.generation!=0;}
    }
    return false;
}
