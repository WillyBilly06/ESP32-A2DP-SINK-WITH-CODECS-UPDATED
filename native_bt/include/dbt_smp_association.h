#ifndef DBT_SMP_ASSOCIATION_H
#define DBT_SMP_ASSOCIATION_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { DBT_SMP_ASSOC_INVALID,DBT_SMP_ASSOC_JUST_WORKS,DBT_SMP_ASSOC_PASSKEY,
               DBT_SMP_ASSOC_NUMERIC,DBT_SMP_ASSOC_OOB } dbt_smp_association_t;
/* Core Vol 3 H 2.3.5/Table 2.8, after the caller validates OOB availability.
 * The wire MITM flags select an association method; they do not create IO.
 * Local authentication requirements and permissions are enforced separately. */
static inline dbt_smp_association_t dbt_smp_select_association(bool sc,bool mitm,bool oob,uint8_t initiator_io,uint8_t responder_io){
    if(initiator_io>4 || responder_io>4)return DBT_SMP_ASSOC_INVALID;
    if(oob)return DBT_SMP_ASSOC_OOB;
    if(!mitm || initiator_io==3 || responder_io==3)return DBT_SMP_ASSOC_JUST_WORKS;
    if(sc && (initiator_io==1 || initiator_io==4) && (responder_io==1 || responder_io==4))return DBT_SMP_ASSOC_NUMERIC;
    bool ik=initiator_io==2 || initiator_io==4,rk=responder_io==2 || responder_io==4;
    bool id=initiator_io==0 || initiator_io==1 || initiator_io==4;
    bool rd=responder_io==0 || responder_io==1 || responder_io==4;
    return ((ik && rd)||(rk && id)||(initiator_io==2 && responder_io==2))?DBT_SMP_ASSOC_PASSKEY:DBT_SMP_ASSOC_JUST_WORKS;
}
#endif
