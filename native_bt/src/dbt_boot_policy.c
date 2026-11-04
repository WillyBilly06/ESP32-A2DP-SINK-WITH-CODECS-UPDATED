#include "dbt_boot_policy.h"
static bool equal(const char *value,const char *expected,unsigned limit){
    if(!value)return false;
    for(unsigned i=0;i<limit;++i){if(value[i]!=expected[i])return false;if(!expected[i])return true;}
    return false;
}
bool dbt_boot_owned_slot(uint8_t type,uint8_t subtype,const char *label,const char *project){
    if(type!=0 || !equal(project,"native_bt_probe",32))return false;
    return (subtype==16 && equal(label,"ota_0",16)) || (subtype==17 && equal(label,"ota_1",16));
}
