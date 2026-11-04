#define DBT_PROBE_PRODUCT_SECURITY 1
#define DBT_SECURITY_REQUEST_TEST 1
#include <stdbool.h>
#include <stdint.h>
bool test_admission_allowed;
bool test_frozen;
uint32_t test_prompt_window;
#include "../smp/probe_adapter.c"
bool probe_admission_le_begin(const dbt_link_t *link,uint32_t now){
    (void)now;return test_admission_allowed && link && link->used && link->generation;
}
bool probe_admission_frozen(void){return test_frozen;}
uint32_t probe_admission_prompt_epoch(uint32_t now){(void)now;return test_prompt_window;}
void probe_admission_revalidate(void){}
