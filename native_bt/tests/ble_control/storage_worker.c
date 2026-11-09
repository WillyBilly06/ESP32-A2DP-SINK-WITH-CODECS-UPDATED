/* Actual GATT persistence task with modeled reserve completions. The reserve
 * and actual PCM queue have separate source-bound sanitizer coverage. */
#include "freertos/task.h"
static void storage_worker_backend(void);
static void storage_worker_queue_checkpoint(QueueHandle_t);
static void storage_worker_cases(void);
#define DBT_PROBE_AUDIO_OUTPUT 1
#define DBT_PROBE_STORAGE_RESERVE 1
#define DBT_BLE_TEST_STORAGE_WORKER 1
#include "schema.c"
static bool gate_audit,gate_refuse,gate_ready,gate_running,gate_releasing,gate_retired;
static unsigned gate_calls,gate_waits,gate_backends,gate_before_commits;
static const void *gate_cookie;
dbt_result_t probe_output_storage_prepare(const void *cookie,uintptr_t waiter){
  assert(worker_context && cookie && waiter==1);
  if(!gate_audit)return DBT_OK;
  assert(!gate_cookie || cookie==gate_cookie);gate_cookie=cookie;++gate_calls;
  if(!gate_ready)return DBT_PENDING;
  if(gate_refuse)return DBT_NO_SPACE;
  assert(!gate_running && !gate_releasing);gate_running=true;return DBT_OK;
}
void probe_output_storage_cancel(const void *cookie){(void)cookie;assert(!gate_audit);}
void probe_output_storage_release(void){
  if(!gate_audit)return;
  assert(worker_context && gate_running && gate_backends==1);gate_running=false;gate_releasing=true;
}
dbt_result_t probe_output_storage_retire(void){
  if(!gate_audit)return DBT_OK;
  assert(worker_context && gate_releasing);return gate_retired?DBT_OK:DBT_PENDING;
}
uint32_t ulTaskNotifyTake(BaseType_t clear,TickType_t timeout){
  assert(gate_audit && worker_context && running_worker && clear && timeout==portMAX_DELAY);
  assert(!uxQueueMessagesWaiting(control_done));++gate_waits;
  if(!gate_ready){assert(!gate_backends && commits==gate_before_commits);gate_ready=true;}
  else {assert(gate_releasing && !gate_running && gate_backends==1);gate_retired=true;}
  return 1;
}
static void storage_worker_backend(void){
  if(!gate_audit)return;
  assert(worker_context && gate_running && gate_ready && !gate_releasing && !gate_refuse);++gate_backends;
}
static void storage_worker_queue_checkpoint(QueueHandle_t q){
  if(gate_audit && running_worker && q==eq_work && !uxQueueMessagesWaiting(q) && uxQueueMessagesWaiting(control_done)){
    assert(gate_refuse?(!gate_backends && gate_waits==1):(gate_backends==1 && gate_retired && gate_waits==2));
    longjmp(worker_exit,1); /* Only after job completion and all reserve cleanup. */
  }
}
static void storage_case_reset(bool refuse){
  assert(!control_jobs && !cccd_jobs);
  control_job_t *marker=0;while(xQueueReceive(eq_work,&marker,0)==pdTRUE)assert(!marker);
  while(xQueueReceive(control_done,&marker,0)==pdTRUE)assert((uintptr_t)marker<=3u);
  eq_worker=true;eq_retiring=false;eq_retry_pending=false;
  gate_audit=true;gate_refuse=refuse;gate_ready=gate_running=gate_releasing=gate_retired=false;
  gate_calls=gate_waits=gate_backends=0;gate_cookie=0;gate_before_commits=commits;
}
static void storage_run_task(void){
  fake_worker_core=DBT_ESP32_PERSISTENCE_WORKER_CORE;worker_context=running_worker=true;
  if(!setjmp(worker_exit))control_task((void *)(uintptr_t)1);
  worker_context=running_worker=false;assert(gate_calls==2 && uxQueueMessagesWaiting(control_done)==1);
}
static void storage_worker_cases(void){
  /* An EQ backend and its completion are both withheld at their proper gates. */
  storage_case_reset(false);unsigned before=commits;
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK);storage_run_task();assert(commits==before+1);
  gate_audit=false;pump_control();assert(!control_jobs);
  /* Temporary reserve refusal does not invoke NVS or publish a false durable EQ. */
  storage_case_reset(true);before=commits;uint32_t old=durable_eq;
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK);storage_run_task();assert(commits==before && durable_eq==old);
  gate_audit=false;pump_control();assert(!control_jobs);
  /* A bonded CCCD refusal produces ATT Resources, keeps old subscriptions and
   * permits a later successful retry without poisoning the bond incarnation. */
  storage_case_reset(false);gate_audit=false;dbt_att_clear(&server);link0->generation++;link0->encrypted=true;
  committed_bond=true;test_bond_tag=3;memcpy(cccd_saved[3],(uint16_t[]){2,1,0,0},sizeof cccd_saved[3]);cccd_present[3]=true;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();
  gate_audit=true;storage_run_task();gate_audit=false;pump_control();sync_cccd();
  assert(cccd_binding(link0)->phase==CCCD_BOUND && !server.connections[0].cccds[2] && !control_jobs);
  const uint8_t enabled[]={1,0};uint8_t out[256];unsigned n;
  storage_case_reset(true);write_req(0x25,enabled,2,false);storage_run_task();assert(!cccd_saved[3][2]);
  gate_audit=false;pump_control();tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_RESOURCES && !server.connections[0].cccds[2] && !cccd_binding(link0)->values[2] && cccd_binding(link0)->phase==CCCD_BOUND);
  storage_case_reset(false);write_req(0x25,enabled,2,false);storage_run_task();assert(cccd_saved[3][2]==1);
  gate_audit=false;pump_control();tx(out,&n);assert(n==1 && out[0]==0x13 && server.connections[0].cccds[2]==1);
  puts("Actual persistence task: backend only after grant, completion only after retirement, EQ/CCCD resource refusal and bonded retry passed");
}
