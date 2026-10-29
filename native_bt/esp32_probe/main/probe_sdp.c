#include "probe_sdp.h"

static dbt_sdp_t server;
static dbt_sdp_bearer_t bearer;
static const uint8_t server_class[] = {0x35, 3, 0x19, 0x10, 0};
static const uint8_t versions[] = {0x35, 3, 9, 1, 0};
#ifdef DBT_PROBE_AUDIO_OUTPUT
static uint8_t state[] = {10, 0, 0, 0, 2};
#else
static uint8_t state[] = {10, 0, 0, 0, 1};
#endif
static const dbt_sdp_attribute_t server_attributes[] = {{1, sizeof(server_class), server_class},
                                                        {0x200, sizeof(versions), versions},
                                                        {0x201, sizeof(state), state}};
/* Private diagnostic class: e8319409-8ac4-4f14-9e87-e04799f11001.
 * No AudioSink/AVRCP class is published before those profiles are implemented. */
static const uint8_t echo_class[] = {0x35, 17,   0x1c, 0xe8, 0x31, 0x94, 9,    0x8a, 0xc4, 0x4f,
                                     0x14, 0x9e, 0x87, 0xe0, 0x47, 0x99, 0xf1, 0x10, 1};
static const uint8_t protocols[] = {0x35, 8, 0x35, 6, 0x19, 1, 0, 9, 0x10, 1};
static const uint8_t browse[] = {0x35, 3, 0x19, 0x10, 2};
static const uint8_t language[] = {0x35, 9, 9, 0x65, 0x6e, 9, 0, 0x6a, 9, 1, 0};
static const uint8_t name[] = {0x25, 20,  'E', 'n', 'c', 'r', 'y', 'p', 't', 'e', 'd',
                               ' ',  'B', 'a', 's', 'i', 'c', ' ', 'e', 'c', 'h', 'o'};
static const dbt_sdp_attribute_t echo_attributes[] = {{1, sizeof(echo_class), echo_class},
                                                      {4, sizeof(protocols), protocols},
                                                      {5, sizeof(browse), browse},
                                                      {6, sizeof(language), language},
                                                      {0x100, sizeof(name), name}};
#ifdef DBT_PROBE_AUDIO_OUTPUT
static const uint8_t sink_class[]={0x35,3,0x19,0x11,0x0b};
static const uint8_t sink_protocol[]={0x35,16,0x35,6,0x19,1,0,9,0,0x19,0x35,6,0x19,0,0x19,9,1,3};
static const uint8_t sink_profile[]={0x35,8,0x35,6,0x19,0x11,0x0d,9,1,3};
static const uint8_t sink_features[]={9,0,2};
static const uint8_t sink_name[]="\x25\x12" "Native SBC Speaker";
static const dbt_sdp_attribute_t sink_attributes[]={
    {1,sizeof(sink_class),sink_class},{4,sizeof(sink_protocol),sink_protocol},{5,sizeof(browse),browse},
    {6,sizeof(language),language},{9,sizeof(sink_profile),sink_profile},{0x100,sizeof(sink_name)-1,sink_name},
    {0x311,sizeof(sink_features),sink_features}};
#endif
/* AVRCP 1.4 controller and category-2 target, AVCTP 1.4. No browsing
 * protocol descriptor or browsing feature is published. Attribute sequences
 * follow the assigned numbers and AVRCP service record tables. */
static const uint8_t ct_class[]={0x35,6,0x19,0x11,0x0e,0x19,0x11,0x0f};
static const uint8_t tg_class[]={0x35,3,0x19,0x11,0x0c};
static const uint8_t avrcp_protocol[]={0x35,16,0x35,6,0x19,1,0,9,0,0x17,0x35,6,0x19,0,0x17,9,1,4};
static const uint8_t avrcp_profile[]={0x35,8,0x35,6,0x19,0x11,0x0e,9,1,4};
static const uint8_t ct_features[]={9,0,1};
static const uint8_t tg_features[]={9,0,2};
static const dbt_sdp_attribute_t ct_attributes[]={
    {1,sizeof(ct_class),ct_class},{4,sizeof(avrcp_protocol),avrcp_protocol},{5,sizeof(browse),browse},
    {9,sizeof(avrcp_profile),avrcp_profile},{0x311,sizeof(ct_features),ct_features}};
static const dbt_sdp_attribute_t tg_attributes[]={
    {1,sizeof(tg_class),tg_class},{4,sizeof(avrcp_protocol),avrcp_protocol},{5,sizeof(browse),browse},
    {9,sizeof(avrcp_profile),avrcp_profile},{0x311,sizeof(tg_features),tg_features}};
static dbt_sdp_record_t records[5];
static unsigned record_count;
static bool initialized, controller_enabled, target_enabled;

dbt_result_t probe_sdp_enable_avrcp(bool controller, bool target) {
    if (initialized || (target && !controller)) return DBT_INVALID;
    controller_enabled=controller; target_enabled=target; return DBT_OK;
}
dbt_result_t probe_sdp_init(dbt_runtime_t *runtime) {
    if (initialized) return DBT_INVALID;
    record_count=0;
    records[record_count++]=(dbt_sdp_record_t){0,server_attributes,sizeof(server_attributes)/sizeof(server_attributes[0])};
    records[record_count++]=(dbt_sdp_record_t){0x10000,echo_attributes,sizeof(echo_attributes)/sizeof(echo_attributes[0])};
#ifdef DBT_PROBE_AUDIO_OUTPUT
    records[record_count++]=(dbt_sdp_record_t){0x10001,sink_attributes,sizeof(sink_attributes)/sizeof(sink_attributes[0])};
#endif
    if (controller_enabled) records[record_count++]=(dbt_sdp_record_t){0x10002,ct_attributes,sizeof(ct_attributes)/sizeof(ct_attributes[0])};
    if (target_enabled) records[record_count++]=(dbt_sdp_record_t){0x10003,tg_attributes,sizeof(tg_attributes)/sizeof(tg_attributes[0])};
    uint32_t db_state=(probe_sdp_audio_sink()?2u:1u)+(controller_enabled?1u:0u)+(target_enabled?1u:0u);
    state[1]=(uint8_t)(db_state>>24); state[2]=(uint8_t)(db_state>>16); state[3]=(uint8_t)(db_state>>8); state[4]=(uint8_t)db_state;
    dbt_result_t result = dbt_sdp_init(&server, records, (uint8_t)record_count, db_state);
    if (result==DBT_OK) result=dbt_sdp_bearer_init(&bearer, runtime, &server, 672);
    initialized=result==DBT_OK; return result;
}
dbt_result_t probe_sdp_receive(const dbt_channel_t *channel, const uint8_t *data, uint16_t size) {
    return dbt_sdp_bearer_receive(&bearer, channel, data, size);
}
void probe_sdp_event(dbt_channel_event_t event, const dbt_channel_t *channel) {
    dbt_sdp_bearer_event(&bearer, event, channel);
}
dbt_result_t probe_sdp_poll(void) { return dbt_sdp_bearer_poll(&bearer); }
uint32_t probe_sdp_wait_ms(void) { return dbt_sdp_bearer_wait_ms(&bearer); }
unsigned probe_sdp_bytes(void) { return sizeof(server) + sizeof(bearer); }
unsigned probe_sdp_records(void){return record_count;}
bool probe_sdp_audio_sink(void){
#ifdef DBT_PROBE_AUDIO_OUTPUT
    return true;
#else
    return false;
#endif
}
