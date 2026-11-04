#ifndef DBT_OTA_H
#define DBT_OTA_H
#include "dbt_ble_control.h"
#include <stdatomic.h>

/* Authorization is a local grant; a trusted digest/CRC are optional. The
 * baseline OTA wire format itself has no authenticated digest or CRC. */
typedef struct {
    uint32_t length,crc32;
    uint8_t sha256[32];
    char project[32];
    bool trusted_digest;
} dbt_ota_manifest_t;
typedef struct {
    void *context;
    /* Worker only: grant plus current encrypted/bonded peer policy. A repeated
     * call must return the same manifest while the grant remains valid. */
    bool (*authorize)(void *,const dbt_ble_control_peer_t *,uint32_t,dbt_ota_manifest_t *);
    uint32_t (*now_ms)(void *);
    bool (*begin)(void *,const dbt_ota_manifest_t *);
    bool (*write)(void *,uint32_t,const uint8_t *,uint16_t);
    /* Verify readback SHA-256, image format/checksum, chip/revision and project. */
    bool (*finish)(void *,const dbt_ota_manifest_t *);
    bool (*abort)(void *);
    /* Explicit owner action only. This must not reboot the device. */
    bool (*activate)(void *,const dbt_ota_manifest_t *);
} dbt_ota_hooks_t;
typedef enum { DBT_OTA_IDLE,DBT_OTA_RECEIVING,DBT_OTA_VERIFIED,DBT_OTA_FAULT } dbt_ota_state_t;
typedef struct {
    dbt_ota_hooks_t hooks;dbt_ota_manifest_t manifest;dbt_ble_control_peer_t peer;
    uint32_t offset,crc,last_ms,timeout_ms;uint8_t sequence;bool sequence_started;dbt_ota_state_t state;
    _Atomic bool cancel;
} dbt_ota_t;
/* init once before publication; object lives until all owner/worker references end. */
bool dbt_ota_init(dbt_ota_t *,const dbt_ota_hooks_t *,uint32_t timeout_ms);
/* Worker only; result uses baseline OTA_READY/PROGRESS/COMPLETE/FAILED or ACK. */
bool dbt_ota_command(dbt_ota_t *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
/* Direct dbt_ble_control_hooks.command adapter when context is dbt_ota_t. */
bool dbt_ota_control_command(void *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
/* Owner can request cancellation without blocking; worker must call tick. */
void dbt_ota_request_cancel(dbt_ota_t *);
void dbt_ota_tick(dbt_ota_t *);
/* Explicit owner approval delivered to worker after verification. */
bool dbt_ota_activate(dbt_ota_t *,const dbt_ble_control_peer_t *);
#endif
