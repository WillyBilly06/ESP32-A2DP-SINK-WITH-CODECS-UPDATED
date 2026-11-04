#ifndef DBT_SOUND_ESP32_H
#define DBT_SOUND_ESP32_H
#include "dbt_sound.h"
#include <stdio.h>
typedef struct {
    FILE *temp;uint8_t type;bool created,journaled;
    void *peer_context;bool (*peer_valid)(void *,dbt_sound_peer_t);
} dbt_sound_esp32_t;
/* Uses an already mounted `spiffs` partition and the baseline audio NVS
 * namespace. Never mounts, formats, initializes, or erases either store. */
dbt_result_t dbt_sound_esp32_init(dbt_sound_esp32_t *,void *,bool (*)(void *,dbt_sound_peer_t),dbt_sound_store_t *,bool *);
typedef enum { DBT_SOUND_RECOVERY_CLEAN,DBT_SOUND_RECOVERY_NEEDED,DBT_SOUND_RECOVERY_BLOCKED } dbt_sound_recovery_state_t;
typedef struct {dbt_sound_recovery_state_t state;bool intent,final_file,temp,backup;} dbt_sound_recovery_t;
/* Worker-only; the slot must be quiescent. Inspect never mutates storage.
 * Reconcile only changes files matching a complete, validated v1 intent. */
dbt_result_t dbt_sound_esp32_inspect(dbt_sound_esp32_t *,uint8_t,dbt_sound_recovery_t *);
dbt_result_t dbt_sound_esp32_reconcile(dbt_sound_esp32_t *,uint8_t,dbt_sound_recovery_t *);
#endif
