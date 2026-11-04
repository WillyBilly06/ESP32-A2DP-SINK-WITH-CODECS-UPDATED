/* Compile-only size markers; not linked to firmware. */
#include "dbt_smp_bearer.h"
#include "dbt_smp_features.h"
#include "dbt_smp_nvs.h"
#include "dbt_smp_worker_mbedtls.h"
#include "dbt_smp_session.h"
unsigned char dbt_size_legacy[sizeof(dbt_smp_t)];
unsigned char dbt_size_sc[sizeof(dbt_smp_sc_t)];
unsigned char dbt_size_legacy_bearer[sizeof(dbt_smp_bearer_t)];
unsigned char dbt_size_sc_bearer[sizeof(dbt_smp_sc_bearer_t)];
unsigned char dbt_size_features[sizeof(dbt_smp_features_t)];
unsigned char dbt_size_bond_record[sizeof(dbt_smp_bond_record_t)];
unsigned char dbt_size_reconnect[sizeof(dbt_smp_reconnect_t)];
unsigned char dbt_size_worker_handle[sizeof(dbt_smp_worker_mbedtls_t)];
unsigned char dbt_size_session[sizeof(dbt_smp_session_t)];
unsigned char dbt_size_session_rx_queue[DBT_SMP_SESSION_RX_DEPTH*sizeof(dbt_smp_session_rx_t)];
