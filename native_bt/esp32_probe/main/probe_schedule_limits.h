#ifndef PROBE_SCHEDULE_LIMITS_H
#define PROBE_SCHEDULE_LIMITS_H
#include "dbt_esp32_affinity.h"

#ifndef DBT_PROBE_DECODER_PRIORITY
#define DBT_PROBE_DECODER_PRIORITY 5
#endif
#if DBT_PROBE_DECODER_PRIORITY != 5 && DBT_PROBE_DECODER_PRIORITY != 6 && DBT_PROBE_DECODER_PRIORITY != 21
#error Unsupported decoder priority
#endif
#if defined(DBT_PROBE_HOST_PRIORITY) || defined(DBT_PROBE_RENDER_PRIORITY)
#error Task priorities are derived from the admitted scheduling mode
#endif

#ifdef DBT_PROBE_STAY_NATIVE
#if DBT_PROBE_DECODER_PRIORITY != 21
#error Product decoder requires isolated CPU0 priority 21
#endif
#define DBT_PROBE_HOST_PRIORITY 22
#define DBT_PROBE_RENDER_PRIORITY 20
#elif defined(DBT_PROBE_PRIORITY_ORDER)
#if DBT_PROBE_PRIORITY_ORDER != 1 || DBT_PROBE_DECODER_PRIORITY != 6
#error Ordered priorities require the finite priority6 decoder mode
#endif
#define DBT_PROBE_HOST_PRIORITY 7
#define DBT_PROBE_RENDER_PRIORITY 5
#else
#define DBT_PROBE_HOST_PRIORITY 6
#define DBT_PROBE_RENDER_PRIORITY 6
#endif

/* Yield only offers service to a ready equal-priority decoder. It cannot
 * schedule a lower-priority task while the owner itself remains runnable. */
#define DBT_PROBE_OWNER_READY_YIELD (DBT_PROBE_DECODER_PRIORITY == DBT_PROBE_HOST_PRIORITY)
#endif
