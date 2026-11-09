#ifndef LHDC_DEC_SLOT_H
#define LHDC_DEC_SLOT_H

/*
 * Concurrency "slot" = which CPU core is currently decoding a channel.
 *
 * On the classic ESP32 the two stereo channels are decoded IN PARALLEL, one per
 * core: channel 0 on the A2DP decode task (pinned to core 1) and channel 1 on a
 * small worker task (pinned to core 0, alongside the BT stack which leaves ~63%
 * of that core idle). The channels are independent (LHDC V5 transmits direct
 * L/R, no in-frame cross-channel dependency), and the entropy stage is a serial
 * range coder, so the two channels are the ONLY parallelism available.
 *
 * Because both decodes run at the same instant, every piece of per-decode
 * scratch that used to be single-instance (and simply reused for channel 1
 * after channel 0 finished) must be duplicated NSLOTS-wide and indexed by the
 * running core id, so the two decodes never touch the same scratch.
 *
 * Everywhere else — the ESP32-S31 target and the host round-trip build — NSLOTS
 * is 1 and the slot is always 0, so every [NSLOTS] array collapses to a single
 * instance and the decoder behaves exactly as the original sequential code.
 *
 * Set LHDC_DEC_DUAL_CORE to 0 to fall back to the sequential decoder in one
 * place (e.g. if the second scratch set ever stops fitting in byte-DRAM).
 */
/* DISABLED. Dual-core parallel channel decode was implemented and runs, but on
 * device it CORRUPTS audio: the conceal rate goes 0% -> 11% of frames with peaks
 * 100x-7600x the clip limit, i.e. some per-decode scratch is still shared between
 * the two concurrently-running channels (a race I have not isolated). It also does
 * not pay off: measured total load rose 154% -> 184% (shared flash-cache contention
 * between the two cores + barrier overhead), leaving core0 86% / core1 98%. The
 * sequential decoder is both correct and faster here. Set to 1 only to resume
 * debugging the race. */
#define LHDC_DEC_DUAL_CORE  0

#if defined(CONFIG_IDF_TARGET_ESP32) && LHDC_DEC_DUAL_CORE && !defined(LHDC_HOST_BUILD)
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #define LHDC_NSLOTS      2
  #define LHDC_DEC_SLOT()  ((int)xPortGetCoreID())
#else
  #define LHDC_NSLOTS      1
  #define LHDC_DEC_SLOT()  0
#endif

#endif /* LHDC_DEC_SLOT_H */
