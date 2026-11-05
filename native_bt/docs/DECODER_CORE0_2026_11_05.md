# Decoder isolation and Bluedroid comparison

The released baseline at commit 0d85a975ec079d954f1bdd3e94d2fed70848c252 in D:/bt-idf61 was reviewed using its built SDK configuration and actual task-creation calls. The user reports good playback for all codecs except maximum LHDC V5; no fresh baseline campaign ran.

| Task | Baseline core / priority | New product core / priority |
|---|---|---|
| Controller | 1 / 23 | 1 / 23 |
| BTC / owned host | 1 / 19 (BTC) | 1 / 22 |
| A2DP decoder | 1 / 19 | 0 / 21 |
| DSP/I2S renderer | 0 / 23 | 1 / 20 |

Baseline main.cpp:2135 creates the renderer at configMAX_PRIORITIES-2. The SDK btc_a2dp_sink.c explicitly selects media-decoder core 1 and priority BT_TASK_MAX_PRIORITIES-6. Both maximum-priority constants resolve to 25. Its built header confirms Bluedroid/controller core 1; set_task_core(1) alone is not the evidence for media-decoder affinity.

The owned path previously used host 7, decoder 6 and renderer 5, with host/decoder on core 1 and rendering on core 0. It also adds a full FIR resampler and integer DSP, so this was not a host-only substitution. Existing render-budget failures and PCM backpressure remain material. Low delivered bitrate alone does not distinguish absent packets, insufficient decoder scheduling or a full output path.

## Requested isolation

The user explicitly requested nearly all of core 0 for codec execution. The new layout follows that instruction rather than copying the baseline's core assignment. All codecs use the same core-0 priority-21 worker. Controller/host retain higher priority; DSP cannot run on or preempt the decoder's core.

All compiled app task-creation sites were checked. Configuration, trust, bonds, boot, control/EQ storage, SMP, logging, input and LED workers are pinned to core 1. Previously unpinned workers are included. Built-in codec startup checks use core 0 separately from live product decoding. No task or stack is added.

There is no added fixed decoder delay or retry spin. Runnable work continues in bounded steps; other waits use notifications and actual ownership/protocol deadlines. PCM backpressure remains explicit. Priority cannot manufacture output capacity or establish FIR/DSP headroom.

Core 0 retains required FreeRTOS idle/tick/IPC, interrupts and short SDK housekeeping. The original core-0 esp_timer/restart contract remains: its product restart callback runs only after the audio/runtime quiescence barrier. FreeRTOS timer-service, Wi-Fi and TCP/IP task affinities move to core 1. Flash operations can still affect both CPUs; affinity is not a flash-latency fix.

The new dbt_esp32_affinity.h centralizes placement. Build defaults select decoder core 0 and logger core 1, with product priority 21. Existing SDK configuration choices are updated before fingerprinting. CMake, compile-time assertions and image admission enforce the exact product policy. Historical finite diagnostic modes retain their separate 5/6 priority policy.

## Decoder wait records

The decoder records its actual execution core without printing from that task. The low-priority logger emits separate decoder_wait records for pcm_space, pcm_loan_retirement, control and encoded_input. Each has completed wait calls, elapsed/maximum time, and an advisory current-wait snapshot. Time includes sleep and preemption; it is not decoder CPU utilization or pure scheduler-dispatch latency. An already queued notification can return immediately and still count as a wait call.

render_work reports the renderer's actual task core; i2s_callback_work retains its actual interrupt-core observation. scheduler_layout reports host execution core and the configured policy. Input, queue-full, decode-reject, PCM-busy, underrun and stage-timing counters remain separate. Added numeric observation state is 64 bytes, with no new log queue.

## Evidence boundaries

Compilation/source review and ordinary startup are distinct from streaming qualification. No separately invoked playback, pairing, app or unit campaign runs during the user's test hold. Built-in firmware startup codec checks are recorded separately. Startup can establish the configured layout and observed idle worker cores, not LDAC continuity or maximum LHDC headroom.

Evidence is retained in native_bt/_build/experiments/decoder-core0/. Underruns, reported codec crashes, full feature/security/lifecycle parity and maximum-rate headroom remain open. This change does not claim that every underrun is fixed.

## COM6 deployment

Image 0253143be712ef8024776b36094913407854f850fc8ca4cd52fbedf19388397c was flashed to ota1 at 0x390000 at 921600 baud. Full image/selector readbacks at 230400 matched and the original application remained unchanged. The owned host booted ready, stayed VALID and did not restore Bluedroid. The retained startup window contains no watchdog or backtrace. Built-in codec startup checks passed.

The host startup record reports core 1 and priorities 22/21/20. Separate decoder-wait records observe execution on core 0; render_work observes execution on core 1. I2S interrupt-core evidence requires a nonzero callback count and is not inferred from an idle default value. Live UART resumed with acknowledged source 53.

The idle snapshot reports 154536 internal heap bytes free, 130508 minimum free, and 110592 largest block. These are startup figures, not an equal-workload RAM or playback result. Archive: native_bt/_build/hardware/runs/1793907274894429700-0253143be712/.
