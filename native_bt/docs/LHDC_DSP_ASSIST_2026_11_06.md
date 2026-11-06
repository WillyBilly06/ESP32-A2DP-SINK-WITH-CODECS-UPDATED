# Bounded LHDC DSP assistance - 2026-11-06

192 kHz / 24-bit LHDC still underruns. This is an implemented scheduling experiment, not a maximum-rate fix. The existing integer DSP/EQ arithmetic and floating-point LHDC transform are unchanged. All codecs/product/security/RFCOMM remain enabled. The user reports aptX HD and LDAC sound good on the earlier image92fc7e5705b7; those acoustic reports do not qualify these subsequent images.

## Ownership and bounded work

The opt-in -DspAssist build requires stay-native, asynchronous copied PCM, codec-aware rendering, block DSP and the exact unity-copy path. It excludes whole-LHDC Core0 rendering and PCM boundary capture. Bluetooth/controller and I2S lifecycle remain on Core1. LHDC decoding remains priority21/Core0. The existing lazy priority20/Core0 render task may assist when the decoder reports waiting for PCM space; other codecs retain their policy.

Audio preparation copies at most128 stereo S32 frames into an existing unpublished output span. It does not advance input.read/output.written. A nonzero nonce and immutable metadata identify the reservation. The portable lane uses lock-free IDLE -> READY -> BUSY -> DONE release/acquire publications, with one CAS claim attempt and no spin/retry/allocation. The renderer returns to notification sleep while the borrow is active. Stop/cancel, configuration, raw DSP telemetry and teardown remain fenced before any borrowed access. The whole lane fences lifecycle even between audio reservations and after PCM publication.

The Core0 worker runs the unchanged canonical block math and commits mixing and PCM/input cursor publications itself, while still BUSY. DMA can consume completed PCM without waiting for Core1. Up to four canonical blocks /512 frames share one lease, bounded by the captured output deficit and current input/output availability. Every span is published separately. Empty input/space ends the batch immediately. An attached WAV mixer keeps its existing single-block readiness boundary. DONE follows the final borrowed access; only the Core1 lifecycle owner retires the lane. Decoder notifications follow DONE. The decoder gives a queued/busy helper a turn before starting another media operation; zero media budget still permits worker control retirement.

Stale/duplicate completion, wrong token and invalid matching metadata reject explicitly. A failed matching audio transaction fences further rendering until close/reopen. Close/open cannot free a prepared reservation. No second DSP instance, PCM buffer or task class was introduced. Stream/API ownership is still required; an audio reservation alone does not protect the larger multi-block lease between individual reservations.

## Focused host checks

ASan/UBSan PCM/state comparison passes192 stream pairs,787776 exact output frames,580 prepared jobs and265216 explicit copy-path bytes. It covers four rates,16/24/32-bit mono/stereo, unaligned external input, poisoned retired input, page/ring/counter wrap, variable budgets, rapid integer controls, mixer callbacks, fallback formats/modes, stale/duplicate/failure/token-wrap cases and a real Windows worker thread.

A separate test runs actual output/render/sound and lane C against two mocked FreeRTOS cores. Holding the worker before math proves stop/cancel cannot free early. Holding it after all four PCM publications but before DONE proves DMA-visible PCM does not release lifecycle ownership early. Both four-block and one-block cases pass; the decoder yields while borrowed, terminal receipts occur once, restart becomes quiescent, main blocking waits and blocking queue sends are zero, and pending waits use notifications rather than timed polling. These are deterministic mocked interleavings and memory/arithmetic sanitizer checks, not a data-race sanitizer, running-DMA test, ESP32 timing result or acoustic qualification.

Commands: tools/test_audio_unity_copy.py --dsp-lane and tools/test_dsp_assist_output.py. Both use ccache. The target rebuild caught a missing block API declaration in the new batch caller; the assist-only target definition now exports the existing declaration. Failed build/mock receipts are preserved privately.

## Physical results

Approved Redmagic, COM6 only, native YouTube, about12.2 seconds between play/pause commands with an independent14-second pause guard. Reconnection resets sample rate AND quality, so every accepted run explicitly restores192k/S24/stereo/fixed900. Received packets have566 media bytes and one5ms frame:899200 compressed bits/s after media/frame headers. The phone exposes900 as its highest fixed choice; these traces do not qualify1000kbps. Post-pause drain is included in the separate cumulative counters below. Different music segments/startup/tail windows prevent an isolated speedup percentage claim.

| Worker transaction | Assisted jobs | Canonical blocks | Assisted frames | Max math-finish to publication (us) | Max publication to Core1 retirement (us) | Initial shortage frames | Late recovered frames | Unfilled frames |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| One block; Core1 publishes | 3901 | 3901 | 498176 | 7084 | included in previous column | 793984 | 373248 | 420736 |
| One block; worker publishes | 5309 | 5309 | 679168 | 42 | 8723 | 655168 | 300608 | 354560 |
| Up to four blocks; worker publishes | 1881 | 7518 | 955136 | 41 | 14933 | 330560 | 68032 | 262528 |
| Forced half-frame split (rejected) | 2506 | 9847 | 1217408 | 46 | 14166 | 681152 | 221696 | 459456 |

The batch run accepts/renders/delivers to DMA2644800 source frames. Initial shortages continue growing through playback; they are not limited to startup. Its first shortage still has1024 input frames, zero output frames, six encoded packets queued, decoder waiting for PCM space, and Core1 inside local rendering for about4.5ms; the assist lane is IDLE at that event. Worker publication removes Core1 from publication of an assisted block, but Core1 still serializes subsequent dispatch/retirement. The latter can delay the next job even after the worker is done.

All four runs have zero recorded descriptor faults, DSP-job failures, watchdog events, codec rejects/conceal/nonfinite/clipped frames, encoded queue drops and RTP sequence/SSRC breaks. Batch max DMA EOF gap737us. Batch Core1 local DSP maximum4043311 cycles includes preemption (about16.85ms); helper max single-block329082 cycles also includes preemption. These are elapsed cycle observations, not isolated arithmetic CPU time. High-priority console-log samples do not increase during the single-block playback window, so that capture does not support blaming main-console output. Complete raw timing is unavailable: batch worker detail trace loses4767 records, while aggregate snapshot loss is zero. Do not infer acoustic continuity or complete trace coverage from clean transport counters.

Batch post-stop internal heap is65760 bytes, minimum25068, largest63488; decoder stack low-water5952 bytes and main renderer3804. The reusable auxiliary stack reservation is6144 bytes plus TCB; its target low-water is not independently recorded here. The batch build adds160 bytes DRAM data,28 bytes IRAM text,3544 flash text and656 flash rodata relative to the non-IRAM unity-copy baseline, with unchanged BSS. It does not add PCM storage. These linked deltas include instrumentation and the prepared-job API.

All images are full product builds with10360 stable source inputs. App and selector writes at921600 and full readbacks at115200 match. Original app/bootloader/table remain preserved, owned VALID boot passes, no automatic Bluedroid restore occurs, and37 existing cold codec checks pass. No new boot test was added.

Images: first e02b6ef8fca9e3d2fa0fa976b6ccce5d5c6523af30c15024fa294d879afb2fa5; direct publication dd554680f24b4cbb994e2c2fad0f154214d745237e9826508a44c54ac12b9d77; bounded batch2fd4bbc8dba4a61a2c93ec762b35c1d18c12bcfe33b98b34e185c697b5779263. Current COM6 image is the batch candidate. Exact build/readback/boot archives remain under _build/hardware/runs; raw UART/settings/failed receipts are under _build/experiments/lhdc-dsp-assist-1794002414693.

## Rejected forced-half dispatch

A subsequent policy used bounded signed frame credit to give roughly half the DSP frames to each core, regardless of whether the decoder was already waiting for PCM space. The credit was bounded to512 frames and reset per stream; a deterministic host test confirmed512 helper plus512 local frames with no early teardown. That arithmetic/ownership gate did not establish target throughput.

Image8eba1f53f0a37fd946a1444ca224b3e5b94f0c2b80d9ce1154de4bfc9117dcbc achieved1217408 helper frames out of2434560 rendered frames, but still had459456 unfilled frames. The first gap has576 input frames, zero output and laneREADY about1.2ms after submission: a queued worker has not claimed the job. The decoder-side first-gap snapshot is explicitly incoherent, so it cannot prove an exact simultaneous decoder phase. Queue maximum2912us and retirement maximum14166us remain. Drops, rejects, descriptor faults and watchdog events remain zero; detail trace loss3099 is separate from zero aggregate snapshot loss.

This changes the stalled path without curing it and has no demonstrated overall speed benefit. The forced-half policy is rejected and removed from the retained source. Its exact source and failed receipts remain in the private balanced-batch experiment. The subsequent ccache rebuild reproduces the exact earlier2fd4bbc8dba4 binary; restoring that owned image is not a new throughput result. The retained policy again requires the decoder's atomic PCM-space wait hint, with up to four blocks and worker-side publication. No full fixed-point transform conversion was made.

The existing DBT_PROBE_OWNER_READY_YIELD option only taskYIELDs to equal-priority tasks. With host22 and renderer20 it is not a lower-priority renderer scheduling guarantee. Core1 runnable-work streak duration and DSP preemption need measurement before changing that policy; per-call host maxima do not prove the duration or cause of the entire observed16ms rendering span.

## Remaining work

This experiment shifts roughly36% of rendered frames into the batch helper but still misses continuous output deadlines. Investigate the long Core1 local-render and completed-job retirement intervals using separate scheduler evidence. Reduce dependency on a delayed Core1 dispatch without transferring all DSP to the already busy decoder core. Any continuation scheme needs a bounded work quantum, immediate producer/control progress, one DSP owner and explicit lifecycle fences; merely enlarging a batch risks starving decode or leaving Core1 idle. Full1000kbps, BLE/control/storage coexistence, sustained headroom, acoustic continuity and complete feature parity remain open. No unchecked fixed-point transform rewrite or quality reduction has been adopted.
