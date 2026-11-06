# Shared-core scheduling and bounded startup guard — 2026-11-06

Development follow-up to62aa3238. The full custom-host objective remains incomplete. This report separates source/build evidence, focused digital playback and remaining acoustic/lifecycle/security/max-codec qualification.

## Observed scheduling window

The decoder21/Core0 and light-codec renderer20/Core0 share a CPU. The previous worker quantum could copy decoded PCM and begin another decode before returning. taskYIELD does not schedule a lower-priority renderer while the decoder remains ready. The prior first-gap capture had512 decoded input frames and0 ready output while decoding another unit.

The copied shared-core path now takes one media operation per quantum. When decoded input is present but fewer than256 output frames are ready, it uses a zero-media budget (control/loan retirement still executes), wakes the renderer, and waits on the existing task notification. The renderer notifies after consuming input. Observations use atomic ring counters only, without borrowing buffer/configuration storage. Stop/failure/different-core output keeps the previous scheduling path; separate-core LHDC/Opus retain their two-operation quantum. No priority change, fixed sleep, task or packet queue is added. A separate shared_renderer wait row and first-gap phase8 expose the handoff.

Image9325024c52a117f66dc0330b0aff28b8f2459b742e3d27997f39d01c03d470b9 was written921600 and fully readback115200; owned boot/no fallback, original preserved,37 existing cold codec cases pass. Focused12s LDAC96k traces: plain0 native underruns; EQ/ABR768; highest-quality1280. All accepted=rendered=DMA after flush. The EQ trace first gap had256 input/0 output, decoder waiting for renderer, renderer active402us. This confirms the handoff operated but does not establish an isolated continuity improvement. Do not describe it as an underrun fix on its own.

## Source cadence and quality

The phone was initially Best Effort (Adaptive Bit Rate), so those runs do not prove maximum LDAC bitrate. Its observed Developer setting was changed to Optimized for Audio Quality (990kbps/909kbps),96kHz/32-bit stereo. A same-stream counter interval measured769404 payload bytes over6306061us:976081.9 bit/s, or976.1 kbit/s. This finite payload observation supports the selected mode; it is not sustained maximum-rate qualification.

The highest-quality run recorded one decoder rejection in aggregate; ordinary worker-ring loss omitted the corresponding detailed row. Its cause is not established. That fault remains separate from starvation and must not be hidden by buffering. No queue drop, RTP sequence break, descriptor fault or watchdog was recorded.

The [AOSP encoder source](https://android.googlesource.com/platform/packages/modules/Bluetooth/+/c57f15fc85992d7159ec11af8f6a5e0bcba5ecc8/system/stack/a2dp/a2dp_vendor_ldac_encoder.cc) uses a20ms encoder cadence and256-sample frames at96kHz. This is a source reference, not proof the Redmagic vendor encoder is byte-identical. Incoming observed gaps reached23–25ms. The highest-quality first starvation had empty decoded/output queues. The decoder last entered input wait with encoded queue0,9.863ms before capture; that queue observation is not simultaneous. The latest any-link ACL arrived275us before capture. The maximum admitted-media gap was22.981ms. These traces motivate a bounded startup-margin experiment around the16ms output target; they do not isolate wire, controller or owner receive jitter.

ABR header counts and RTP timestamp increments did not always express the same sample span as the parsed frame count. The active output path does not use those RTP timestamps for PCM clocking. Do not infer duplicate PCM, bitrate or real-time budgets from timestamp deltas alone. LDAC configuration/SDK output checks retain256 samples per96k frame; no sample-rate/depth/quality shortcut is introduced.

## Startup policy

Only AAC/LDAC opt into the new minimum first-accepted-PCM guard. LDAC uses40ms; AAC uses the greater of40ms and two complete1024-frame units, rounded upward (47ms at44.1k,43ms at48k). Other codec guards are unchanged. The existing packet/PCM capacities and steady-state target remain unchanged. The decoder may predecode until existing PCM backpressure and the existing1800x8 encoded queue retain the remaining lead.

The producer arms the timestamp before notifying the renderer. The owner enables I2S after both the guard and existing prefill target are ready. A deadline in the existing notification wait wakes the owner; DONE retires it, without overdue one-tick polling. This adds intentional startup latency and gives AVDTP Delay Report a floor reflecting that component. Existing encoded-queue latency accounting is not claimed complete.

Normal stop during the new guard can bypass it and drain a safely preloadable accepted clip. Cancellation keeps its existing reclaim semantics. The legacy sub-DMA short-clip behavior remains a separate limitation; this change does not qualify every stop/cancel case. Standalone WAV and attached/transitioning sounds bypass music priming to preserve their progress.

Target assertions prove the guard reuses request-prefix padding. Selected ELF symbols show output state3336->3344 bytes and three decoder-wait arrays16->20 bytes:20 added retained bytes total in those changed live symbols. Stats add local snapshot fields. No new buffer/task allocation is introduced; this is not a whole-firmware RAM comparison.

Candidate153107a5a4d10bb41b26c4415d889912067e5412dea931f2928eb5a4f3911aa1 builds with ccache and10349 stable source inputs. One intermediate source-changing build was rejected by its fingerprint gate and was not deployed. Full readbacks and37 existing cold checks passed. Two ABR LDAC traces had0 underruns; their393kbit/s measured payload proves they were not maximum-quality evidence despite the intended high run name. Explicit990 mode then measured988kbit/s but caused4 host queue drops/resets and3072 underrun frames. This motivated the receive-credit follow-up, not an all-codec continuity claim.

Private evidence: native_bt/_build/experiments/shared-render-handoff-1793975538770 and ble-a2dp-regression-1793967920295/handoff-ldac96-1793975852879, secured-eq-ldac-1793975914897, secured-eq-ldac-1793976161306. Raw/private evidence is not published with firmware backups or keys.

AAC44.1k/S16 guard47ms first and resumed12s traces each had0 native underruns and0 new queue/decoder failures. Their flushed accepted/rendered counts were659968; DMA-source659442 and659314 respectively at stop. The existing20ms stop fade can end before every queued frame; do not claim equal end-of-stream counters or acoustically verified continuity. These AAC runs preceded the receive-credit change.

See RX_CREDIT_BACKPRESSURE_2026_11_06.md for the subsequent maximum-quality gate tests and the distinction between an EOF refill shortage and the actual DMA reuse deadline.
