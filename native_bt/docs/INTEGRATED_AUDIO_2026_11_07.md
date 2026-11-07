# Integrated LHDC level, memory and scheduling work — 2026-11-07

LHDC24 decoder output now matches the real Redmagic/LDAC reference within0.014dB in the recorded48k/192k YouTube tests. The192k stereo decoded-input allocation is8192 bytes smaller. Remaining underruns are explicitly unresolved: the current image produces2432 and6080 unfilled frames in two short matched traces. This is development firmware, not a production or maximum-rate qualification.

Current COM6 image:19aae19b7154566ba6a80d6e000a5cb65e82efdd4ea7f172dc147285bc5b63a9.
Physical archive:native_bt/_build/hardware/runs/1794087288687406100-19aae19b7154.
Experiment:native_bt/_build/experiments/integrated-audio-1794081627190.

## Decoder output level

The previous15/16 normalization passed generated C99 encoder round trips, but those tests did not establish that the C99 encoder and the Redmagic encoder use the same level convention. Direct real-phone pre-DSP observations confirmed the reported mismatch. Decimated click measurements initially showed about3.6dB RMS and approximatelysqrt(2) peak excess; periodic stride64 sampling aliases with this metronome and is not used for the final calibration.

The port now applies an additional1/sqrt(2), or-3.0103dB, output trim for24-bit LHDC sources before quality scanning and PCM saturation. Its combined constant is0.6629126073623883f. S16 and source-S32 calibration paths remain unchanged. No buffer, task, extra output traversal, frame-adaptive gain, discarded audio bits or reduced codec setting is introduced. Decoder safeguards and integer DSP/limiting remain active.

A bounded diagnostic discards the first second and then measures every emitted frame for four seconds. It reads signed top16 bits for comparable normalized level, without modifying any PCM. All real audio remains in its original16/24/32-bit container. The decoder owner accumulates integer power; completed metadata is published as lock-free32-bit words for the background logger. This adds diagnostic CPU load during that finite window and must not be confused with free production profiling.

Approved Redmagic, native YouTube120BPM metronome, phone volume150/150:

| Pre-DSP output | RMS,dBFS | Difference from LDAC48 |
|---|---:|---:|
| LDAC48, native S32 container | -23.300023 | reference |
| Corrected LHDC48/S24 in S32 | -23.313349 | -0.013326 |
| Corrected LHDC192/S24 in S32 | -23.308864 | -0.008840 |

Decoder clipped/concealed/nonfinite counts are zero in these corrected runs. These observations establish level matching for the recorded peer/content/settings; they are not all-peer bitstream conformance, acoustic matching, source-S32 qualification or all-content clipping proof.

The known-input gate also checks28 aptX/HD/LL and LDAC configurations through actual workers. LDAC uses the separately pinned [AOSP encoder](https://android.googlesource.com/platform/external/libldac/+/eeee1a3f5f8df1282e3a6d297085885fd886737b/). Observed untrimmed aptX/LDAC gain errors range from-0.000215 to+0.000701dB. The48 LHDC generated cases explicitly expect the product's-3.0103dB S24 trim; S16 remains near unity. They are no longer described as proving external-peer unity by self-roundtrip alone.

Strict native/reference PCM equality still passes. The unmodified sibling decoder's public host calibration hook is set within the reference DLL's own CRT; changing Python's environment after DLL load did not update the static Windows CRT copy. The two failed reference-fixture attempts are retained. Only existing LHDC expected PCM points/hashes were refreshed from that reference. Encoded packets, checkpoint indices, tolerances and37 boot-case count remain unchanged.

## Memory and ownership

The192k stereo S32 input ring now holds1024 frames, sufficient for one960-frame LHDC unit, rather than2048. Asynchronous copying and packet/PCM ownership are unchanged. Processed target remains2048 frames; DMA pages are unchanged. The actual-output model checks31744 versus39936 requested audio/DMA bytes, saving8192 bytes. Five other rate/format layouts retain their previous allocations.

Encoded media remains1800 bytes times8 slots:14528 bytes including slot metadata,14536 including the allocation header. No queue enlargement hides the remaining gaps. The selected decoder workspace remains60704 bytes in live counters.

Finite live traces show minimum internal free heap around26KB, compared with about18KB in the earlier instrumented image. The new level instrumentation costs104 bytes of linked data, and the persistent first-DSP flag changes linked data by8 bytes/IRAM by12 bytes in the measured image. There is no claim that this is the minimum possible complete-stack RAM use or a like-for-like Bluedroid RAM qualification.

## Initial policy and live controls

Previously the first restored EQ policy used the same20ms coefficient ramp as a live audible update. At192k that spans3840 frames, while the processed target holds2048. A failure record had3712 rendered frames,896 decoded frames queued, no processed output and a BUSY helper: the initial scalar-ramp prefix was still active.

Fresh streams now apply their first policy immediately before DSP work. A persistent dsp_started flag is set by real output publication, including mixed silence, and reset on open. Empty render attempts do not mark a stream started. Later volume/EQ/flag changes retain the existing smooth integer ramps. The flag remains correct across32-bit rendered-frame counter wrap. The output fade remains in place. This changes intended startup behavior, not live filter-state safety.

Tests cover the first policy, later ramps, counter wrap,4000 concurrent field publications and the priority-preemption publication model.108 borrowed/copied stream pairs and192 unity/general stream pairs preserve exact PCM, DSP state, mixing, wrap and ownership behavior under their new initial policy. Held-helper stop/cancel, early input reuse, completion-before-preemption and no timed pending polling checks pass.

## Scheduling and jitter reserve

The controller stays priority23/Core1; host and main renderer remain22/Core1; decoder21/Core0 and helper20/Core0. No private cross-core yield API is used.

Top-level owner polls and all15 deadline queries have bounded render checkpoints. They operate only with useful decoded/published work and low output reserve, outside VHCI submission and protocol callbacks. Query expressions evaluate once. Deadline queries share one millisecond epoch and subtract the collection's elapsed time before sleeping, so renderer service cannot extend an absolute protocol deadline.1000000 arithmetic/wrap cases and120 PCM-level format/chunk cases pass. No independent CPU-speed claim is made from inclusive elapsed counters.

LHDC now selects the already-existing40ms asynchronous startup guard used by AAC/LDAC. It is an event/deadline state, not a blocking sleep, and adds zero buffers. It adds startup buffering latency, which existing delay-report accounting includes. The runtime reportsminimum_ms40 andbuffers_added0; this is not by itself proof of the exact first-DMA enable interval in every sound-mixing lifecycle.

At192k LHDC the existing eight-slot encoded queue permits five queued packets before Classic receive backpressure, rather than four. Three previously credited ACL packets still fit:5+3=8. The global controller window stays three. Other codecs/rates retain their policies. The real-policy host test checks the saturation bound and the consumer wake threshold remains policy-driven. No mandatory reply or completion is discarded.

## Physical continuity evidence

Every interval uses native YouTube and an independent14-second pause guard; the helper pauses in a finally path, saves partial failures, preserves raw NDJSON and rejects reused capture names. Settings are explicitly verified after reconnect; Android can revert the rate. COM6 upload921600 and full written-region readback115200 match. The original app is preserved, owned boot is VALID, automatic Bluedroid restoration is false and every deployed image passes the existing37 codec cases.

All following LHDC runs are192k/S24/stereo/fixed900 and phone volume150. Sampled framing remains approximately899200 coded bits/s, not a live1Mbps test.

| Stage | DMA source frames | Unfilled frames |
|---|---:|---:|
| Integrated, before extra level trim |2706240|5248|
| Corrected level |2712960|16256|
|40ms guard |2709120|13184|
|Credit5,first |2714880|4864|
|Credit5,repeat |2731200|9984|
|Initial policy restoration |2719232|4416|
|Query checkpoints,first |2720640|2432|
|Query checkpoints,repeat |2733120|6080|

Runs are sequential and variable; this table is not a controlled isolated speedup claim. Counts include startup/drain, but captured in-play starvation separately proves remaining failures. A finite LDAC48 regression has680512 source frames and zero shortages.

The current repeat's first gap has both PCM rings empty, an idle helper, and a newly admitted packet already being decoded. The last media gap is9519us and the stream maximum36876us; queue-full, decode-reject, sequence-break and SSRC-change counts are zero. Earlier records also captured genuine decoded-input/render-service stalls. These are separate failure signatures. Diagnostic worker trace records can overflow; that reported loss is not silently called packet loss.

Two active DMA counter-snapshot intervals infer approximately192005 and192014 frames/s. IRQ/snapshot error is included; these are not independent hardware-clock measurements. They do not support assuming a large output-clock error as the cause.

## Validation and remaining work

The complete ccache build, source-stability, image-admission and storage gates pass.80 actual-codec deferred cases run with address/undefined instrumentation and strict PCM/timestamp checks, including all37 configurations and the maximum LHDC fixture. No new boot tests were added. Actual codec checks, initial-policy tests, ownership models and physical playback evidence are kept distinct.

The rejected AVRCP bitmap experiment is archived and removed from production code; its two matched runs did not improve overall deadline collection. Existing AVRCP test runners now use ccache, and the paired-runtime source list includes its missing existing ERTM modules;24 protocol cases and3 paired-runtime cases pass.

Passing typed host compiler intermediates and hash-verified reproducible generated tone inputs were reclaimed with manifests. Tested binaries, source, failed receipts, raw phone/UART evidence and Xtensa objects remain. Two accumulated SDK configure logs were losslessly gzipped with decompressed byte equality/SHA verification, recovering57930600 bytes without deleting their contents or changing the storage limit.

Zero software-induced gaps is not achieved. Next work must measure actual startup reserve/enable timing and packet/PCM occupancy across the remaining empty-pipeline events, preserving memory bounds and mandatory credit accounting. Maximum1Mbps, sustained encrypted BLE/control coexistence, live rapid EQ qualification, full codec/product/security/OTA/lifecycle parity, long-duration playback, acoustic checks and comparative whole-host RAM qualification remain open. Do not publish this host as a stable production replacement.

## Subsequent passive music observation

After the controlled tests ended paused, the phone was independently started on a different YouTube music track. A14-second UART-only observation made no phone play/pause/navigation changes. It is diagnostic evidence, not a controlled continuity or acoustic qualification. The stream remains configured192k/S24; its bitrate setting was not re-read during this passive interval.

Using same-record coherent frame-stage sum/count differences over1856 units, decoder-body elapsed time averages2661.01us per5ms unit. The same-image metronome interval averages2089.66us over2848 units. Entropy changes from465.24 to882.86us per stereo unit, while IMDCT remains860.58/860.22us. These wall cycles include preemption and omit adapter/resume time outside the decoder body's measured stages. They do not alone prove complete-pipeline headroom.

An initial approximate5396us ratio combined elapsed and attempt counters whose observation windows differed by roughly ten seconds; that cross-record ratio is invalid and excluded. The aligned frame-stage analysis is retained beside the raw observation. This changes the next optimization priority: busy-content entropy work is approximately as expensive as the transform, rather than relying exclusively on sparse metronome measurements.

Evidence:hardware/runs/1794087288687406100-19aae19b7154/traces/user-music-passive-1794088478863-analysis.json and adjacent raw files. Playback was left under its existing external control.
