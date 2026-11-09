# Rejected DMA refill experiment — 2026-11-08

The 256-frame DMA experiment was rejected as an audio fix and removed from the working source. COM6 was returned to the normal 192-frame own-stack build; no Bluedroid fallback was selected.

## Hypothesis and fixed memory contract

The preserved Bluedroid application source in this worktree configures 3 descriptors of 128 frames. Its nearby comment names 3x256; the comparison uses the assignments. The owned normal build uses 2 descriptors of 192 frames. Both store 384 DMA frames, but their reuse intervals differ. The experiment used 2 descriptors of 256 frames and moved 128 frames out of the processed PCM ring. This was a bounded geometry experiment, not an increase in total PCM payload.

| LHDC192/S24 stereo | Normal | Experiment |
|---|---:|---:|
| Decoded-input frames |1024|1024|
| Processed-ring capacity |2560|2432|
| Processed-ring target |2048|1920|
| DMA descriptors xframes |2 x192|2 x256|
| Total PCM/DMA payload |31744 B|31744 B|
| Startup processed-plus-DMA reserve |2432 frames|2432 frames|

At 192 kHz, nominal EOF frequency changes from 1000 to 750 per second, and the conservative refill period changes from 1000 to 1333 us. Urgent math remains 64 frames per step; at most one descriptor worth of productive steps may precede equal-priority host service. The six-block helper cap, core assignments, priorities, codec/framing configuration, precision and gain settings remain unchanged.

The actual output/DSP lifecycle model passed for both geometries under ASan/UBSan, including frozen helper ownership, queued stop/cancel, early input reuse and the 8 ms reserve boundary. Actual refill/order/expiry/cancel, wrap/coherence and exact split-versus-full DSP samples with EQ/volume changes passed at 100/1000 Hz. Header/CMake/image admission passed 5/8/16 cases. These are host models, not MCU timing proof.

## Hardware result and rejection

Readback-verified candidate 3ae5b531769d used the normal feature preset with heap/tick/task samplers OFF. Static RAM did not grow; IRAM text added 4 B. Existing cold checks passed 37 configurations /173 callbacks with no boot watchdog/backtrace marker. Live geometry confirmed 8192 B decoded input, 2432 output frames and 4096 B DMA, totaling 31744 B payload.

The single guarded native-YouTube/LHDC192/S24/stereo/phone-fixed900 music excerpt had a firmware lifecycle upper bound of 12.788041 s, from verified stream-start acknowledgement to confirmed renderer stop. That includes startup and avoids rounding a fractional DMA period downward. Controls were acknowledged, read back and original subscription restored before pause. Minimum free heap was 20280 B; stopped free 62060 B and largest block 59392 B.

It retained 31552 initial shortage frames, 17600 recovered and **13952 unfilled**. Decoder/media drops, decoder faults, descriptor errors and watchdog events were zero. The first unfilled event at 224566816 us took 192 of 256 required frames, with decoded input and output both empty. The decoder intent was decode, 2347 us into that phase; helper state was idle and the renderer was outside audio_render. The associated media callback arrival was 224563661 us, admitted at 224564434 us. The last host-admission media gap was 10557 us; these observations do not establish RF loss or completed decoder CPU cost.

A verified decoder_wait record was stamped at 224616607 us, after the first miss, and received on the PC at 1794205444.2891986. Pause was requested at 1794205446.1412702 on that same PC clock. Thus the first miss preceded the requested pause; no MCU/PC offset alignment is needed for that causal ordering. This does not classify every unfilled frame.

The earlier normal music excerpt had no underruns but its BLE link connected after pause; the candidate completed controls during playback. The runs therefore are not a matched A/B performance comparison, and the count difference is not attributed solely to geometry. The candidate failed the zero-gap requirement, so a metronome run and default promotion were skipped. Its source, immutable image, trace and failed receipts remain preserved.

## Normal restoration and remaining work

Normal image b302e29ecaed1b34eb495d40cdf7478a4c5e37863081baf6032508c45ae7167e was built with ccache and 10433 stable inputs, admitted, flashed at 921600 and fully read back at 115200. It has the published 72089fca source and 192-frame DMA geometry. All 12 allocated ELF code/data sections match baseline f2062fa445c6 except .flash.appdesc build metadata. Cold checks again pass 37 configurations /173 callbacks, minimum after allocation 106104 B, with no boot watchdog/backtrace marker. Automatic Bluedroid restore remains disabled; the old application region is preserved. BDK Audio is back in the foreground, Classic control is connected, YouTube is paused, and the selected LHDC 192 kHz/S24/stereo/phone-fixed900 settings were read back after restoration. The verified UART monitor is listening at 921600.

The first restoration reconfigure failed after a manual cache edit; CMake cache was regenerated. A later post-build 512 MiB artifact gate failed; completed logs/symbols were compressed losslessly with hashes before the successful rerun. These failures were retained, not relabeled as successful builds. SDK objects/ELF/headers and device evidence remain.

The first candidate miss occurred before PCM publication; the previous normal metronome first miss had decoded PCM available. These are different observed failure states. The next diagnosis needs adjacent media-ingress timestamps, host delivery and decoder turnaround, rather than treating all shortages as one cause. Controller modem sleep and system PM are disabled in the current SDK configuration. No codec-performance, sustained 1 Mbps, acoustic or full-feature qualification is claimed.

Machine-readable [results](DMA_REFILL_EXPERIMENT_RESULTS_2026_11_08.json) retain the rejected outcome. Device/source capsule: native_bt/_build/hardware/runs/1794205064654282700-3ae5b531769d (including experiment-source/SHA256.json and traces/dma256). Normal restoration: runs/1794206428092279900-b302e29ecaed. Host models, guard cases, failed builds and lossless archive indexes: native_bt/_build/experiments/dma-refill-256-1794204349597.
