# Bounded five-block DSP assistance — 2026-11-07

The full Bluetooth/audio goal remains active and incomplete. The retained development selection uses five canonical128-frame helper blocks instead of four, after the BUSY-refill and completion-wake fixes. Buffer sizes, fidelity, task priorities and core affinity remain unchanged. Four blocks remain the source/build default.

## Bounded option and ownership

Use `-DspAssistBlocks 5` with `-DspAssist` in PowerShell. The admitted values are4 and5; default4 is explicitly passed to CMake on every build, clearing a prior5 selection. CMake, compiled header, build receipt, exact image checks and UART max_blocks_per_job agree on the value. Numeric type, dependency, duplicate/missing/overridden macro and per-unit mismatches reject admission.

A lease remains one DSP/lifecycle borrow, with at most640 frames in five128-frame blocks. Every completed span is release-published and can serve the existing guarded descriptor window while the helper remains BUSY. Attached sound retains its existing per-block readiness restriction. The owner still retires DONE and supplies its separate local block. No task, stack, pool, allocation or new wait is added. Raw/processed/DMA payload remains39936 bytes:2048 raw frames,2560 processed capacity/2048 target and3x128 DMA.

## Checks

Actual source tests pass ASan/UBSan in the full fair/early/rebalanced mode at both4 and5, plus ordinary5 mode. Supplying one block beyond the cap proves the helper publishes only512/640 frames before DONE, including page-boundary crossing; the owner supplies the additional128. Stop/cancel, no early free, early input reuse, five unaffected rate/format layouts, sound/lifecycle signals and producer-preemption completion tests still pass. Fixed-point arithmetic keeps its established wrapping option. The guarded BUSY-refill test also passes at100Hz/1000Hz model clocks. These are host models, not MCU timing.

Admission covers20 actual CMake,51 actual image,9 compiled header and2 early CLI cases. Existing task-cycle, early-input and PCM-rebalance gates still pass. The complete product builds with ccache, all codecs/product/security/RFCOMM included, temporary task-cycle hooks OFF. New image validation also checks the helper boolean and dependencies, not just the new batch number.

## Guarded hardware evidence

Baseline a9427d8cafde6e89ee4826435e392fe7eef1653ba7e7b8873a60ff01963fdb57, archive runs/1794037693865568200-a9427d8cafde. Candidate041f25beb2ef58aaecf12a8ce0d83afe51ca77d287988e7e502009330f9f4609, archive runs/1794039193932149000-041f25beb2ef. Both full candidate app/selector readbacks match (write921600/read115200), owned VALID boot and37 existing cold codec cases pass, original app/table/bootloader remain preserved, no automatic fallback. UART reports five blocks and an exercised640-frame maximum.

Redmagic native YouTube120 BPM metronome, UI LHDC192k/24/stereo/fixed900, phone volume80/150, sampled DSP volume68/EQ2/8/12/flags12. Every command interval has an independent14s pause guard; aggregate fields include startup/drain and are not a complete acoustic/per-sample proof.

| Mode | Command interval | Accepted/rendered/DMA frames | Initial shortage | Recovered | Unfilled | Copy/mask max us |
|---|---:|---:|---:|---:|---:|---:|
|4-block baseline|12.066s|2684160|453120|253824|199296|6/9|
|5-block run1|12.077s|2756672|260224|135552|124672|5/9|
|5-block run2|12.240s|2814720|211968|103168|108800|7/9|

Observed unfilled totals are37.4%/45.4% lower than the fresh baseline. This is repeated short-segment evidence, not an isolated mathematical speedup or sustained continuity pass. Core0 helper runtime is about36.7–37.6%, decoder50.7–52.7%, idle10.6–12.0%; the prior measured four-block windows had about33–35% helper and14–16% idle. Runtime includes IRQ/current-quantum coverage limits. No descriptor/contract/helper fault or new queue/decode/PCM/sequence rejection is recorded. Detail losses3439/2833/2831 are retained separately from zero aggregate losses. Since-boot helper maximum wall7243us includes asynchronous waiting/preemption and is not a pure arithmetic duration.

LDAC96k/stereo/32-bit phone input,12.197s native YouTube, accepts/renders/delivers1445632 source frames with zero initial shortages/unfilled, descriptor or contract errors. Wire samples661 bytes minus1-byte media header and512 RTP steps at96k are consistent with990000 compressedbps. This is sampled inference. Detail loss5469 is retained; no new aggregate or packet/decode/PCM/sequence errors occur.

Relative to the prior normal image, flash text+4 bytes; IRAM/data/BSS/rodata unchanged. Matched active LHDC records show60704 codec-resident bytes,19604 free internal heap,18432 largest block and5960 decoder stack low-water in both candidate runs. The older baseline has prior codec-switch history and19592 free/16384 largest; this is not a leak/fragmentation comparison. LDAC has28196 resident/68524 free/57344 largest in the active sample.

## RTP clock discrepancy: separate from local drops

The RTP decoder reads the timestamp directly from header bytes4–7; the generic worker copies it unchanged, and the LHDC wrapper preserves it with only a per-frame offset for multi-frame packets. Sampled adjacent raw RTP sequences step960 ticks, consistent with5ms at192k. Complete per-stream counters are all566-byte media packets. Packet counts2799/2872/2932 cover2687040/2757120/2814720 samples at960 each, but first-to-last RTP spans including the final frame cover2875200/2864640/2895360. Excess clock spans are188160/107520/80640 samples (980/560/420ms), multiples of28 five-ms units, despite zero input sequence breaks. Two late baseline packets are not decoded before suspension; that is far smaller than the clock discrepancy.

This demonstrates clock timeline advancing beyond received frame content, not an ESP queue-full or decoder-size rejection. It does not yet prove the mechanism on the handset. The [AOSP software source](https://android.googlesource.com/platform/packages/modules/Bluetooth/+/refs/heads/main/system/btif/src/btif_a2dp_source.cc) can flush encoded buffers on transmit-queue overflow and distinguishes offloaded operation. Congestion flushing is a hypothesis here; the Redmagic vendor/offload path is unverified and read-only logcat queries returned no retained matching-tag records. No source-side settings were changed and no missing-audio concealment was added.

Exact sources, separate test/flash/trace receipts, RTP accounting and CPU/heap samples: native_bt/_build/experiments/assist-five-1794038745721. [Structured results](ASSIST_FIVE_BLOCKS_RESULTS_2026_11_07.json) retain the scope and image binding. Full1000kbps, sustained LHDC continuity, encrypted coexistence, acoustic/control/OTA/security stress and complete feature parity remain open. The phone is left paused on working LDAC and the raw UART monitor is listening.

## Reproduce the retained development selection

This is a build-only command; the guarded COM6 deployment remains separate. Omitting the optional batch argument restores the default four blocks on the next build.

```powershell
.\native_bt\tools\build_probe.ps1 -DspAssistBlocks 5 -DspBlockIram -ConsoleIrq -TaskRuntimeStats -HighRatePcmRebalance -EarlyInputRelease -RenderFairShare -VolumeRampBlock -DspAssist -UnityPcmCopy -StayNative -WordDsp -PriorityOrder -CodecAwareRender -SchedulerTrace -AudioStageProfile -PcmCopied -BlockDsp -PcmTargetMs 16 -DecoderCore 0 -LoggerCore 1 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir "D:\ESP32 Bluetooth\LHDC-V5-Decoder-main"
```

The later [input-clock diagnostic](RTP_INPUT_CLOCK_2026_11_07.md) confirms a consecutive RTP sequence pair with an extra28 frame periods before media-queue admission and decode. It retains helper5, adds24 bytes of bounded metadata, and does not attribute a sender/credit/RF cause yet.
