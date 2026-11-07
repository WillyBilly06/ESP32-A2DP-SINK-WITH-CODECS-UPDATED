# Host phase charge and deadline clock — 2026-11-06

The full Bluetooth/audio goal remains active and incomplete. LHDC still misses output deadlines. This experiment follows the decoded-size audit in PCM_BUFFER_HANDOFF_2026_11_06.md; it does not enlarge queues or reduce codec fidelity.

## Correcting the measurement

The optional TaskCycleAccounting diagnostic now has separate permanent host and renderer slots. Switch hooks make two identity comparisons, remain call-free IRAM code and allocate no task, stack, queue or heap. Own-task marks include the running tail and interrupted-task IRQ/marker work; switched-away execution and OUT-to-IN selection are excluded. Marks use the supported ISR-context query under the outer local interrupt mask. These are scheduler-charged cycles, not pure instruction timings.

The first diagnostic ff824fb2e193 and caller-capture image07672281b4af reported a non-owner phase call before audio. No playback was commanded on either image, and their frozen totals are rejected for CPU interpretation. Captured caller0x400ddd04 resolves to host_printf at probe_main.c:20. The shared print wrapper is used by the background owned-boot worker as well as the host; background tagging could overwrite the global host phase. The wrapper now tags only the registered host task. All messages still print. Existing interrupted-task console samples from before this correction cannot establish real host logging CPU cost.

## Valid baseline

Image2ece8d96fef6cb57ed1b25c58af12e94ac4811decd76586339ead6e4bdf242f1, archive runs/1794034759718241700-2ece8d96fef6: full app/selector readbacks, owned VALID selection, original preservation and all37 existing cold codec cases pass. No automatic fallback. Hook/invalid/context faults remain zero before and through playback.

Redmagic native YouTube 120 BPM metronome, LHDC V5 UI192k/24/stereo/fixed900, volume80/150. A newly received coherent host baseline was saved after settings settled. Commanded playback12.278s, independent14s pause guard,7s post-pause capture. Incoming payload maximum566 bytes, oversize/queue/decode/PCM-failure/sequence counters zero. PCM busy7939 is explicit backpressure, not zero work waiting. Accepted/rendered/DMA source counts all2656128; initial shortage416256, recovered142848, unfilled273408, descriptor faults0. Counts span playback and drain; do not treat a source-empty tail as in-play failure without timing evidence. Aggregate record/UART losses0, worker detail loss3528. Clipped-frame quality records are preserved, not asserted clean.

Central coherent host interval111851575->116724719us: runtime dispatch27.270%, deadline calculation21.654%, other14.451%, audio7.650%, GATT6.278%, security5.812%, AVRCP5.042%, I/O4.602%, product2.771%, submit2.718%, query1.752%, console0%. These are fractions of charged host cycles, with diagnostic/IRQ costs included. Corresponding steady task-runtime windows show host26.64/27.27% of Core1, renderer43.43/42.84%, controller11.91/12.45%, logger3.20%, idle10.59/10.08%. Core0 decoder52.61/51.49%, helper33.27/33.43%, idle14.14/15.06%. They do not prove sustained headroom or acoustic continuity.

## Deadline clock candidate

The hot deadline block read now_ms separately for12 consumers. ELF disassembly confirms now_ms calls esp_timer_get_time and the ROM signed64-bit division routine. The candidate takes one fresh time for all15 deadline queries, preserves every query and its return semantics, then subtracts time spent calculating from any finite nonzero wait. It clamps an elapsed deadline to zero and preserves the infinite sentinel. It adds no polling or delay, allocation, task, stack or persistent RAM. Relative to the valid diagnostic baseline, flash text+20 bytes; other measured sections unchanged.

Actual extracted aggregation passes ASan/UBSan with ccache for all15 queries, common time, earliest deadline, elapsed debit, due/overdue clamp, infinite/zero wait, restart limit and raw millisecond wrap. Both scheduler modes, task-cycle unit and admission tests pass. The entire wait-phase cost must not be attributed to clock conversion alone.

Raw sources, failed receipts, exact hook/clock disassembly, captures and analyses: native_bt/_build/experiments/host-charge-1794032927114. Diagnostic hooks/storage must be disabled again for the retained normal image. Full1000kbps, sustained LHDC continuity, complete codec/acoustic/control/coexistence/OTA qualification remain open.

## Candidate rejected; normal restoration

Same-diagnostic image39546528f3b30b3a40ec391d71f5d37d5aa300275ee42bc9ca2528c3a2e026db passes both guarded readbacks, owned VALID boot, original preservation and37 cold codec checks. Native YouTube playback12.275s under the independent14s guard uses the same restored UI192k/24/stereo/fixed900 and volume80. Host markers remain valid. Central wait charge per host iteration is444.10us versus445.93us before (-0.41%); task-runtime host load remains26.86/27.25% of Core1. This short pair does not establish a useful speedup.

Candidate source/rendered/DMA counts all2618432, initial shortage457728, recovered145152, unfilled312576 and descriptor faults0. The prior diagnostic had273408 unfilled. Different metronome segments/startup/tail windows do not prove causality, but there is no demonstrated continuity gain. The clock candidate and its temporary extracted test are archived and reverted. Original deadline behavior is retained; no optimization claim is made.

The corrected host phase attribution and opt-in separate host cycle accounting remain. A normal build without temporary task-switch hooks/storage is being restored. First-starvation baseline directly records1920 decoded input frames and0 processed output frames: samples are waiting for output preparation, not rejected because an encoded packet exceeded capacity. Full loss-free input and end-to-end sample identity are still separate qualification gates.

Normal image ea790bbaaa24f6d04d0e7e7c35c5c71b930d50cd568bcb24ac0393af661e15a6 builds with ccache and passes exact source/image admission with TaskCycleAccounting OFF. ELF symbols confirm switch hooks and host-account storage are absent. Compared with normal6e7afd71e8a0: flash text+72 bytes; IRAM, data, BSS and rodata unchanged. This is measurement-correctness work, not an audio-speed optimization. Guarded restoration passes full app/selector readbacks, owned VALID boot, original preservation and37 existing cold codec cases (archive runs/1794035670765910300-ea790bbaaa24). No watchdog/backtrace is recorded. The dashboard is resumed and listening. This image has boot/source evidence only; the paired LHDC playback evidence above belongs to the diagnostic images.
