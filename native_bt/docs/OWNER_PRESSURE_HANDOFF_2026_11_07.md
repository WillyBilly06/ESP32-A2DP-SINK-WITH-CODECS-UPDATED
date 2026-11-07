# Conditional host handoff experiment — rejected

The handoff prototype is removed. COM6 is restored to exact image
`9283f7559e3333c7af43d5fe832060a813bfd2eb0a8595c4a53c3b9425101f2e`,
retaining the measured compact-FPU decoder improvement. LHDC underruns remain.

## Contract and prototype

The installed kernel does not request an immediate context switch for an
equal-priority task notification. The inspected remote-yield primitive is
explicitly private (`portmacro.h:316`, `esp_private/crosscore_int.h:31–32`), so
application code did not call it. The public scheduling behavior is described
in the [IDF FreeRTOS guide](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/api-reference/system/freertos_idf.html#smp-scheduler).

The prototype instead used a zero-time notification take and ordinary
`taskYIELD()` after a bounded host batch. A consumed pending hint always led
to another protocol poll before any sleep. The handoff required actual local
render input, a matching completed helper job, or published PCM for a pending
DMA copy. A borrowed helper with no consumable output did not qualify. The
nonsharing/pacer policy and all priorities remained unchanged.

A separate `pressure_handoffs` counter kept these handoffs distinct from
existing immediately-runnable yields. Actual ELF changes were flash text
+368 bytes and rodata +16 bytes, with no net data/BSS or instruction-RAM growth.
No task, buffer, codec, security rule or DSP arithmetic changed.

## Host verification

The actual extracted host tail, pressure wrapper/predicate, counter functions
and log format passed ASan/UBSan in sharing and nonsharing modes. The old tail
fails the intended negative control: a positive-timeout take consumes its
pending hint and returns without a voluntary renderer slot. The prototype
passes, including a new hint arriving during the yield, no-pending normal
sleep, zero-wait compatibility, stream/stop/failure fences, counter wrap,
helper completion and pending-DMA work above the usual reserve.

Existing two-core DSP/sound ownership tests and header/CMake/image fairness
checks pass. A separate old reserve fixture failed to compile because it lacked
the existing `actual_target_frames` field. That fixture is repaired and now
checks its target-cap boundary as well; production decoder admission did not
change. This small test repair is the only retained code change from this
experiment. Initial fixture failures and corrected negative controls remain
archived rather than being counted as passing tests.

Full ccache build, source/image admission, guarded 921600 upload/full 115200
readback and all 37 cold codec cases pass for the prototype. These checks do
not establish audio performance.

## Physical comparison

Approved Redmagic, native YouTube 120 BPM metronome, LHDC 192 kHz/stereo/S24,
fixed 900 setting, phone 90/150 and unchanged EQ 11/8/12. Each trace has an
11-second command interval and an independent 14-second pause guard.

| Run | Source PCM frames | Unfilled frames | New handoffs |
| --- | ---: | ---: | ---: |
| Original 9283 baseline | 2738688 | 15104 | not present |
| Prototype | 2696640 | 20224 | 172 |
| Prototype repeat | 2723520 | 9216 | 106 |
| Restored exact 9283 | 2732160 | 12416 | not present |

The path executes, but its continuity result is mixed. The reverse comparison
does not establish a repeatable benefit or causal degradation. No protocol,
descriptor or refill-contract failure was observed. First gaps still show
decoded input waiting for processed output; no continuity fix is claimed.

## Active CPU evidence

Approximately five-second active task-runtime intervals show:

| Core 1 role | Charged runtime |
| --- | ---: |
| Host, priority 22 | 29–32% |
| Renderer, priority 22 | 40–43% |
| Controller, priority 23 | about 13% |
| Logger | about 3% |
| Idle | 7–9% |

Core 0 remains approximately 20–23% idle while decoding and helper DSP run.
These are active interval differences, not whole-boot idle averages. Interrupt
time is included in the charged task and the current running tail is excluded;
the numbers are not pure application-instruction cost.

Tick-phase samples put runtime polling at roughly 24–31% and deadline
calculation at 19–22% of sampled host ticks. They are not exact per-call times
and may contain sampling bias. The actual phase observer is small atomic
bookkeeping, and real compile commands enable local block DSP, unity PCM copy
and word quantization. A missing block-DSP definition is therefore ruled out.
Availability-wrapper disassembly has no visible wait loop; this alone does not
measure its full cost or establish a controller bottleneck.

The next useful measurement is bounded per-query/poll timing, followed by a
work-ready/quiescent fast-path audit. Preserve queued completions, deadlines,
security/ATT replies and all BLE/product behavior. Do not attribute the cost
to a particular bounded loop or change priorities without stronger evidence.

## Restoration and evidence

All four production files are restored byte-for-byte; the rebuilt/flashed
image is byte-identical to 9283. Written-region readbacks match, owned boot is
VALID, the original application is preserved and automatic Bluedroid fallback
is false. All 37 restored cold cases pass: 173 callbacks, tracked peak 56899 bytes,
minimum free after allocation 102996 bytes and stack low-water 3472/8192 bytes.
The phone is paused at 90/150 with 192k/S24/fixed 900 restored; raw UART logging
is listening.

Local archive:
`native_bt/_build/experiments/owner-pressure-handoff-1794075555279/`.
It retains prototype/baseline source, generated positive/negative C fixtures,
host results, raw traces, interval analyses and all deployment receipts.
The archived Python test expects its original `native_bt/tools` location;
replay the saved C fixture/command or restore prototype files into an isolated
checkout. Failed storage gates remain recorded. Only typed completed host
intermediates, reproducible passing tone inputs and hash-identical copies of
the installed compiler runtime were reclaimed. Tested programs, generators,
source/results, Xtensa objects and all raw physical evidence remain preserved.

Restored physical archive:
`native_bt/_build/hardware/runs/1794078186890866000-9283f7559e33/`.
[Compact results](OWNER_PRESSURE_HANDOFF_RESULTS_2026_11_07.json) preserve image
binding, source restoration, CPU intervals and the full comparison. The entire
master goal remains open, including uninterrupted LHDC, 1 Mbps, sustained
coexistence and complete feature/security/OTA/lifecycle/stress/acoustic gates.
