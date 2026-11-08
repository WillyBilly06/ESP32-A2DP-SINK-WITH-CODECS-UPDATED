# Core0 DSP leases backed by processed PCM reserve

Development only: LHDC continuity and the full master goal remain open.
Current candidate image: e2a370485f2780fc959a23d733e4929a0ca90f210c3e2090d2761e3e8196259e. Previous image: de1ef69d17fbc4ad0eeeb56c7a4f683d8b4fcca2896fb02ac43839724f982167.

## Change and ownership

The existing lower-priority Core0 helper can now receive a DSP lease during a
codec media/resume call only while the output owner observes at least8ms of
published processed PCM. At192kHz this is1536 frames. Existing PCM-space and
encoded-input wait admission remains. PCM/reset/end transitions revoke the
new busy-decode hint; control, loan and renderer-handoff states do not grant it.
The phase is functional producer metadata, independent of optional profiling.
Atomic observations remain scheduling hints, not a cross-generation ownership
proof. Preparation, the single borrowed DSP job, completion/retirement and
lifecycle fences remain authoritative.

Controller/host/render priorities23/22/22 stay on Core1; decoder21 and helper20
stay on Core0. Decoder execution can preempt the helper. There is no new task,
audio buffer, allocation, wait, codec arithmetic, sample precision or quality
change. The8ms margin is not a worst-case scheduling guarantee. A separate
counter records successful reserved-job submissions, not simultaneous execution
of two tasks on one core.

## Source and device checks

- Actual admission/producer-hook ASan/UBSan tests pass8192 codec/state/reserve
  cases plus decode/reset/PCM/end revocation without a profiler.
- Actual two-core output/helper tests pass local progress below1536 frames,
  admission at1536 exactly, enabled DMA, a held borrow and queued stop with no
  early release. Existing cancellation, completion ordering, bounded six-block
  leases and sound ownership checks pass.
- The shared sound mock still assumed3x128 and is now derived from real DMA
  geometry. The separate BUSY-helper refill fixture now checks128+64 partial
  publications into a192-frame descriptor, unchanged DSP state/ownership and
  complete prefix order. Both100/1000Hz sanitized variants pass.
- Full ccache build,10413-source stability, image admission and ISR root/direct
  reference checks pass. Ccache performs explicit object compilation in the
  admission runner. Passing typedAMD64 COFF intermediates retire only after
  execution and a durable hash manifest; binaries/source/failures remain.
- Guarded COM6 write921600/full readback115200 match, original app preserved,
  owned VALID and no automatic Bluedroid fallback. Existing37 cold codec
  configurations/173 callbacks pass, tracked peak56899B,min free106740B,
  stack low-water3580/8192B. No new boot cases were added.
- Linked static RAM and IRAM are unchanged; the four-byte counter fits padding.
  Flash text grows96B and rodata32B. Adjacent running-I2S heap samples report
  free31152B,largest28672B,resident codec60704B; reported minimum since boot
  reaches28592B. These are adjacent observations, not an atomic heap/I2S read.

## Guarded native YouTube observations

Approved Redmagic9125299103B7; LHDC V5/192kHz/S24/stereo, fixed900kbps setting,
volume150, PCM5102A/no encoders. Each owned window is bounded with an independent
phone-side14-second pause guard. Totals include startup and postpause drain;
these are short sequential observations, not a reversed/randomized comparison
or1Mbps qualification.

| Mode / fixture | Source PCM frames | Helper share | Reserved submissions | Unfilled PCM frames |
| --- | ---: | ---: | ---: | ---: |
| Previous image, fresh Get Lucky control | 2539008 | 53.45% | 0 | 576 |
| Candidate, Get Lucky | 2525760 | 74.44% | 2130 | 2592 |
| Candidate,120BPM metronome | 2512832 | 74.18% | 1825 | 864 |
| Candidate, Get Lucky repeat | 2527936 | 74.73% | 1885 | 1120 |

The helper counter is since boot, so shares use before/after deltas. Render-stage
counters reset per stream and are not subtracted from a previous stream.
Core1 local DSP charge is445.61 cycles/source-frame in the control versus
238.09–256.28 in the candidate. This charge includes preemption and is not total
CPU utilization. The coherent published frame-stage sums/counts give2326.65us
mean in the music control and2281.95us in the candidate music repeat; this is not
an isolated decoder arithmetic speedup or a per-frame deadline guarantee.
Maximum observed helper queue/wall spans are3291/4334us; ownership failures0.

All accepted PCM reaches rendering and DMA. Descriptor/refill-contract errors,
encoded queue-full/decode rejection/reset/PCM failure and stream input-sequence
breaks are0. Normal PCM_BUSY is backpressure, not a dropped packet. Its detailed
retry trace overflows the existing worker diagnostic ring; the aggregate counter
and trace-loss count remain, so complete call-by-call evidence is not claimed.
Snapshot-record/UART loss stays0 and no corrupt checked record is observed.
The queue high-water remains7; the reserved8th slot was not newly used.

All three candidate first finalized gaps have empty raw and processed PCM while
a decode call is active. At those instants the active unit has not yielded PCM before
the DMA deadline. These snapshots do not show why the earlier reserve depleted
or where its packet was delayed. There are still gaps: this is retained as a
provisional development work-distribution improvement, not a continuity fix.

## Next transport review

Incoming Classic acceptance already requests role1; the speculative master-role
explanation is not supported by that policy. No role change was made.
The owned credit gate is stronger than the reference buffer-completion paths:
it waits on encoded-queue depth before returning pending HCI credits.
[BTstack's pinned HCI implementation](https://github.com/bluekitchen/btstack/blob/9edb5d82ea6ec4466a393415d65030929330acdf/src/hci.c)
accounts consumed ACL buffers independently of codec queue state. The supplied
Bluedroid hci_hal_h4.c similarly calls its completion helper before upper-layer
packet dispatch when C2H flow control is enabled. This is a concrete policy
difference, not proof of the remaining gap cause.

A next bounded experiment can replace all-or-nothing release with a conservative
partial credit limit, retaining the three-packet ingress window and one encoded
slot of safety. It must account for all other potentially outstanding controller
credits, conserve counts across failed sends/disconnect/generation changes, and
leave protocol/BLE replies responsive. The4096-byte HCI queue/520-byte control
reserve cannot safely advertise four full1029-byte packets. No window increase,
unconditional credit release or new RX pool has been made here.

Evidence: native_bt/_build/experiments/assist-reserve-1794123459631 and
native_bt/_build/hardware/runs/1794124508011785700-e2a370485f27. Phone paused and921600 UART resumed.
The full1Mbps/coexistence/long-duration/all-feature/security/OTA/acoustic and
comparative whole-host RAM gates remain unqualified.
