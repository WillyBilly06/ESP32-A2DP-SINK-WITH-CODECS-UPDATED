# LHDC buffered duration and UART checkpoint

The host now counts LHDC encoded backlog in PCM sample frames and includes it
in negotiated AVDTP delay estimates. This corrects omitted queue duration; it is
**not an underrun fix**. Later fixed-900 playback still misses output deadlines,
including a confirmed gap while decoded PCM is waiting for DSP. The custom host
remains development firmware; sustained 1 Mbps, BLE/control headroom and complete
feature/security/OTA qualification remain open.

## Duration ownership and units

One RTP packet is not necessarily one codec frame. Redmagic fixed-900 packets
advance 960 RTP ticks / 5 ms at192kHz; observed Auto packets advance1920 ticks /
10 ms and contain two5ms codec intervals. Counting every packet as5ms would
underestimate Auto backlog. These durations are milliseconds, not microseconds.

A startup-only bounded media-duration callback uses the negotiated host-owned
configuration and the same structural LHDC length parser as decoding. It never
calls the decoder, allocates, waits, or inspects a decoder-owned PCM pointer.
Only admitted packets add duration. Existing copied-PCM delivery counters retire
sample frames as each partial transfer is accepted, including256-frame chunks.
A full queue or BUSY transfer is not counted as successful delivery. Unknown
framing, decode rejection, stopped/replaced epochs and unsupported loan/legacy
modes produce an explicitly unavailable estimate.

The observer makes one atomic snapshot attempt, rejects producer interference,
and never retries or touches live queue slots. No per-packet metadata, queue,
scratch buffer, task or allocation is added. Linked BSS increases **16 B**;
DRAM data and IRAM are unchanged. Flash text/rodata increase1032/384B relative
to the IRQ-dispatch checkpoint. Buffer capacities and audio format are unchanged.

Encoded samples are added to PCM/DMA/resampler duration **before** the priming
minimum is applied. The actual tested example is70ms encoded plus18.08ms output:
88.1ms reported, rather than omitting encoded audio or adding a second minimum.
Saturating wide intermediate arithmetic handles high rates and overflow bounds.

The peer-facing path runs only when delay reporting is negotiated. Redmagic's
observed LHDC configuration has **delay_reporting:false**, despite the endpoint
advertising the capability. Normal builds therefore skip unused per-packet
parsing after its first unavailable result. Detailed scheduler tracing can opt
into independent duration observation. Other codecs retain their prior PCM-only
estimate. Positive on-wire peer acceptance of the new estimate is **not tested**;
host delay-signaling tests do not substitute for that device evidence.

Immutable host records use existing trace storage; the diagnostic send result is
an API result, not a peer ACK. Known queue samples on the instrumented Auto run
are6720/3840/10560/8640/8640/8640/6912 frames, or20–55ms at192kHz. Non-packet
multiples reflect PCM already copied downstream. Normal logging suppresses
repeated unchanged unavailable-duration rows.

## Why the observed Auto rate is about400kbps

The user's C99 encoder initializes AUTO at the LOW/400kbps stage. The published
[reference encoder policy](https://github.com/sprlightning/liblhdc-collections/blob/09a74e701ff81840e2648122e3abd4f82dbf5ce4/AOSP/liblhdcv5/src/lhdcv5BT_enc.c)
also defaults ABR to400kbps; its192kHz adjustment table ends at400kbps and its
higher VBR promotion is gated to48kHz. The inspected75213-byte file is bound to
Git blob e22b683c778ecbc9ec32a6bc7a36d52b0dbd1440. This is consistent with the
captured Auto404.8kbps AVDTP payload rate, including framing. It does **not**
identify the proprietary OEM phone binary or prove all encoders use that policy.
The observed fixed-900 payload rate is905.6kbps, also including framing. Neither
mode qualifies the requested1Mbps target.

## Host verification and repaired test harnesses

- Portable protocol/worker suite: **188/188**.
- Real deferred codec comparisons: **80/80**, every PCM byte and timestamp,
  including37 configurations and the maximum LHDC fixture under ASan/UBSan.
- Actual LHDC structural parser: **1528** grouped/header/length/truncation cases.
- Actual delay arithmetic: **960** state/rate/minimum/overflow cases.
- Actual worker duration fixture: grouped packets, partial PCM acceptance,
  queue-full/BUSY, restart, rejection, wrap, changed-producer rejection,
  unsupported modes and allocation symmetry. One-packet/one-frame and omitted-
  encoded negative controls both fail as intended.
- Actual output/refill tests pass at100/1000 simulatedHz, with exact full/split
  DSP PCM at48/192kHz, EQ/volume changes, startup, cancellation and wrap.
- Normal and detailed host-state trace formatters pass bounded storage,
  blocked-consumer behavior, loss accounting and typed delay-record checks.
- Six app-worker variants pass: four legacy/default combinations plus copied
  and loaned asynchronous output. RTOS/output providers are mocked; these are
  not physical scheduling or source-role tests.

The old trace extractor included unrelated functions and its PCM-busy expectation
predated normal-mode suppression. The old app fixture also had stale provider
interfaces, wrong raw-aptX test input framing, and omitted source/discovery link
dependencies. The replacements use typed inert source-role mocks with an asserted
zero-call count. These mock/test files are absent from MCU compile commands.
Legacy default logger affinity is retained; the actual image explicitly selects
Core1. Test priority6 is a legacy fixture value, not the board's decoder21.

The legacy app wrapper initially compiled directly; that attempt did not satisfy
the requested ccache workflow. It now compiles each object through ccache with
explicit -c, linking separately. The final300 object commands across six variants
all use ccache. Each run has its own directory, initially failing receipt,
source archive/hash, commands and final source check. Passed object retirement
is hashed and PDB archives are verified. Earlier failed logs/source snapshots
remain; one historical DLL was overwritten by the old fixed-name wrapper before
the immutable-run repair, so its failed executable is unavailable.

## Exact deployed image and physical results

COM6 image:
**9bced750ed38fb3c74ab3f24d77f208d5b06451273f8136388cbbd00a895041f**.
Full ccache build, compiler-input recheck, image admission and the limited ISR
root/dependency audit pass. There are10419 stable source/build inputs and
500.337MiB rebuildable output under the unchanged512MiB gate. The ISR audit
is not a complete callgraph or cache-off OTA proof.

Upload uses921600baud, verification115200baud; all written-region readbacks
match. The original app is preserved, owned selection is VALID and automatic
Bluedroid restore is false. Existing37 cold configurations /173 callbacks pass.
Cold tracked peak is56899B, minimum free after allocation106160B and decoder
stack low3612/8192B; these are not live-stream or matched Bluedroid figures.
No boot watchdog, backtrace or idle-starvation marker is observed.

Approved Redmagic/native YouTube only,192kHz/S24/stereo. Every excerpt uses an
independent14-second phone pause guard. Counts below include the complete
stopped trace and tail; no shortage is subtracted because it is near pause.
The observable image has duration tracking enabled even without negotiation;
the final normal image gates that work off for this peer.

| Image / mode / music excerpt | Initial shortage | Recovered | Final unfilled |
| --- | ---: | ---: | ---: |
| Instrumented fc570271 / Auto, requested10s | 0 | 0 | 0 |
| Instrumented fc570271 / fixed900, requested10s | 6848 | 4160 | 2688 |
| Instrumented fc570271 / fixed900, requested11.5s | 1920 | 1344 | 576 |
| Normal9bced750 / aborted freshness gate | 960 | 576 | 384 |
| Normal9bced750 / bounded logger, requested11.5s | 16448 | 7616 | 8832 |

The first fixed900 instrumented miss is bounded394.9–722.9ms after the pause
request; that older harness has no pause-ACK timestamp. Its repeat has a first
miss8.61–8.86s **before** pause with832 raw/0 processed frames waiting. Only the
first gap is latched; this does not place every576 frame in the active interval.

The first final-image attempt aborted after about3.8s because its logger
freshness gate incorrectly required the low-priority logger to report within
3s throughout playback. Its failed receipt/raw records and a separately saved
follow-up remain. UART resumes and the phone is paused safely. It is not an
11.5-second completed test.

The corrected bounded capture checks identity throughout and freshness before/
after playback while recording logger age as a diagnostic. Its maximum observed
UART age is1442.6ms. The first finalized miss is10.32–10.57s before pause, with
640 raw and256 processed frames in the latch. Thus an active-playback deadline
miss remains independently of any tail gaps. It does not prove all8832frames
occurred while active or that this accounting change caused the variation.

On that final run, queue-full, decode/reset/PCM-failure/timeout, RTP-sequence,
ingress, descriptor/refill-contract, helper and log-loss counters are zero.
Accepted/rendered/DMA-source frames agree after stop. Expected PCM backpressure
remains counted separately. No continuity improvement is claimed from this stage.
The next measured hypothesis is the processed-reserve condition for lower-
priority Core0 DSP assistance; no admission/priority policy is changed here.

## UART logging and final handoff

Firmware UART0 and the raw web monitor run at **921600baud**. The console uses
interrupt-backed driver writes. Snapshot formatting/UART work belongs to the
priority1 background logger on Core1; it may sleep for UART space. Audio/host
trace producers only publish bounded metadata, with explicit full/loss counters.
Normal mode counts expected PCM backpressure without per-event printing. Heavy
scheduler tracing is off. Necessary cumulative pipeline diagnostics remain at
one-second cadence; detailed faults/first-gap records remain separate.

Logging is not one CPU cycle. At8N1,921600baud is at most92160bytes/s; a1024-byte
record occupies about11.1ms on the wire. Background priority prevents UART waits
from becoming waits inside the audio callback, but formatting/interrupts still
have a cost and the logger can be delayed by busy higher-priority tasks.

A fresh6s passive window has **195 CRC-Verified records, zero Invalid/Unverified**,
and75465 UART bytes. Monitor PID34852/source113 remains identity-verified at
921600. Phone YouTube is PAUSED and I2S stopped. Final stopped internal heap
free/minimum/largest is63020/29372/59392B; decoder/render/logger stack low waters
are5912/3772/1388B. These describe this finite history, not leak/stress qualification.

Evidence is retained locally under:

- native_bt/_build/experiments/lhdc-delay-accounting-1794144863071/
- native_bt/_build/hardware/runs/1794147193283712600-fc570271bc6e/traces/
- native_bt/_build/hardware/runs/1794151077945381100-9bced750ed38/traces/

Development branch only. Main/stable releases are not promoted by this checkpoint.
