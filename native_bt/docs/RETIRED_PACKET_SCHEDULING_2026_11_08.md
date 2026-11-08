# Retired-packet owner scheduling and receive-credit ordering

The native host now offers a conditional renderer yield point after each healthy
HCI packet has fully retired, with one nonblocking receive-credit poll before
that point. This reduces the work that can intervene between scheduling checks;
it does **not** establish uninterrupted LHDC playback. Corrected short music
windows have256/704 finalized unfilled frames and metronome3456. The second music
and metronome captures have confirmed playback-time misses.

## Ownership and protocol behavior

The previous native path offered its internal owner checkpoint only after a
batch of up to eight HCI packets. An equal-priority renderer could become ready
while the host continued that batch. The actual compiled ESP-IDF kernel uses a
strictly higher-priority condition for notification-triggered rescheduling.
Observed host-poll spans exceed the1ms DMA period, but include preemption and
adjacent profile work; they are not isolated RX-batch CPU timings.

New optional host/runtime poll entries take an invocation-local scheduling hook.
After each healthy packet, its protocol/observer callbacks have returned and
sensitive scratch has been wiped. Host/runtime poll fences remain set. The hook
may yield/enqueue work and must not wait, reinitialize or retain packet views.
A faulting packet does not invoke a healthy-packet checkpoint. A hook-triggered
abort stops further RX and unwinds through existing cleanup.

The runtime first gives consumed RX credits one bounded nonblocking flow poll.
Existing downstream gates, per-generation accounting, controller/transport
availability and retained transmit storage remain authoritative. SDK acceptance
of an NCP command is not a controller acknowledgment. A full/held transport never
causes a retry loop or buffer rewrite. Normal command pumping still follows the
bounded RX batch. Original poll APIs and the legacy post-batch hook keep their
prior behavior. The native fair-sharing path uses the new entry with the existing
LHDC/output-pressure predicate; other callers retain the original entry.

There are no new persistent fields, allocations, tasks, queues, packet copies,
buffer capacities, codec restrictions or audio-format changes. Linked DRAM
BSS/data and IRAM are unchanged; flash text grows136B relative to129961a1's
image. The scheduling context is three transient pointers. Controller23/Core1,
host22/Core1, renderer22/Core1, decoder21/Core0 and helper20/Core0 are unchanged.
The8ms processed-reserve gate for decoding-time Core0 assistance is unchanged.

## Failure retained, then ordering corrected

The first candidate2d616f6739bf yielded before the runtime's post-batch credit
poll. Its music windows ended960/13632 unfilled frames. The repeat's first gap
is before pause with empty raw/processed PCM and roughly44ms since the last
admitted media packet. This does not identify an RF or controller-credit cause:
the sampled pending-count observation is unavailable at that gap. These results
are retained and are not presented as a successful optimization.

The corrected implementation polls consumed credits before its owner handoff.
The tests now require that ordering, while preserving explicit credit gates and
transport lifetime. No claim is made that credit ordering alone caused the
physical count differences below. These sequential runs have different runtime/
RF histories; there is no reverse-control or isolated causal speedup proof.

## Checks and exact deployed image

- **191/191 portable tests pass**, including three new runtime tests for packet
  order/budgets, sensitive wipe before yield, recursive poll fences, abort/fault
  cleanup, credit-before-yield, closed gates and immutable retained NCP storage.
- Focused actual-host/runtime ASan/UBSan checks pass. Removing the packet point
  fails the new fixture; independently omitting the pre-yield credit poll fails
  at fixture line1181. Failed binaries/sources/receipts are retained.
- Every C object compilation in these runs uses ccache with explicit -c; links
  run separately. Host mocks do not qualify real scheduling, RF or acoustics.
- Complete firmware build/rechecked10419 inputs, image admission and the limited
  ISR dependency audit pass. The audit is not a complete ISR callgraph or
  cache-off OTA-continuity proof. Rebuildable output is510.706MiB under512MiB.

A first rebuild stopped before compiling at529.625MiB. Completed symbols/logs/
viewer snapshots and passed candidate executables were losslessly gzip archived
with on-disk decompression/SHA checks, saving20892994bytes. Passed AMD64 objects
were retired with hashes. The disk quota was not raised; failed executables stay
available. The old baseline ELF is preserved as its verified gzip archive.

Current COM6 image is
**e7948405b95b38723f2602d68ea2449d9314b5cafa2262500f67e0dcf92341fb**.
Upload921600/readback115200; all written regions match. The original app is
preserved, owned selection is VALID and automatic Bluedroid fallback is false.
Existing37 cold codec configurations/173callbacks pass, tracked peak56899B,
minimum free after allocation106160B and decoder stack low3468/8192B. No boot
watchdog, backtrace or idle-starvation marker is observed. These cold tests do
not qualify live codec operation or full product parity.

## Native YouTube results

Approved Redmagic only, LHDC V5/192kHz/S24/stereo/fixed900, phone media150/150.
Matching valid DSP snapshots show gains11/8/12dB, DSP volume127 and flags12.
The split/crossover bypass flag does not bypass the four common filters. Each
request plays10s with an independent14s phone pause guard; observed I2S spans
remain13.153–13.374s, below the15s excerpt limit. Final counters include the tail.
Only the first gap is latched; pause classification cannot place every later
unfilled frame. No count is subtracted for pause.

| Image/workload | Initial shortage | Recovered | Final unfilled | First finalized gap |
| --- | ---: | ---: | ---: | --- |
| Baseline9bced750/music1 |768|320|448|after pause ACK|
| Baseline9bced750/music2 |9472|7040|2432|7.02–7.41s before pause|
| First candidate2d616f67/music1 |3008|2048|960|pause boundary, ACK not separable|
| First candidate2d616f67/music2 |27008|13376|13632|6.77–7.02s before pause|
| Corrected e7948405/music1 |1152|896|256|after pause ACK|
| Corrected e7948405/music2 |2624|1920|704|3.16–3.48s before pause|
| Corrected e7948405/metronome |7040|3584|3456|7.97–8.23s before pause|

The corrected metronome first shortage is at251341256us with256 raw/0 processed
frames. Its finalized window closes at251342257us: only64 of192missing frames
were recovered, with832 raw/0 processed frames waiting. The last render-phase
begin marker is251342031us,775us after that initial EOF; the guarded deadline is
251342192us. These are phase/intent markers, not proof of actual running-task
identity or pure DSP execution time. Helper state is IDLE and the decoder began
waiting for encoded input194us before closure. A consumer deadline miss with
decoded PCM available therefore remains. Another corrected music gap instead
has empty raw/processed input; the causes must remain separate.

All seven windows have zero queue-full/decode rejects and host/worker/UART log
losses. Accepted/rendered/DMA-source totals agree after each stop. Integrity is
293/331/314/317/327/336/329 Verified with1/1/1/1/1/1/3 explicitly Unverified legacy
records, zero Invalid. These finite digital observations do not qualify acoustic
quality, sustained1Mbps operation, BLE/control headroom or full feature/security/
OTA/stress parity. No performance percentage is claimed from these excerpts.

Final6s passive capture:190 CRC-Verified, zero Invalid/Unverified, monitor source117
at921600. Native YouTube is PAUSED; I2S stopped. Stopped heap free/minimum/largest
is63020/27576/59392B; decoder/render/logger stack low waters5912/3820/1388B.
This describes a finite allocation history, not a leak/stress qualification.

Evidence roots:

- native_bt/_build/experiments/packet-owner-checkpoint-1794152372666/
- native_bt/_build/experiments/packet-owner-checkpoint-1794152372666/credit-order/
- native_bt/_build/hardware/runs/1794152719814690200-2d616f6739bf/
- native_bt/_build/hardware/runs/1794153448845593400-e7948405b95b/

The next diagnosis needs actual render/helper wake, publication and dispatch
latency around the remaining late Core1 work, rather than inferring the running
task from phase markers or changing the helper reserve without measurements.
Main/stable releases remain unchanged; this checkpoint is development firmware.
