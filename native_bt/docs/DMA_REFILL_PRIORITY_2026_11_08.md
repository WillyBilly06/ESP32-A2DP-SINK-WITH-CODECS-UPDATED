# DMA refill priority and correlated owner evidence

The native renderer now services an open DMA refill before render bookkeeping.
Urgent DSP remains at 64 frames independently of the 192-frame descriptor size.
While real input and a valid refill deadline exist, Core1 may execute at most
two additional steps before yielding to equal-priority host work. Empty input,
stop, failure, a closed descriptor or deadline expiry ends that continuation.
The cap also holds when another EOF opens a new window. No busy-wait, extra
buffer, task, priority change or relaxed DMA guard is introduced.

The ordering fix and 64-frame cap change no linked section sizes relative to
the observer image. The bounded continuation adds 88 bytes of flash text and
no linked SRAM/IRAM. The render elapsed-time counter now includes the immediate
refill service; its scope must be considered when comparing old counters.

## Evidence that led to the fix

An observer image captured a finalized music gap with 864 raw PCM frames and
96 processed frames still available. The owner had just marked a render yield,
cached pending Classic/LE completions were both zero, the transmit fence was
clear, and a completion submission had returned from the SDK 1.716 ms earlier.
This does not identify a controller acknowledgment. The source inspection found
render timing/counter work before the still-open refill opportunity.

A deterministic virtual-clock regression reproduces the ordering defect in the
actual render_completed implementation. At the last safe boundary, one unrelated
timer read consumes the opportunity: the old code fails at fixture line 117;
copy-first succeeds with the original 32 us copy budget and 64 us guard.
The failed executable, exact old product/fixture sources, hashes, log and
losslessly archived symbols/intermediates are retained. The first corrected run
was held by the 512 MiB output gate before executing tests; old build logs were
archived with on-disk gzip/hash verification, then the complete test passed.
No hardware trace or failed receipt was removed.

ASan/UBSan checks pass at 100 and 1000 simulated ticks per second. They cover
initial/final immutable latches, copy order/expiry/cancel, exact split/full DSP
PCM with EQ/volume changes at 48 and 192 kHz, and the new boundary regression.
The burst helper also passes 3,000 continuously-ready decisions per tick mode,
with an independent two-extra-step bound and stop/failure/expiry/empty guards.
These tests use the actual production implementation with mocked peripherals;
they do not establish real RTOS scheduling or acoustic continuity.

## Separate owner and credit records

The new observer records owner intent, sampled pending completions, the last
Number Of Completed Packets SDK-return time/count, and the independent atomic
transmit gate. Fields are published by the host owner. A runtime mutation window
invalidates cached counts until republished. A one-attempt version check reports
unavailable state on interference; it never spins. Phase stamps precede operations
and are not RTOS task-state queries. Counts aggregate all Classic/LE links;
the SDK-return stamp does not prove controller processing. Timestamp zero/wrap,
malformed H4 records, full-width completion sums, nested phases and interrupted
snapshots pass a separate ASan/UBSan test. Its version-negative control fails
as intended. Actual output-latch tests prove later owner changes cannot rewrite
the captured gap.

The observer costs 120 bytes persistent SRAM and 392 bytes IRAM versus e34ab6.
It adds separate starvation_owner/credits and unfilled_owner/credits records to
the existing two immutable latches, without per-packet UART output. The linked
getter is in IRAM, has no calls or retries, and reads internal state and a stable
internal atomic gate. The existing limited ISR-root audit also passes.

## Target build and physical checks

Current COM6 image:
**d77107465293bf97948e99d3c092a6b305e2b05db1f132cc755468e0b10ef15e**.
The ccache build, compiler-input recheck, 10,417-entry source stability, image
admission and workspace gate pass. Rebuildable output is 511.863 MiB under the
unchanged 512 MiB gate. Scheduler tracing/runtime accounting remain disabled.
Complete readback matches after a 921600 upload. The original app is preserved,
owned selection is VALID, and automatic Bluedroid restore is disabled.

Existing 37 cold codec configurations / 173 callbacks pass: tracked peak
56,899 B, minimum free after allocation 106,500 B and decoder stack low-water
3,580 / 8,192 B. The selected LHDC decoder remains 60,704 B resident. After three
playback windows, stopped free/minimum/largest internal heap is
63,360 / 28,784 / 59,392 B; decoder/render/logger stack low waters are
5,912 / 3,772 / 1,404 B. These histories differ across boots and are not a
matched Bluedroid RAM comparison or sustained maximum-load qualification.

Approved Redmagic, native YouTube, LHDC V5 / 192 kHz / S24 / stereo, fixed
900 kbps UI selection, volume 150. Every owned excerpt has an independent
14-second phone pause guard and stays within 15 seconds of playback.

| Image / workload | Final unfilled PCM frames |
| --- | ---: |
| Fresh e34 music control | 1824 |
| Owner observer / metronome | 0 |
| Owner observer / music | 1536 |
| Copy-first, 64 frames / metronome | 2880 |
| Copy-first, 64 frames / music | 768 |
| Bounded refill burst / metronome | 768 |
| Bounded refill burst / music | 1152 |
| Bounded refill burst / music repeat | 192 |

The fresh e34 control's first gap is within one final packet, so pause-tail
classification remains separate. These short windows have different RF/runtime
histories; the table does not prove an isolated performance improvement. The
bounded policy's accepted/rendered/DMA frame totals agree after each stop.
All three current windows have zero queue/decode/reset/PCM timeout/sequence,
ingress, descriptor and refill-contract faults. Metadata status/lost and all
host/worker/snapshot loss counters are zero. All 8,015 ACL metadata entries
retire. Playback integrity counts are 293/283/298 Verified and 2/2/1 explicitly
Unverified legacy records; none is invalid. A fresh paused six-second capture
has 190 CRC-Verified records. UART/web reader remains identity-verified at
921600, PID 34852/source 97; phone is PAUSED and I2S stopped.

## Remaining failures

The first two current finalized gaps have empty raw/processed PCM while decode
is in progress. Music has known-zero sampled pending completions and a clear
transmit gate; metronome catches the runtime mutation window, so its pending
count is explicitly unknown. The repeat still has 832 raw frames and no
processed PCM at its finalized gap. Therefore both delivery/decode timing and
consumer completion remain open. Earlier copy-first music also captures empty
PCM with an owner wait marker, known-zero pending completions and a clear gate.
These events must not be collapsed into one cause or labeled packet drops.

The deterministic ordering bug is fixed. LHDC continuity, 1 Mbps, sustainable
headroom with BLE/control traffic, acoustic parity, all feature/security/OTA
coverage and comparative RAM remain unqualified. This is development firmware;
main and stable releases are not promoted.

Reproduce host checks:

~~~powershell
python native_bt/tools/test_owner_state.py --out native_bt/_build/owner_state
python native_bt/tools/test_probe_output_format.py --starvation-trace --light-gap --owner-state
~~~

Evidence roots:
- native_bt/_build/experiments/gap-owner-1794132194837/
- native_bt/_build/experiments/refill-first-1794133293753/
- native_bt/_build/experiments/refill-burst-1794134072906/
- native_bt/_build/hardware/runs/1794134398654422200-d77107465293/traces/
