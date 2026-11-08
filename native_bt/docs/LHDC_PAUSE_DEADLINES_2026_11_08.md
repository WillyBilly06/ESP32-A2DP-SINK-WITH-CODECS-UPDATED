# LHDC pause boundaries and rejected urgent DSP dispatch

The urgent-idle helper candidate was rejected. Its three short physical runs
show no consistent continuity benefit, so its product changes and extra counter
are removed. The retained renderer/decoder code is byte-for-byte the pre-trial
source. The rebuilt image differs from ccc991544951 only in application build
time, embedded ELF SHA256, and image checksum/digest; all other image bytes match.
This is development firmware; stable main/releases are not changed.

## A confirmed playback-time deadline miss

The pre-trial image ccc99154495138cba3e5ff6c572796820dbeb430a64b8feb5e1f2151532a60d6
was tested on approved Redmagic with native YouTube Get Lucky from 30 seconds,
LHDC V5/192 kHz/S24/stereo/fixed 900 kbps, media volume150. The late-pause run's
first finalized shortage is 9.549908 seconds after I2S enable. It predates the
pause request by at least2.027 seconds under the causal bounds below.

At device time 1616741426 us, the descriptor has 128 of 192 frames. There are
768 raw PCM frames and 64 separately published processed frames. The guarded
copy deadline was1616741362 us. The render phase ended at 1616741365 us,
three microseconds beyond that cutoff. The decoder had begun waiting for encoded
input200 us before closure. The host's last marker is a render-yield intent8 us
before closure; markers are not RTOS running-state evidence. Sampled Classic/LE
pending completions are both zero, sampled2.818 ms earlier; the last completion
submission returned from the SDK3.290 ms earlier. This is not a controller ACK.
The final total is 1024 unfilled PCM frames, not1024 dropped codec packets.

PCM-frame counts describe stereo sample instants. At 192 kHz, 4,928 unfilled
frames represent about 25.67 ms cumulatively; a 960-sample LHDC codec frame
represents 5 ms. This count alone does not identify decoder overload.

This proves a consumer deadline miss with decoded data available. It does not
prove every gap has this cause: earlier captures also contain empty raw/processed
PCM and a decoder waiting for late input. Those cases remain separate.

## Pause timing without assuming USB latency

Every compared trace starts with YouTube PAUSED and I2S stopped, and has a new
verified startup-enable timestamp. No other media/control workload is introduced.
For a latched event E and enable timestamp S, a lower host-time bound is:

    fixture_open_request_time + unsigned32(E - S) / 1000000

Output cannot have enabled before that action under those preconditions. An
upper bound is the smallest of the following over CRC-verified UART records:

    UART_receive_time + signed32(E - record_at_us) / 1000000

Formatting the record precedes receiving it; UART scheduling/transport delay is
nonnegative. No maximum UART delay or synchronized PC/ESP clock is assumed.
Bounds require continuous PC wall time within a short run, matching UART source,
a fresh stream, and half-range-safe device timestamp distances. All observed
intervals are nonempty. A shortage is before pause only when its upper bound
precedes the request. It is after pause acknowledgment only when its lower bound
exceeds the recorded acknowledgment. Old traces without acknowledgment keep that
limitation. Post-pause shortages remain real counters and are not automatically
excused as expected behavior.

## Controlled physical observations

The earlier 6-second pause and later 11.5-second pause use the same pre-trial
image and music fixture. The candidate uses the same settings; its metronome
fixture is native YouTube xpiVrEET-YQ from 1 second. Each excerpt has an independent
14-second phone pause guard; actual I2S EOF durations stay below15 seconds.
Fixed900 kbps is the phone selection, not qualification at the goal's 1 Mbps.

| Image/workload | Requested play | Final unfilled frames | First finalized gap relative to pause request | Urgent helper leases |
| --- | ---: | ---: | --- | ---: |
| Pre-trial music |6s|3456|+0.929 to+1.219s; after acknowledgment|n/a|
| Pre-trial music |11.5s|1024|-2.247 to-2.027s; before request|n/a|
| Candidate music |11.5s|320|+1.534 to+1.822s; after acknowledgment|3|
| Candidate music repeat |11.5s|1472|-0.470 to-0.164s; before request|2|
| Candidate metronome |11.5s|1536|+1.530 to+1.775s; after acknowledgment|5|

Totals include the entire stopped trace and are never reduced by pause
classification. These sequential excerpts have different RF/runtime histories;
there is no isolated speedup claim. The repeat still misses a deadline during
playback with 960 raw PCM frames and no processed output. Candidate runs have
zero queue/decode/reset/PCM-timeout/sequence, ingress, descriptor/refill-contract,
helper, quality-concealment/nonfinite/clipping, and log-loss faults. Their UART
integrity totals are255/217/256 Verified,1/1/2 explicitly Unverified legacy
records, and zero Invalid. All accepted/rendered/DMA-source frame totals agree
at stop. This is digital/timing evidence, not acoustic qualification.

## Rejected change and host checks

The candidate allowed a single urgent helper lease of at most64 frames only
when existing decoder admission reported idle. Busy decoding still required the
existing 8 ms processed reserve for a nonurgent lease. Borrowed-state lifetime,
priorities, task count, DMA geometry, audio buffers, PCM precision and DSP math
were unchanged. The linked cost was8 B DRAM data,76 B flash text,16 B flash rodata,
and zero IRAM/BSS increase. The rejection removes that cost.

Actual extracted submission and render-dispatch code passed 12,320 ASan/UBSan
policy/resource cases. Restoring the previous urgent-dispatch exclusion as a
negative control failed at fixture line 74, case 10,034. Actual two-core mocked
output/DSP/sound lifecycle checks passed with six-block assistance, early input
release, high-rate rebalance and fair share.8,192 actual codec/reason/reserve
admission cases passed. These host checks do not measure ESP32 scheduling.
The candidate source, test/executables, failure receipt and all raw traces are
retained under the evidence roots. They are not current production source.

## Clock check and next diagnosis

The pre-trial first-shortage counters imply roughly192001.3-192001.7 source
frames/s over 10.5-13.2 seconds, with zero descriptor recovery/error and maximum
EOF gaps of 1052-1073 us. A 2.5-second older sample implies 192007.2. This is a
rough check: the startup enable timestamp is sampled after enable returns,
so fixed startup bias affects the rate. It is not independent BCLK measurement
or proof of phone clock accuracy. It does not support a gross I2S rate error
large enough to explain millisecond gaps; automatic retuning/FIR/drop-insert
was not enabled.

The next target is wake/dispatch/preemption latency on Core 1. In the candidate
repeat, the render phase begins833 us after the initial short EOF, leaving
about104 us before the guarded cutoff; the decoder becomes idle10 us after that
render phase begins. At the finalized gap, the helper is idle. Moving a few
late blocks between cores does not establish that this scheduling delay is
fixed. Actual task-state/notification evidence is needed before attributing it
to equal-priority scheduling or controller execution.

LHDC continuity, sustainable1 Mbps headroom with BLE/control traffic, acoustic
parity, comparative RAM, full security/feature/OTA/stress coverage remain open.

## Evidence roots

- native_bt/_build/experiments/lhdc-pause-boundary-1794140117515/
- native_bt/_build/experiments/urgent-idle-dsp-1794140332548/
- native_bt/_build/hardware/runs/1794140561286648900-eca9fda58ca5/traces/

Restoration identity/readback and final passive log checks are recorded below.

## Restored device state

COM6 now runs **bbff77712450f0412c45c7a3e030afeaa18bcc8f35acdeadae8401a8a25669c6** at **921600 baud**.
Full readbacks match after a 921600 upload. The original application is preserved,
owned selection is VALID, and automatic Bluedroid fallback is false. The existing
37 cold configurations/173 callbacks pass, with 56,899 B tracked peak, 106,500 B
minimum free after allocation, and 3,580/8,192 B decoder stack low-water. No new boot
selftests were added. A fresh six-second passive capture has 179 CRC-Verified
records, no Invalid/Unverified records, and source 105 on monitor PID 34852.
Native YouTube is PAUSED; I2S is stopped. Restored source/build admission passes.

The first restoration build was correctly failed by the unchanged 512 MiB
rebuildable-output gate at 512.660 MiB. Completed PDB/build logs were losslessly
gzip archived with on-disk decompression/SHA checks and manifests before original
files were retired. Failed receipts, executables and raw hardware traces remain.
The complete ccache rebuild then passed at 465.880 MiB; no quota was raised.
Restoration archive: native_bt/_build/hardware/runs/1794141413617965300-bbff77712450/
