# IRQ refill dispatch at equal task priority

The native I2S callback now requests a scheduler decision when useful refill
work is ready on Core 1 and the renderer is not already the interrupted task.
The host and renderer retain priority 22; the controller retains priority 23.
Decoder/helper priorities, core affinity, codec support, audio precision,
buffers, DMA geometry and the guarded refill contract are unchanged.

Three fixed-900 and two Auto-bitrate LHDC excerpts end with **zero finalized
unfilled PCM frames**. This is finite digital/timing evidence. It does not
qualify sustained 1 Mbps operation, simultaneous BLE/control load, acoustic
quality, or full feature/security/OTA parity.

## Why the existing wake flag was insufficient

The actual ESP-IDF FreeRTOS kernel calls taskIS_YIELD_REQUIRED with equal-priority
yield disabled in vTaskGenericNotifyGiveFromISR. On SMP, the comparison is
strictly greater than the current task priority. Thus waking a priority-22
renderer while the priority-22 host runs does not itself request immediate ISR
preemption. Earlier traces captured a consumer deadline miss with decoded PCM
waiting; another captured a render phase starting 833 us after a short EOF,
leaving only about 104 us before the copy cutoff. Those phase markers do not
identify the actual interrupted task.

The original ESP32 I2S driver propagates the application callback result to
portYIELD_FROM_ISR. The correction uses that existing mechanism. It asks for
scheduling only if the existing wake flag is false, the refill index is valid,
output is healthy and not stopping, IRQ/renderer are on Core 1, raw or processed
PCM is available, another task is current, and more than the existing 32 us
copy budget remains. It never increases task priority. The existing bounded
render burst and host-yield rules still apply. No busy-wait, new notification,
queue, task, or allocation is added.

A separate dispatch_requests counter records requests, not actual switches.
Formatting and UART output remain in the priority-1 background logger at
921600 baud; this counter does not print per IRQ.

## Public getter and IRAM dependency

The current-task identity comes from the public xTaskGetCurrentTaskHandle API.
Its existing linker fragment now also applies to native audio without enabling
the heavy tick sampler. The linked getter, its literal addresses, internal
pxCurrentTCBs pointer and SDK ROM interrupt-mask restore dependency are checked.
Application code does not read private TCB fields or query task names/priorities
inside the interrupt. The getter briefly masks local interrupts and does not
take the kernel cross-core lock.

The expanded audit checks the owned sent/finish_sent/audio roots and direct
references plus these required getter dependencies. It remains a limited audit,
not proof of the entire ISR callgraph or cache-off OTA continuity. Its receipt
starts false, preventing an old passing receipt from surviving a failed check.
The pre-placement image fails the getter-IRAM gate as expected. A subsequent
held audit exposed SDK absolute ROM symbols without nm size fields; inspection
of the actual linked instructions led to explicit ROM/literal validation.

Relevant primary references are the [ESP-IDF I2S callback documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2s.html)
and [FreeRTOS interrupted-task API guidance](https://html.freertos.org/FreeRTOS_Support_Forum_Archive/February_2013/freertos_How_identify_interrupted_task_from_ISR_6735048.html).
The pinned local kernel and original ESP32 driver implementations were also
inspected; the scheduling conclusion is based on that actual source.

## Host and target checks

The actual extracted dispatch policy and callback epilogue pass **122,880 native
and 122,880 disabled ASan/UBSan cases**: existing wake handling, self/other task,
useful input/output, stop/failure/core/deadline fences, and timestamp/counter wrap.
A return-only negative control retaining the old missing-dispatch behavior fails
at fixture line 85, case 23,147. Failed fixture/executable/output are retained.

Actual output tests at 100 and 1000 simulated Hz pass immutable initial/final
latches, partial refill/copy order, expiry/cancel, startup/wrap, and exact
split-versus-full PCM with EQ/volume changes at 48/192 kHz. The actual two-core
mocked DSP/sound lifecycle passes held borrows, queued stop/cancel, bounded
six-block assistance and completion/producer ordering. These host tests do not
measure the real ESP32 scheduler.

The complete ccache build and compiler-input recheck pass with 10,418 stable
source/build inputs. Final rebuildable output is 469.755 MiB under the unchanged
512 MiB gate. Image admission and the expanded target ISR audit pass. Linked
changes versus the previous renderer are **+228 B IRAM text, -32 B flash text,
+16 B flash rodata, and unchanged DRAM data/BSS**. The new counter fits existing
linked alignment; this does not make the IRAM addition free SRAM.

Current COM6 image is
**6f9382af415a3eeabc0aa57752b249e5e30e3ddf187cb21f88ada64b868f1ad0**.
Upload uses 921600 baud; full written-region readback matches. The original app
is preserved, owned selection is VALID, and automatic Bluedroid fallback is
false. Existing 37 cold codec configurations / 173 callbacks pass, with tracked
peak 56,899 B, minimum free after allocation 106,176 B and decoder stack low
3,468 / 8,192 B. These cold figures differ from earlier boots and are not a
matched Bluedroid memory comparison.

## Physical playback and actual bitrate

Approved Redmagic, native YouTube only, LHDC V5 / 192 kHz / S24 / stereo.
Every excerpt has an independent 14-second phone pause guard. Fixed-900 tests
request 11.5 seconds; Auto tests request 10 seconds. Final counts include the
entire stopped trace, including its tail; nothing is subtracted for pause.
Music is Get Lucky from 30 seconds; metronome is xpiVrEET-YQ from 1 second.

| Mode / workload | Initial shortage | Recovered | Final unfilled | Dispatch requests |
| --- | ---: | ---: | ---: | ---: |
| Fixed 900 / music | 64 | 64 | 0 | 1 |
| Fixed 900 / music repeat | 0 | 0 | 0 | 0 |
| Fixed 900 / metronome | 0 | 0 | 0 | 0 |
| Auto / music | 0 | 0 | 0 | 0 |
| Auto / metronome | 0 | 0 | 0 | 0 |

The one initial shortage is bounded after the recorded phone pause acknowledgment
and is fully recovered before its deadline. The other fixed-rate excerpts have
no initial shortage. Earlier pre-change music had a confirmed playback-time
miss, but these sequential runs are not an isolated A/B speedup proof. The new
request path is exercised once; zero-request runs cannot attribute their clean
result to a scheduler request that did not occur.

Fixed-900 payloads average 566 bytes with 960 RTP ticks / 5 ms per packet,
**905.6 kbps of AVDTP payload including codec framing**. Auto payloads average
506 bytes with 1920 ticks / 10 ms per packet, **404.8 kbps**, including framing.
Every measured active Auto interval agrees at that media-time payload rate.
The Auto packets cover two 5 ms codec-frame intervals; one packet must not be
assumed to equal one codec frame. These are source RTP media-time payload rates,
not RF throughput or an encoder API setting. Auto therefore did not exercise
the 1 Mbps target; why the source remained at this rate needs encoder/adaptation
review rather than an unsupported maximum-bitrate claim. Encoded-queue duration
and AVDTP delay accounting also need review against the observed two-frame
Auto packets; no one-packet/one-frame assumption is justified by these traces.

Valid in-playback DSP snapshots show gains **11/8/12 dB**, DSP volume **127**,
and flags **12**; phone media volume is **150 / 150**. Flag 4 selects the full-band
path and bypasses the split/crossover branch. The source still processes the
four common filter stages; flag 8 enables analysis. This is not an EQ-disabled
benchmark. Crossover/boost/flip mode coverage remains open. The metronome invokes
the DSP limiter and has post-EQ over-full-scale observations; zero queue gaps
does not prove transparent limiting or distortion-free acoustic output.

All five runs have zero queue/decode/reset/PCM timeout/sequence, ingress,
descriptor/refill-contract, helper and log-loss faults. LHDC decoder-quality
concealment/nonfinite/S32-clipping counters are zero, separately from the DSP
limiter above. Accepted/rendered/DMA-source PCM totals agree after each stop.
UART integrity totals are 249/243/257/551/509 Verified, 2/1/2/3/3 explicitly
Unverified legacy records, and zero Invalid.

After five excerpts, stopped internal heap free/minimum/largest is
63,036 / 27,488 / 45,056 B; decoder/render/logger stack low waters are
5,912 / 3,772 / 1,404 B. LHDC resident allocation is 60,704 B; its eight
1800-byte encoded slots account for 14,536 B including headers. No buffer was
increased. This is a finite allocation history, not a leak/stress qualification.

A fresh six-second passive capture has 190 CRC-Verified records and no Invalid
or Unverified records. Monitor PID 34852/source 107 is live at 921600. Native
YouTube is PAUSED and I2S stopped. The phone retains 192 kHz/S24/stereo/Auto.
Sustained maximum bitrate, BLE/control headroom, acoustics and broad parity
qualification remain open. Main and stable releases are not promoted.

## Reproduction and evidence

~~~powershell
python native_bt/tools/test_refill_dispatch.py
python native_bt/tools/test_probe_output_format.py --starvation-trace --light-gap --owner-state
python native_bt/tools/test_dsp_assist_output.py --fair-share --early-input --rebalance --assist-blocks 6
python native_bt/tools/audit_audio_isr.py
~~~

- native_bt/_build/experiments/refill-irq-dispatch-1794142199099/
- native_bt/_build/hardware/runs/1794142767374182800-6f9382af415a/traces/

Failed controls, the initial held audits, exact source hashes, typed-object
retirement manifests, lossless symbol archives and raw UART traces are retained.
