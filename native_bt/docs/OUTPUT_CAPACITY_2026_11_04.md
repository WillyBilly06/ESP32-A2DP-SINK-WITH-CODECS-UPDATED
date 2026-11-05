# Output-ring capacity without power-of-two slack

The output provider targets 64 ms of PCM. Its startup watermark adds the
existing three 128-frame DMA descriptors; allocation adds the existing extra
128-frame margin. Previously the sum was rounded to a power of two, reserving
slots above every watermark the sole renderer can reach.

The output ring now accepts 2..32768 frames and allocates its final page at its
actual size. The provider rounds only to 512-frame allocation pages. Input
capacity, target buffering time, prefill margin, DMA count/size, precision,
codec configuration, processing and ownership are retained.

| Stereo S32 rate | Previous output bytes | Output bytes | Saved |
|---:|---:|---:|---:|
| 48 kHz | 32768 | 28672 | 4096 |
| 96 kHz | 65536 | 53248 | 12288 |
| 192 kHz | 131072 | 102400 | 28672 |

At 192 kHz, input plus output PCM is still 167936 bytes, before DMA and codec
resources. This saving alone does not prove the full radio product will admit
maximum LHDC, and does not solve the measured render throughput failure.

The renderer can write only while occupancy is below its watermark, and bounds
each render request by the difference. Therefore peak occupancy cannot exceed
target plus startup DMA prefill, even if DMA consumes concurrently. Removing
allocation above that bound does not remove a usable buffering reserve.

The publication counters retain uint32 wrap semantics. Each SPSC endpoint now
owns a separate physical cursor, independent of the counter: render writes the
write cursor and DMA writes the read cursor. Cursors wrap with a bounded
comparison/subtraction; there is no division/modulo, lock, allocation or wait in
sample processing. Neither endpoint reads the other's non-atomic cursor.
Open/close still require quiescent producer/consumer ownership. The two cursors
add eight bytes per audio object.
Before enabling DMA, the render owner performs preload consumption; the read
cursor transfers to the ISR only when that preload is complete. It is never
shared between concurrent task and ISR consumers.

Host ASan/UBSan tests cover 13 capacities from 2 to 32768, partial final pages
with poisoned allocation extents, distinct physical/counter wrap points,
mixer boundaries, full/empty operation and allocation unwind. Thirty format/
channel/capacity comparisons match every output sample against the smaller
power-of-two reference ring. All 180 portable test groups pass, including
separate producer/render/consumer threads. O3 drift/spectral checks and actual
provider capacity/format/error tests pass. The concurrent three-owner test uses
a 6656-frame output ring and validates 16384 frames against the DSP reference.
The ccache full-product build and source/image/partition admission pass.

The [LDAC96 hardware follow-up](RF_CAPACITY_LDAC96_2026_11_04.json) ran DMA with
the page-sized ring: 22537 callbacks, zero descriptor errors, maximum callback
45 us and EOF gap 1378 us. Core0 still produced six watchdog warnings and
1381248 underrun frames; mean render work was 1801 us per bounded unit versus
the 1333 us budget. Duration/control timing differ from previous runs, so no
isolated CPU speedup is claimed. The phone confirmed live Deep Bass and the
subsequent Balanced restore.

The [maximum LHDC admission follow-up](RF_CAPACITY_LHDC_MAX_2026_11_04.json)
still fails before I2S start. Its CRC-validated failure record explicitly binds
the request to 192 kHz/stereo/four-byte PCM containers: a 4096-byte page failed
with 11740 free bytes and a 3712-byte largest block. These heap figures depend
on live allocation state; they do not isolate a memory improvement. The 28 KiB
saving describes the required capacity, not a successful full 192 kHz allocation.
Released 1.0.4-idf6.1 ota0 returned with System ready, and the phone was paused.
