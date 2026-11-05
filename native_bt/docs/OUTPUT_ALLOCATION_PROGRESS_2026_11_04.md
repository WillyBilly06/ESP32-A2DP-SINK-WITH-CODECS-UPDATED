# Measure allocation progress before changing memory policy

The preceding maximum-LHDC run failed a 4096-byte page request with 6656 bytes
free and largest free block 1984. It did not record successful page count or
remaining payload. That evidence alone cannot distinguish a final-page
fragmentation failure from a larger total-memory deficit.

The startup allocator now latches audio payload requested, successful audio
payload and completed output pages when a request fails. Input allocation is
counted separately from pages in the copied provider. It also records 8-bit
free heap before audio allocation, after partial-allocation cleanup, and the
free/largest internal 32-bit-capable heap at failure. The 8-bit and 32-bit sets
overlap; their totals must never be added. Pages still request 8-bit memory.
DMA/channel allocations are not made through this allocator and are excluded
from these audio-payload counts.

Only render-owned startup allocation updates the plain progress counters.
Failure fields are latched into atomic diagnostic counters. Periodic snapshots
retain their bounded observation-window semantics; they are not one coherent
instantaneous cross-task tuple. Stable failure records, with the same failure
timestamp repeated after cleanup, support admission analysis. No allocation
retry, buffer-capacity change, precision change or real-time sample work is
introduced.

The copied-provider tests inject failure after the input and one page, verify
167936 requested/69632 successful bytes and one completed page at 192 kHz, and
check that the next successful stream clears failure metadata. Sanitized loan
tests inject failure after one page and on the first page, verifying 102400
requested bytes, 4096/0 successful bytes and 1/0 completed pages, respectively.
Both verify complete partial cleanup. Copied tests run at 100/1000 Hz task
ticks; the loan provider uses ASan/UBSan. These are host failure-injection tests,
not physical SRAM admission results.

The expanded record is at most 664 characters including conservative snapshot/
checksum suffix allowance, within the existing 1024-byte logging bound. The
ccache-enabled full product build and source/image/layout admission pass.
Compared with the exact preceding full-product ELF, linked DRAM data increases
by 48 bytes (23861 to 23909); BSS 33264 and IRAM text 122891 are unchanged.
This diagnostic has a measured memory cost, not a claimed saving. Exact
hardware evidence is recorded below.

## COM6 maximum-rate admission

[The source-bound result](OUTPUT_ALLOCATION_PROGRESS_RESULTS_2026_11_04.json)
uses the full codec/product/security/RFCOMM build. Android freshly confirms
LHDC V5, 192 kHz, 24-bit stereo and the 1000 kbps target; measured bitrate is
not reported. Playback remains paused during preference application, then the
existing media session is started. No preceding 48 kHz DMA stream is included
in these allocation measurements.

The request needs 102400 bytes of processed PCM. Twenty of 25 output pages
succeed (81920 bytes), leaving 20480 bytes when a 4096-byte page fails. At that
point 8-bit internal heap has 10744 free bytes and largest block 3328; internal
32-bit-capable heap has 17168 free bytes and largest block 6144. These are
overlapping capability views, not two additive pools. The current path still
allocates 8-bit pages.

Channel creation follows successful audio allocation, so its three 128-frame
stereo/S32 DMA buffers (3072 bytes) have not yet been allocated. Remaining PCM
plus DMA payload therefore exceeds the measured 8-bit free count by at least
12808 bytes. Even hypothetical word-only eligibility leaves a 6384-byte
payload deficit. Both lower bounds exclude future allocation and driver
overhead. Fragmentation also exists, but changing only allocation capabilities
cannot satisfy this measured total-capacity requirement.

Free 8-bit heap is 92744 both before the attempt and after partial cleanup.
Stable repeated failure records agree on timestamp, requested/allocated bytes
and page count. This failed attempt reclaims its payload; it is not a repeated-
lifecycle leak qualification. No PCM or DMA processing starts. Zero watchdog
warnings in this idle/failed-start trial do not fix the previously measured
streaming starvation.

The 180-second capture has 1892 CRC/time-valid periodic records and one rejected
truncated record. Released 1.0.4 firmware returns with System ready and no fatal
marker; phone playback is verified paused. This remains a maximum-output
admission failure, not a feature-parity or playback pass.

The main memory target is now the 102400-byte processed-PCM queue. A replacement
must preserve full sample precision, bounded ownership, jitter handling and
all product functions. Merely adding more small pages or selecting word-only
memory cannot close the measured capacity gap. Any latency/buffering experiment
must remain explicit until end-to-end timing and stress behavior are verified;
the current default 64 ms target remains unchanged at this checkpoint.
