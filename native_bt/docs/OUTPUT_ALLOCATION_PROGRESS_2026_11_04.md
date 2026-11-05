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
hardware evidence is pending.
