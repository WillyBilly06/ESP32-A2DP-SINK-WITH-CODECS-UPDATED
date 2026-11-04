# Snapshot and DMA fault timestamps

Historical periodic `at_us` fields were sampled at the beginning of a logger
iteration. On an overloaded core, the low-priority logger could be preempted
before reading later counters. Those fields therefore cannot establish exact
counter-observation time or correlate a DMA fault precisely with watchdog output.
The actual counters and renderer/decoder work spans were measured independently;
their final quiescent totals remain usable. Historical raw records are retained.

The new logger records `snapshot_begin_us`, `at_us` (collection completion) and
`snapshot_span_us`. Values were observed somewhere within that interval; a long
interval is not an instantaneous coherent snapshot. Existing loop timing is
renamed `collection_cycle_us`. PCM/profile/output groups start their window
before collecting local values. Each record is bounded-format into a 1024-byte
logger stack buffer and emitted in one ordinary task write. Oversized/malformed
formatted records increment `snapshot_records_lost` and are not partially emitted.
SDK early/ISR output can still interleave on the UART independently of normal
task stream locking; raw captures retain such data. These changes add no
formatting, allocation, waiting or UART work to transport/decoder callbacks.

The I2S ISR now records the fault's own `fault_at_us`, preceding EOF gap and
stop-request state when shape/map/order validation fails. These remain separate
from logger observation windows and are reset with each output start. Existing
validation and fail-closed behavior are unchanged. The logger also reports its
stack low water so the temporary formatting-buffer cost is visible.

The public SDK task-watchdog hook also records an ESP-timer timestamp, event
count and ISR core using scalar IRAM/DRAM operations. In this SDK the hook runs
after task-list diagnostics and before backtrace handling. Its time is not the
beginning of the entire timeout handler, and ISR core is not the identity of
every starved core. Normal watchdog monitoring and diagnostics remain enabled.
This allows same-clock comparisons without assuming SDK text timestamps have
an identical origin or blaming a fault on a stale logger cycle time.

Host tests exercise delayed observation, uint32 clock wrap, bounded formatting
loss, each actual ISR error class/time/gap/stop state and stream reset. Hardware
validation is recorded in [the full-output follow-up](RF_CAPTURE_WINDOWS_2026_11_04.json).
Of 1044 parsed snapshot records, one has inconsistent timestamp fields and is
excluded from timing analysis; its raw values remain in the report. The cause
is undetermined. The other 1043 pass the unsigned timestamp invariant. The
formatting-loss count is zero; that counter does not detect UART/capture
corruption. The logger retains at least 1568 stack bytes. The watchdog hook was called once
on core1. An order fault occurred 14807 us after that hook, following a 11677 us
I2S callback gap, with no stop request. This supports an interrupt-priority
interference experiment; temporal correlation alone does not prove causality.
Full output still failed. Do not use old periodic loop timestamps as if they
were fault-event timestamps.
