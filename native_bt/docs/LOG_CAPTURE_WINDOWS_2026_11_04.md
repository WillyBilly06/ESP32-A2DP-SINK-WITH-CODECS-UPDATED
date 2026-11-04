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

Host tests exercise delayed observation, uint32 clock wrap, bounded formatting
loss, each actual ISR error class/time/gap/stop state and stream reset. Hardware
validation of these new time fields is pending; do not use old periodic loop
timestamps as if they were fault-event timestamps.
