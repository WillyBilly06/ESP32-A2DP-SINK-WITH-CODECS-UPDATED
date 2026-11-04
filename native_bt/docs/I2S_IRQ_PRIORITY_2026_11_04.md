# I2S interrupt priority experiment

The [same-clock priority1 trace](RF_CAPTURE_WINDOWS_2026_11_04.json) records an
order fault 14.807 ms after the task-watchdog hook, with an 11.677 ms preceding
callback gap and no stop request. SDK cross-core/backtrace processing is one
candidate source of interference. Renderer throughput is independently too
slow, so removing the order fault would not establish audio continuity.

The experiment configures interrupt priority2 using the public
`i2s_chan_config_t.intr_priority` field. Descriptor shape/map/order validation,
watchdog monitoring/diagnostics, DMA size/count, buffer capacities, task placement,
codec settings and DSP are retained. The public driver converts the requested
priority to an interrupt-allocation level mask. No controller registers or
private driver state are modified.

Application callback calls/elapsed total/maximum/core are recorded separately.
This measures through the callback's accounting point, including any higher
priority preemption; it does not measure the entire SDK ISR wrapper. Recording
uses scalar IRAM/DRAM operations and no printing/allocation/waiting in the ISR.
The configured priority and counters appear in `i2s_callback_work`; counters
reset on output start. Host tests verify the actual driver configuration,
callback timing and existing format/error/teardown behavior.

Hardware comparison is pending. Watchdog or underrun failures remain failures,
even if interrupt gaps improve. The released fallback remains preserved.
