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

The [finite hardware comparison](RF_I2S_IRQ2_2026_11_04.json) serviced 23638
callbacks on core0, with a 46 us maximum application callback and a 1378 us
maximum EOF gap. Descriptor shape/pool/order errors remained zero through six
core0 watchdog events. Priority1 previously faulted after an 11677 us gap;
this supports retaining priority2 for further experiments, without claiming
an isolated cause or long-duration qualification.

Full LDAC output still fails: 1502592 underrun frames and 1472 encoded-queue
drops. Mean render work is 1898 us per bounded unit, above the 1333 us budget
for 128 frames at 96 kHz; the mean includes drain/partial work and preemption.
One record with a missing timestamp-field character is excluded. Counters were
observed repeatedly after pause; the historical UART records have no checksum.

A subsequent [maximum LHDC output attempt](RF_LHDC_MAX_OUTPUT_2026_11_04.json)
confirmed 192 kHz/S24/stereo/1000 kbps target in Android, but failed a 4096-byte
audio allocation with 7540 free bytes and a 3072-byte largest block. No LHDC
output start or continuity pass follows. COM6 returned to released 1.0.4-idf6.1
ota0 with System ready; phone playback was verified paused.
