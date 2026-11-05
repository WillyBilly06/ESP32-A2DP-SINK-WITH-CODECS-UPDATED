# Exact DSP alternatives: target comparison preparation

The latest RF measurement needs about 7.95 ms of renderer wall time per 960
PCM sample frames, including preemption. Those 960 frames represent 5 ms at
192 kHz. Larger queues cannot provide sustained throughput by themselves.

The isolated comparison now rotates four implementations through identical
128-frame input blocks: canonical Os, canonical O3, the current exact block
kernel, and the exact product-state block prototype. It retains three rates,
three routing/EQ policies and three trials. Every block checks the entire PCM
output and canonical DSP state against Os. Ramp prefixes remain canonical;
the prototype applies only after coefficients stabilize. It does not reduce
sample rate, precision, filter count or user-visible DSP behavior.

The new unit is diagnostic-only. No renderer selects it. Its wrapper stays Os
and the existing stable filter helper retains its local O3 attribute. Target
comparisons record core cycles, including preemption, and alternate ordering;
controller, BLE and I2S are inactive during this kernel comparison. These are
not full-product headroom measurements. The diagnostic workspace is allocated
once and freed; the comparison deadline is bounded at 14 seconds and the
outer pipeline diagnostic at 27 seconds, within the 35-second capture.

Local validation: the actual comparison runner passes 27 groups / 108 rows
with full PCM/state equality under ASan/UBSan. Six actual CMake configurations,
ten image-validator unit/optimization checks and two early CLI rejections
pass. Host timer/cycle/RTOS/heap values are stubs. Target timing and performance
remain unproven until the separately admitted finite hardware run completes.
