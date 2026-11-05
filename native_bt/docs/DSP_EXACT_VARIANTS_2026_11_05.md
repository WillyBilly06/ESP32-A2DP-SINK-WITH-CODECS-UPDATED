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

## Target result

The [target receipt](DSP_EXACT_TARGET_RESULTS_2026_11_05.json) records all 108
rows and complete PCM/state equality. At 192 kHz, the median steady costs for
the current block kernel versus the product-state prototype are 5.998 versus
7.160 us per PCM sample frame for flags9, 6.072 versus 7.232 for flags13,
and 3.153 versus 3.518 with flat EQ. The prototype therefore costs about
19.1–19.4% more time with the non-flat policies and 11.6% more with flat EQ.
Keep the current block kernel. Plain whole-unit O3 is also slower than the
current block implementation in these cases.

Disassembly already shows native `mull`/`mulsh` products in the current stable
kernel. The prototype has a larger helper and more load/store instructions;
static counts are not dynamic execution counts or speed proof. Target timing,
rather than an assumption that a different formula or optimization level is
faster, drives the decision.

The separate synthetic pipeline accepts 12992 PCM sample frames but completes
only 13 of 96 required packets. It has no allocation failure; it fails the full
correctness/continuity gates and never starts DMA in that observation. Its
startup/ramp stage costs cannot be substituted for steady kernel timing.
Controller-off kernel correctness does not qualify the full pipeline. The
diagnostic selects the baseline and its released boot is observed cleanly.
