# LHDC complete-frame cycle accounting — 2026-11-07

The70 live profile measured about640us outside the six channel marks. This change adds four nonoverlapping frame regions: preparation/header/readiness, the complete channel-call envelope, output scale/safety scan, and output/tail bookkeeping. Frame counts are separate from channel counts. Publication now happens after every16 completed frames, so a normal stereo snapshot contains matching frame/channel endpoints; final close still publishes partial work. Original64-bit finite channel APIs remain compatible.

Only diagnostic timing and background formatting change. Existing codec arithmetic, safety checks, PCM, layouts, ownership, tasks, priorities and buffers are untouched. Host ASan/UBSan tests cover frame cadence/partial publication, per-frame low-word and private64-bit wrap, canaries, epochs, cold visibility, inactive behavior and actual logger JSON. Existing diagnostic tests pass.

The real compiler preprocessor confirms CONFIG_IDF_TARGET_ESP32=1, LHDC_NSLOTS=1 and cycle-stage profiling enabled; there is no unintended generic/dual-slot decoder selection.

Full ccache build and image admission pass for8ecd27c50ec85e4780af82b977d279796b98d691be1bd32b36036b37aa30d7f7 (1340464bytes). Relative to70, BSS+88, IRAM/data0, flash text+352 and rodata+160 bytes. This is debug accounting overhead, not a speed optimization. No new boot cases. At document creation no target run is claimed.

Experiment: native_bt/_build/experiments/lhdc-frame-cost-1794064131432.
