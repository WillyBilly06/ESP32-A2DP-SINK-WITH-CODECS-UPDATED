# Finite task-priority ordering

`-PriorityOrder` selects host7, decoder6 and renderer5. The SDK controller keeps
its priority23 (`configMAX_PRIORITIES=25`, `ESP_TASK_BT_CONTROLLER_PRIO=max-2`).
The host has a compile-time assertion that it remains below the controller.
No controller, IRQ, affinity, task stack, buffer, codec, DSP or storage behavior
is changed. This mode is restricted to a finite live asynchronous audio test;
it cannot confirm itself as the active production image.

The common scheduling header derives task priorities and the owner-ready-yield
condition. A priority7 host cannot give priority6 decoder work service merely
by yielding, so the equal-priority yield experiment is disabled in this mode.
The owner must actually exhaust runnable protocol work and wait for an event.
Bounded owner/worker batches and event/deadline notifications remain intact.
The mode is a test of priority ordering, not a throughput or fairness guarantee.
CPU affinity still matters: lowering the renderer on core0 does not create
extra core1 capacity for the controller, host and decoder.

Default priorities remain host6/decoder5/renderer6. The preceding optional
equal-priority experiment remains host6/decoder6/renderer6 with ready yields.
The build receipt, cache and actual three task units bind the chosen order.
Runtime layout records expose the derived host, decoder and renderer priorities.

Local checks pass: ten CMake cases, eight actual compiled header configurations,
sixteen image-admission cases, the previous decoder/finite/yield checks, a real
codec/worker run with the ordered mode, and actual renderer task-creation,
format, compact-copy and descriptor checks at 100/1000 Hz. These are not RF or
deadline evidence. Target build and a maximum-LHDC playback comparison remain
separate gates; no priority benefit has yet been measured on hardware.
