# Wake the retiring BLE control worker on events

The general BLE control worker previously used an unconditional 20 ms
`vTaskDelay` while an asynchronous provider finished retirement. A newly
accepted command or a sound/OTA provider completion could signal the existing
semaphore during that sleep, but could not wake it early. This is a control
latency issue, not evidence for the measured LHDC transport/render failure.

Retirement now waits on the existing persistent binary semaphore with the
same maintenance interval. A wake arriving before the wait remains pending.
The worker rechecks queued commands and the external-worker requirement before
continuing retirement; the existing resumed/retired acknowledgments and owner
fences remain intact. No task, allocation or queue is added.

The periodic maintenance wait is clamped to at least one RTOS tick. Both idle
maintenance and retirement use it, avoiding a zero-time polling loop when
20 ms rounds down to zero at a coarse tick rate. This does not promise a hard
20 ms bound at every tick frequency or under preemption. The deployed target
uses 1000 Hz; scheduler and real provider latency still need measurement.

The modified actual-worker regression fails against the old sleep at the
assertion requiring two semaphore waits and zero fixed delays. After the
change it passes at 10, 100 and 1000 Hz, injecting a newly accepted command at
the retirement wait and checking that the same worker resumes before retiring
again. Existing ATT/product/NVS-shim tests also pass at each rate, including
the four separate accepted EQ commits, durable state, stale generations,
CCCD handling, OTA barriers and control acknowledgments.

Receipts are `_build/ble_control/receipt.json`, `tick10/receipt.json` and
`tick100/receipt.json`. They bind the actual source inputs and remain host/shim
evidence. There has been no radio/flash timing qualification of this change.
