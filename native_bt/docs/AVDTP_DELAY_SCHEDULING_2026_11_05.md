# AVDTP delay-report owner deadlines

## Confirmed scheduling gap

The adapter already attempted delay reports once a second in probe_audio_poll. Its owner wait function returned only the acceptor's protocol/channel deadline, omitting the report cadence. With an otherwise quiet configured or suspended stream, the owner could wait indefinitely and the next report depended on unrelated traffic. This is a scheduling defect in the existing report policy, not evidence that a report changes codec bitrate or fixes PCM throughput.

## Change

One owner-only eligibility/deadline helper is shared by poll and wait. A fresh binding reports immediately; subsequent attempts become due after the existing 1000 ms interval. Completed configuration/reconfiguration re-arms the first report even if the signaling binding stays unchanged, independently of the product observer.

Pending worker commands and a used local AVDTP transaction suppress the report deadline. Their existing completions, protocol deadlines and transport notifications wake the owner; no retry polling task is introduced. Failed, retiring, disabled and restart-held profiles do not arm report deadlines. The old two-word binding/time state is moved to file scope, with no added persistent bytes or allocation.

Attempts advance the cadence before parsing/queuing, so an unavailable provider cannot leave an immediate deadline that spins the owner. The three-byte delay command uses existing inline AVDTP storage. If command submission observes a newly expired/failed signaling state, the acceptor is explicitly retired instead of sleeping on a failed signal with no remaining deadline.

## Source-derived scheduling cases

| State | Report contribution to owner wait |
|---|---|
| Fresh accepted configuration, ready signaling | 0, then one attempt |
| Same binding, 250 ms since prior attempt | 750 ms |
| Same binding, interval expired | 0, then one bounded attempt |
| Remote command pending or local command busy | No additional report timer; protocol/completion wake remains |
| Reconfiguration commits on existing binding | Immediate once signaling is ready |
| Session idle, failed, retiring, disabled or restart held | No report timer |
| Attempt fails without signaling failure | Next attempt after 1000 ms |
| Submission discovers signaling failure | Existing asynchronous session retirement |

Timestamp subtraction retains the existing unsigned wrap-safe interval arithmetic. Reports still use the current PCM/input/DMA/FIR latency estimate in probe_output_delay_100us; this change does not claim that estimate is measured end-to-end latency.

## Evidence boundary

Source review and firmware compilation/integrity/deployment evidence are recorded separately. No functional delay-report, playback, codec-switch or app test campaign is run, following the user's instruction. Maximum-rate LHDC headroom and the reported codec crashes remain unresolved.

The DSP inspection in this pass found existing identity-filter and endpoint-blend fast paths, so no duplicate bypass was added. The limiter still preserves its original wide integer division. No DSP fidelity or sound settings were changed.
