# A2DP worker-fault session recovery

## Confirmed source defect

The product adapter treated any nonzero cumulative worker.faults counter as a permanent global failure. Its next poll called dbt_a2dp_acceptor_stop, setting profile.enabled false. That latch was never cleared during normal operation. A codec reset failure, a 200 ms PCM-delivery stall, or an asynchronous stream-control timeout could therefore prevent subsequent A2DP connections or codec switching until reboot, even after all failed-session resources were retired. This is a source-derived recovery defect; it does not establish the origin of the user's reported panics or maximum-rate performance failure.

## Replacement behavior

An owner-only fault-generation cursor detects each change in the cumulative worker counter and retires the current A2DP session with dbt_a2dp_acceptor_retire. It does not stop the whole service. The existing acceptor keeps admission closed while channels are staged or the session drains. It closes media/signaling channels, waits for bearer retirement, submits DROP, and requires the worker's READY acknowledgement before discarding the old binding.

The decoder worker continues to fence every accepted PCM loan: cancellation/poll must actually acknowledge retirement before reset, close or free. A missing provider acknowledgement can therefore keep retirement pending; this change does not force-free pinned memory or restore admission early. It allocates nothing and adds one four-byte owner cursor.

The cumulative fault counter is retained, so logs still expose the failure. Existing native_profile records show enabled/active/draining independently. No automatic retry loop, reconnect attempt, controller reset, new task or periodic timer is added. A later valid peer connection may establish a fresh session after retirement.

Explicit restart/OTA holds and renderer/ownership failures keep their existing stop policy. This change does not clear output.failed or request_fault, erase bonds, bypass security, or restart the Bluetooth controller.

## Evidence boundary

Reviewed the worker gate, immutable media and PCM ownership, async stream tickets, acceptor drain, and sink DROP completion. Full ccache firmware compilation, source-integrity checks, flash readback and ordinary startup are recorded separately. Functional recovery, codec switching and playback tests remain stopped at the user's request. No claim of end-to-end recovery qualification or crash-free playback is made.

## Reference comparison

BTstack commit 9edb5d82ea6ec4466a393415d65030929330acdf, src/classic/a2dp.c lines 977-1002, distinguishes stream release from signaling release and returns the affected configuration process to CONFIGURED/IDLE. This supports the connection-lifecycle boundary; it is not proof of our decoder or renderer ownership. The independently authored drain/DROP implementation above remains our own. Reference snippets are retained locally under native_bt/_build/experiments/worker-recovery/pinned-reference.json.
