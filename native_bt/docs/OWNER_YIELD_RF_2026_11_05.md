# Owner-ready yield: measured RF result

The companion JSON binds the exact image, ELF, source manifest, complete capture
and phone receipt. Eight owning inputs were copied from the unchanged worktree
and hash-checked against the archived build manifest before the next DSP edit.

Android confirms LHDC V5, 192 kHz, 24-bit stereo and a 1000 kbps target; actual
transmission bitrate is not reported. Native CIE matches. The run delivers
1,418,176 source PCM sample frames to DMA, but still records 1,385 queue-full
drops, 149 PCM-busy results, one core1 idle watchdog and one descriptor-order
fault. Input sequence/SSRC breaks, VHCI ingress rejection, oversized payload
rejection and allocation failures remain zero in the corresponding snapshots.
Largest encoded payload is 628 bytes, below the 1800-byte decoder slot.

The first retained queue rejection is at 53.075766 seconds. The descriptor
fault occurs at 67.839821 seconds after a 9740 us EOF gap; the public watchdog
hook follows 22 us later. Shape/pool checks remain valid; expected descriptor2
and observed0 differ. Initial software queue saturation therefore precedes
the fatal output fault by 14.764055 seconds. Bounded trace losses are explicit;
the retained trace's `expected` field is not independent sequence-loss proof.

Render wall time is 11,737,480 us over 1,418,176 delivered PCM sample frames:
8.27646 us per sample frame, or 7.94540 ms per 960 sample frames. A codec frame
in this stream lasts 5 ms and contains 960 stereo PCM sample frames at 192 kHz.
The 5.20833 us figure is the budget for one PCM sample frame, not for one
encoded LHDC frame. These wall measurements include preemption and separate
collection windows; they are not isolated DSP timings or exact CPU usage.

The phone runner pauses about 14.967 seconds after Play upon observing watchdog
text. Its original immediate result says no audio was exercised because the
next complete I2S record had not arrived yet. That receipt remains unchanged;
the full capture contradicts that field and establishes real PCM/DMA progress.
The runner now allows up to four seconds for the fault snapshot to drain after
pausing. It does not delay the Pause command.

Baseline return is clean and verified. The cooperative yield requests do not
prove effective context switches, sustained headroom, continuous audio or full
feature parity. Different start/stop timelines prevent a controlled percentage
comparison against the previous priority6 run. Next, measure the exact block
DSP alternatives on LX6 with equal inputs and full state comparison, separately
from the controller and renderer.
