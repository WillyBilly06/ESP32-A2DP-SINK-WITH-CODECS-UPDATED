# Partial controller-to-host completion returns

The owned host can now return the safe subset of pending ACL completions when
only one or two encoded-packet slots become available. The previous boolean
gate waited until enough space existed for its entire pending batch. That
conservative batching was not a specification violation, but it delayed reuse
of already available capacity. This change does not increase any buffer.

The limiter preserves the three-packet physical ingress window, one reserved
encoded slot, and permits held by other links. For the current Classic audio
link, releasable completions are bounded by both its pending count and
free slots minus all other outstanding permits. LE and nonaudio completions
retain ordinary handling. Counts retire only after transport acceptance;
rejected sends, retained TX ownership, disconnects and handle reuse remain
accounted separately. The existing boolean gate API remains compatible.

Decoder slot retirement now wakes the owner whenever it publishes capacity in
the pressure region, rather than only on the low-water equality. This makes
partial releases event-driven. There is no new poll, task, queue or allocation.
The limiter callback is pure, bounded and must not mutate or reenter the flow
controller. Linked DRAM grows 8 bytes for diagnostic metadata; IRAM is unchanged;
flash text grows 128 bytes and rodata 96 bytes versus the e2a370485f27 image.

## Verification

- Focused actual-source ASan/UBSan checks pass: legacy setup and gate behavior,
  partial/failed/oversized grants, mixed Classic/LE, ordinary command-credit
  independence, disconnect/generation/retained-wire lifetime, 126 application
  policy combinations and lifecycle guards. Actual retirement code verifies
  publish-before-notify and a 7-to-6 queue transition permitting one credit.
  A bounded 100,000-step mixed-link/fragment/deferred-PDU model conserves ACKs
  and capacity for depths 4 and 8. This model is not radio timing evidence.
- Full host suite: **188/188 pass**, including concurrent SPSC 20,000 packets
  and three-owner audio/control 16,384 frames. Object compilation uses ccache.
  Thirteen stale fixture failures were also reproduced with the pre-change flow
  implementation. Fixtures now include the existing Connection Accept Timeout
  setup command, valid Discover responses, local rejection of malformed peer
  traffic, genuine sticky fatal-fault stimuli, and immediate initial DSP policy.
  No product validation was weakened to satisfy those fixtures. The omitted
  existing input-clock test was restored to the suite inventory.
- Target ccache build, source stability, image admission and actual-ELF ISR
  audit pass. Rebuildable output is 510.586 MiB under the existing 512 MiB gate.
- COM6 image **eda9122c88da83b86cdf5c133d036c7634c8a3683e55c0860a51b4ffc36f9586**:
  written at 921600, full readback matches, preserved original app, owned VALID
  selection, no automatic Bluedroid restore. Existing 37 cold configurations /
  173 callbacks pass; tracked peak 56,899 B; minimum free after allocation
  106,732 B; decoder stack low-water 3,476 / 8,192 B.

Redmagic 9125299103B7, native YouTube, LHDC V5 / 192 kHz / S24 / stereo,
fixed 900 kbps UI selection, volume 150, independent 14-second pause guards:

| Window | Final unfilled PCM frames | Accepted = rendered = DMA frames | Partial completion commands |
| --- | ---: | ---: | ---: |
| Fresh prior-image music control | 1152 | 2537280 | unavailable / old policy |
| Candidate music | 576 | 2512320 | 1744 |
| Candidate music repeat | 0 | 2545472 | 2065 |
| Candidate metronome | 0 | 2541888 | 1966 |

Each candidate window has zero queue/decode/sequence/reset/PCM timeout,
ingress rejection, descriptor or DMA contract faults. Checked records have
valid CRC; some legacy records remain explicitly Unverified. The first music
gap has empty raw and processed PCM while decoding is active, with a preceding
27,273 us maximum admitted-media gap. Deep arrival metadata is disabled, so
that record does not identify a radio-versus-controller-versus-host cause.
Routine PCM_BUSY records still overflow worker diagnostic storage; cumulative
audio counters and immutable gap latches remain available. These short windows
are not an isolated causal A/B result or continuous-playback qualification.

Firmware and monitor remain **921600 baud**. Full raw UART remains available;
formatting/transmission runs in the background logger, priority 1 on Core 1.
An earlier 19.884-second control capture used approximately 12.37 percent of
921600 8N1 wire capacity; that is traffic volume, not measured CPU utilization.

Evidence: native_bt/_build/experiments/partial-credit-1794127072299/ and
native_bt/_build/hardware/runs/1794129342294660200-eda9122c88da/traces/.
Phone is paused and I2S stopped at completion. One clean repeat does not prove
LHDC continuity, 1 Mbps, sustained BLE coexistence, acoustic parity, complete
feature/security/OTA coverage or reduced RAM against Bluedroid. This remains
development firmware and is not promoted to main or a stable release.
