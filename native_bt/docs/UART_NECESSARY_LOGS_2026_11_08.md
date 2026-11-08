# Necessary runtime logging at 921600 baud

COM6 application UART, the owned build defaults and the live raw-UART web
monitor use **921600 baud**. This checkpoint suppresses individual expected
PCM_BUSY episodes in normal stay-native mode before they enter the diagnostic
ring. The cumulative pcm_busy counter is unchanged. Explicit SchedulerTrace
builds retain individual episodes; worker_pcm_busy_events reports that choice.
Sequence changes, decoder/PCM faults, timeouts, lifecycle records and immutable
first-gap snapshots keep their existing handling and separate records.

The normal trace producer only tests the event kind and returns for PCM_BUSY.
It performs no UART write, wait, allocation or additional counter update for
that event. Timing samples remain available. Formatting, CRC calculation and
UART transmission run in the existing priority-1 Core1 background logger;
the background writer may sleep for UART space. Logging still costs CPU time,
interrupt work and synchronization. It is not a one-cycle operation. At 8N1,
a byte occupies about 10.85 us of wire time at 921600; that is not a claim that
the audio task blocks for that duration. Full raw UART remains in the viewer.

## Verification

- ccache target build passes, with compiler-input rechecks and stable contents
  for 10,414 source/input entries. Actual-ELF ISR audit and offline image
  admission pass. Rebuildable output: 511.744 MiB within the 512 MiB gate.
- Linked SRAM and IRAM are unchanged versus the immediately preceding partial
  completion image. Flash text -4 bytes, rodata +32 bytes. No new task, ring,
  stack, allocation or audio buffer is added.
- COM6 image **0510a1c220e4bbe10a795cf567b88f1ac668020afbcc8f26fcbff0cbdbf4afe0**
  is written at 921600 and matches complete readback. Original app is preserved,
  owned selection is VALID, and automatic Bluedroid restore is disabled.
- Existing cold checks pass: 37 configurations, 173 callbacks, tracked peak
  56,899 B, minimum free after allocation 106,732 B and decoder stack low-water
  3,460 / 8,192 B. These are cold checks, not live all-codec qualification.
- Fresh six-second passive window: 185 raw UART records, all CRC Verified.
  Monitor PID34852/source89 is connected with verified COM6 USB identity at
  921600. Phone is PAUSED and I2S stopped at completion.

Two native YouTube music windows on Redmagic9125299103B7 use LHDC V5,
192kHz/S24/stereo, fixed900kbps UI selection and volume150. Each has an
independent phone-side 14-second pause guard and remains within15s playback.

| Window | PCM busy counter increase | Individual busy records | Worker trace losses | Final unfilled PCM frames |
| --- | ---: | ---: | ---: | ---: |
| First | 5582 | 0 | 0 | 6528 |
| Repeat | 5833 | 0 | 0 | 0 |

The preceding image's three short windows lose3232/2322/2927 worker diagnostic
records while emitting34/28/31 busy records. These are diagnostic losses,
not audio packet drops. The new filter avoids that routine trace pressure;
no decoder speedup or continuity improvement is inferred from these windows.
Both current windows retain zero queue/decode/reset/PCM timeout/sequence,
ingress rejection, descriptor or DMA contract faults. Playback records are
CRC Verified except2/1 explicitly Unverified legacy records. The repeat's
128 initially short frames were recovered, leaving zero finalized unfilled
frames. The first window's6,528 finalized unfilled frames remain a failure.

The first finalized gap has empty raw and processed PCM while the decoder is
waiting for encoded input. Deep HCI-arrival association is disabled, so this
cannot determine whether the delay originated on-air, inside the controller,
or in host delivery. LHDC continuity and1Mbps remain unqualified. Sustained
BLE/control coexistence, full feature/security/OTA parity, acoustic checks and
the comparative Bluedroid RAM gate remain open. This is development firmware.

Evidence: native_bt/_build/experiments/quiet-backpressure-1794129874158/ and
native_bt/_build/hardware/runs/1794129991340320500-0510a1c220e4/traces/.
