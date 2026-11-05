# Finite playback orchestration

The first owner-ready-yield run was boot-only: no phone playback was started
within its finite window. `OWNER_YIELD_BOOT_ONLY_2026_11_05.json` records zero
accepted/DMA source frames and a clean return to the preserved baseline. It is
not audio evidence.

`tools/run_phone_audio_trial.py` runs independently of the model and flasher.
Start it before the normal admitted `deploy_audio_trial.py` finite deployment.
It never opens an ESP32 serial port. It waits for a capture written after its
own start and both Classic security and GATT readiness, with the expected
priority-6 owner-yield layout. The exact Redmagic serial and COM6 Bluetooth
address are fixed. Android Settings must already expose that device's details;
an unrelated device or unexpected screen fails closed. No bonds are removed.

Fresh UI dumps locate each control. Clipped/disabled targets are rejected.
The runner selects LHDC V5, 192 kHz, 24-bit, stereo and 1000 kbps target, checks
fresh Android current-connection readback, then starts the existing paused
media source. Android does not expose actual encoded transmission bitrate.
Playback observation lasts at most 90 seconds; default 60. A separate capture
thread validates periodic record CRC/time fields and pauses on I2S failure or
descriptor faults. Exceptions also attempt Pause and stop the BDK app. UI work
has a 125-second deadline after readiness. Readiness itself is bounded.

Private receipts and UI XML stay in `_build/hardware/phone-trials`. Accepted PCM
and DMA source progress establish whether audio was exercised. The native CIE
is reported separately. This orchestration does not claim continuous-audio,
acoustic, full feature, or long-duration qualification. Standard deployment
verification and baseline return remain mandatory and separate.

Four offline safety tests cover exact fixture identity, clipped/disabled taps,
fresh actual configuration readback, and rejection of corrupt, duplicate-key
or unprotected periodic records. No device access occurs in those tests.
