# Mixer/I2S handoff fix

This version enforces the mixer architecture:

- Max-volume sound is forced to SOUND_MODE_OVERLAY inside SoundPlayer, even if a caller accidentally asks for exclusive mode.
- Overlay mode fails safely if overlay hooks are not connected; it never falls back to direct I2S writes.
- The overlay push callback wakes the audio render task whenever samples are accepted.
- AudioPipeline keeps clocking I2S while an overlay is active, even when Bluetooth has only a partial chunk or is still prebuffering.
- Drift catch-up frame dropping/inserting is disabled while overlay/ducking is active.
- OverlayMixer treats empty overlay data as silence, clears consumed slots, resets duck gain on clear, and uses mutex-safe active checks.

Rule: for max-volume/system overlay sounds, only AudioPipeline writes to I2S. SoundPlayer decodes/resamples WAV PCM and feeds OverlayMixer.
