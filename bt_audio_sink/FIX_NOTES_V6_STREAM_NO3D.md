# V6 streaming SFX + no 3D

Changes:

- Reverted away from whole-WAV caching. Sound effects stream buffer-by-buffer from SPIFFS.
- The streaming loop closes the WAV file and stops resampling immediately when EOF, timeout, or stop() occurs.
- Overlay ring-full guard is shorter so a stalled renderer cannot keep the sound task alive for seconds.
- Resampler chunk-boundary interpolation was fixed so it does not skip the first sample of each new chunk.
- 3D sound is removed/no-op in this low-RAM build. The encoder/BLE API remains compile-compatible but does not allocate/process the 3D crossfeed.

Architecture remains:

Bluetooth PCM -> DSP/EQ/beat detect -> OverlayMixer -> I2S
WAV PCM ----stream/resample chunk----> OverlayMixer -> I2S

Only AudioPipeline/I2SOutput writes to I2S for max-volume overlay sounds.
