# V9 seamless duck-gain fix

This version removes the v8 same-rate I2S recovery/mute behavior and fixes the actual post-overlay distortion cause.

Root cause: OverlayMixer used int16_t for duck gain. During the post-WAV restore ramp, the gain stepped upward toward 32767. The final add could exceed INT16_MAX and wrap negative, so the BT gain became a repeating negative/positive sawtooth. That made Bluetooth audio sound permanently distorted/static after the max-volume overlay ended.

Fixes:
- OverlayMixer duck gain and target are now int32_t.
- Duck gain ramp clamps safely to [0, UNITY_Q15].
- Removed v8 hard same-rate I2S recovery after overlay finish.
- Max-volume sound still streams buffer-by-buffer from SPIFFS through OverlayMixer.
- 3D sound remains removed.
- aptX, AAC, and LDAC remain enabled.
