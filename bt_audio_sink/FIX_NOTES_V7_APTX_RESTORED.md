# V7 aptX restored

This package keeps the v6 streaming overlay/no-3D mixer fixes and restores CONFIG_BT_A2DP_APTX_DECODER=y in sdkconfig and sdkconfig.defaults.

Still disabled for RAM unless explicitly needed:
- CONFIG_BT_A2DP_OPUS_DECODER
- CONFIG_BT_A2DP_LC3PLUS_DECODER

Mixer architecture remains:
Bluetooth PCM -> DSP/EQ/beat detect -> overlay mixer -> I2S
WAV PCM -> streaming resampler -> overlay mixer -> I2S
