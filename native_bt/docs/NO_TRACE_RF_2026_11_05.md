# Maximum LHDC with extra task tracing disabled

[Exact results](NO_TRACE_RF_RESULTS_2026_11_05.json) bind source 27b8e761,
image eb94debe629a, its ELF and the frozen build/source receipts. Neither the
scheduler unit nor scheduler/host-phase records are present. Existing separate
codec, transport, queue, PCM, renderer and DMA diagnostics remain.

The hot atomic-status fix is included. Host priority6, decoder priority5 on
core1, renderer priority6 on core0 and logger priority1 are unchanged. Compact
copied input, exact block DSP and the finite 16 ms output target are retained.
The source is reset to its beginning and paused before connection; Classic and
GATT readiness are observed before connecting. Android and ESP32 confirm
192 kHz/S24/stereo LHDC with the 1000 kbps nominal target. Actual encoded
bitrate is unavailable.

All 37 pre-controller codec configurations pass. Live output nevertheless
fails: 144000 source frames reach DMA, 847232 underrun frames, 788 queue-full
drops, one core1 idle-starvation watchdog and one descriptor-order fault.
The fault follows a 12110 us EOF gap at 123.849049 s, 35 us before the watchdog
hook. Pool/shape are valid, expected descriptor2, observed1, not stopping.
Ingress rejection, upstream sequence breaks and size rejections stay zero.

The exact interrupted stack is in L2CAP/runtime packet processing; it does not
measure the duration of one parser call. The failure exists without the extra
task/phase sampler, so it is not necessary to produce this behavior. General
periodic diagnostic overhead is still present and is not excluded by this test.

All 1891 complete periodic records used here pass individual CRC/time checks;
two malformed/interleaved lines are excluded. The finite helper fails its
runtime-health gate and observes a clean released-baseline return. Phone pause
is verified. This is not a continuous-audio or feature-parity pass. The next
finite comparison changes decoder service priority while keeping audio and
controller settings the same.
