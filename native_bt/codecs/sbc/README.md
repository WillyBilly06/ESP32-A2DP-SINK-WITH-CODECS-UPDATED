# Standalone SDK SBC backend

The owned wrapper and worker build only the existing SDK's SBC codec sources.
They do not use Bluedroid's A2DP wrapper, protocol state, scheduler or public APIs.
The codec files remain in `esp-idf/components/bt/host/bluedroid/external/sbc/decoder`;
their per-file Open Interface/AOSP copyright and Apache-2.0 notices remain intact.
BTstack is not a dependency.

`prepare_port.py` generates three bounded arithmetic-port translation units under
the reusable build directory, preserving the original notices and recording
original/generated hashes. The SDK is unchanged. Fixed-width types and explicit
wrapping arithmetic are shared between Linux and ESP32 tests.

Run the normal core tests, then `test_sbc_linux.py` for actual encoded-frame PCM,
allocation and sanitizer tests. `build_probe.ps1 -WithSbc` compiles this component;
`-WithAudioProbe -WithClassicSecurity -TestPeer <address>` enables the restricted
PCM-counter diagnostic. See `docs/SBC_PIPELINE.md` for evidence and limits.
