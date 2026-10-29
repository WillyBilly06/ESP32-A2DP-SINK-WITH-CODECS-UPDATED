# Standalone LDAC codec candidate

The native LDAC adapter connects `dbt_ldac_codec` to the shared codec worker.
`esp32_probe/components/ldac_decoder` is a codec-only ESP-IDF component with
dependencies on `native_bt` and `codec_core`; it is not registered or advertised
by this component. A caller can include `dbt_ldac.h`, add `&dbt_ldac_codec` to
the worker's codec list, and use `dbt_a2dp_ldac_validate` for its endpoint after
completing integration and on-device qualification. The codec path uses SDK
decoder units only and does not route through the Bluedroid host wrapper.

The adapter accepts 44.1, 48, 88.2 and 96 kHz in mono, dual-channel or stereo,
with 32-bit PCM sample slots. The SDK decoder does not support 176.4/192 kHz.
The packet parser accepts one to 15 unfragmented LDAC frames per payload;
fragmented LDAC transport is currently rejected. The codec has process-global
bookkeeping and allows one active worker owner.

The generated, source-pinned SDK port allocates decoder channel and transform
state according to negotiated rate and channel count. The handle's planar PCM
is bound to the worker allocation, and the encoded-frame lookahead is bounded.
The port also makes Huffman descriptor and transform-pointer tables read-only.
The SDK's transform-shift table uses negative sentinels; Xtensa defaults to
unsigned plain `char`, so the generated table uses explicit `signed char`.
A failed reset now destroys partially constructed decoder state immediately;
the worker's separate packet queue remains until close/abort.

For real audio validation, a test-only copy of the Apache-2.0
[AOSP LDAC encoder at commit eeee1a3](https://android.googlesource.com/platform/external/libldac/+/eeee1a3f5f8df1282e3a6d297085885fd886737b/)
generates deterministic PCM fixtures offline. `prepare_ldac_fixture_encoder.py`
fetches only bounded encoder source into `_build`, checks a pinned canonical
tree digest, and refuses to overwrite differing files. The encoder is neither
linked into the device component nor copied into owned runtime sources.

`test_ldac_encoder_fixture.py` decoded 1,552 encoder-generated frames in 16
cases: SQ for all 12 supported rate/mode combinations and HQ/MQ stereo at
48/96 kHz. The native worker output matched the original SDK decoder byte for
byte; both channels' delayed waveform correlations to the generated inputs were
at least 0.995 for this finite fixture. It also checked per-frame sample counts,
PCM layout, and RTP timestamp progression across 32-bit wrap. A second host run
forced unsigned plain `char` and passed the same checks. Fixture alignment is not an end-to-end
latency measurement, and this is not a general fidelity or interoperability
qualification.

The separate lifecycle test checked 100 initialization allocation failures
and 52 reset allocation failures across the 12 configurations, successful reset
peaks, and SBC/LDAC switch rollback. Host worker live bytes ranged from 8,405 to
17,997; the tested 96 kHz stereo SBC-to-LDAC switch peaked at 17,869 bytes.
These are host allocator measurements, not target heap measurements.

`test_ldac_target_component.py` compiles the actual component recipe and shared
codec core with the existing ESP32 compiler, verifies the generated sources
were selected, checks the source pin and target read-only table sections, and
performs a relocatable link. The result has 0 bytes of data and 13 bytes of BSS;
the latter is single-owner bookkeeping, not a decoder working set. Compiler
stack-usage output gives the IMDCT core a 1,104-byte static frame; this does not
bound its complete call chain or establish a safe worker stack. Remaining SDK
warnings concern unused internal declarations and an unused local; they are
recorded in the target receipt. The relocatable object retains unresolved
toolchain/libc symbols and is not a complete firmware link. No ESP32 execution,
audio output, throughput, heap low-water, RF, or external-peer test was run.
