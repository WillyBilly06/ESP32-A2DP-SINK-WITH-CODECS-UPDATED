# Standalone AAC and aptX target components

`esp32_probe/components/aac_decoder` and `aptx_decoder` expose the native
codec adapters through the existing `codec_core` and `native_bt` components.
Both recipes list codec sources explicitly and reuse the current SDK codec
sources. Neither depends on the SDK's Bluedroid host or its A2DP wrapper.
The caller enables the components in its own project configuration; these
recipes alone do not register or advertise an A2DP endpoint.

For AAC, the recipe runs `codecs/aac/prepare_port.py` against the pinned Helix
`aacdec.c` source and compiles the generated guarded unit, never the original
unit as a second decoder. Compilation uses `ESP32=1`, `USE_DEFAULT_STDLIB`,
`AAC_MAX_NCHANS=2`, `-std=c11`, and `-fwrapv`. The port refuses SBR and the
adapter checks the guard symbol. For aptX, the backend includes the existing
SDK `freeaptx.c` exactly once; it is not compiled separately. Its recipe uses
`-std=c11` and `-fwrapv`. The existing aptX adapter's two compact control-flow
lines have a narrowly scoped GCC misleading-indentation warning exemption.

For a worker, include `dbt_aac.h` and `dbt_aptx.h` and add `&dbt_aac_codec`
and `&dbt_aptx_codec` to its codec-ops list. Associate the AAC endpoint with
`dbt_a2dp_aac_validate` and each of the standard, HD, and LL vendor endpoints
with `dbt_a2dp_aptx_validate`. The generic worker's configure path chooses the
matching codec and only its active configuration allocates its working state.
The supported CIE ranges are AAC-LC 44.1/48 kHz mono/stereo and aptX/HD/LL
44.1/48 kHz stereo. AAC emits 16-bit samples; aptX emits 32-bit sample slots.
The AAC state and PCM workspace retain two-channel Helix bounds even for a
mono negotiation, to preserve the decoder's output bound on malformed input;
the raw input allocation is sized to the negotiated channel count.
LL currently accepts only the CIE with its final extension byte zero and uses
the standard aptX decode path; host PCM comparisons do not establish the LL
latency protocol or external-peer interoperability.

The Windows host real-frame tests pass for AAC mono/stereo at both rates and
aptX standard/HD/LL at both rates against their standalone SDK decoder
references. They exercise codec switching, selected allocation failures and
fragment handling. The measured host AAC worker live/switch peak was 30,524
bytes; direct AAC fragment growth reached 38,692 bytes. The measured host aptX
worker live/switch peak was 9,608 bytes. These are host allocator measurements,
not ESP32 heap or playback measurements.

`test_native_codec_target_components.py` reads the actual component recipes,
compiles their 21 units plus codec core and AVDTP capability parsing with the
existing ESP32 compiler, and performs a relocatable link. It rejects an
unreviewed AAC source and SBR build; selected AAC and aptX tables are in target
read-only sections and the combined object has no named writable static
symbols. The compiler reports a 640-byte `decode_mux` frame calling a 592-byte
`dbt_aac_latm_parse` frame; these static frame sizes are not a complete call
chain or proof of a safe worker stack. The relocatable result has unresolved
runtime/library symbols and is not a complete firmware link. No device codec
playback, memory low-water, throughput, latency or RF behavior has been tested.
