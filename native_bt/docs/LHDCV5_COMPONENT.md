# LHDC V5 worker component

`esp32_probe/components/lhdcv5_decoder` is the single supported LHDC V5 IDF
component. With `DBT_PROBE_LHDC_BUILD` enabled, it compiles the generated lazy
port and `dbt_lhdcv5_codec.c`, and depends on `native_bt`, `codec_core`, `log`,
`esp_common`, `esp_timer`, `heap`, and `freertos`. The former raw-session
adapter remains available for isolated host tests but is not included in the
component. Treating `codecs/lhdcv5` as a second IDF component fails explicitly
to prevent duplicate global cache owners.

A generic codec worker may include `dbt_lhdcv5_codec.h`, add
`&dbt_lhdcv5_codec` to its codec list, and associate the endpoint with
`dbt_a2dp_lhdcv5_validate`. The supported selection is stereo, 44.1/48/96/192
kHz, and 16/24/32-bit; 24-bit PCM uses 32-bit sample slots. An endpoint
capability CIE covering the adapter's accepted masks is
`00 ff 3a 05 00 00 35 4c 35 07 11 40 00`. Capability and subset validation
were host-tested for all 12 selections. The source component does not register
an endpoint or by itself establish interoperability with a peer.

The source-pinned generated port keeps the FAC model, transform buffers, PCM,
and fragment storage lazy and associated with the one active decoder owner.
Immutable reciprocal/window tables are in target read-only sections. The 960
and 1920 transform's small generated twiddles live in the active transform
allocation instead of permanent writable arrays; unused table-block space is
removed. The 480 transform's 3,840-byte temporary scratch is allocated only
for that active transform and freed at close. This reduces its ESP32 compiler
stack frame from 3,152 bytes to 272 bytes for the measured flags; those values
are individual static frames, not the full call-chain or a stack safety proof.
Decoder error and concealment logs are gated by the trace flag, which is off by
default. Target execution and decode throughput are still unmeasured.

For actual encoded input, `prepare_lhdcv5_fixture_encoder.py` fetches a bounded
test-only copy of the Apache-2.0 [portable LHDC V5 encoder at commit 3f9d198](https://github.com/WillyBilly06/LHDC-V5-Encoder/tree/3f9d1980fa9c57cdfb78792f9455a7dbac50edda),
checks a pinned canonical tree hash and refuses to overwrite differing files.
It does not clone an SDK or link the encoder into firmware.
`test_lhdcv5_encoder_fixture.py` encoded deterministic stereo tones at all
four rates in 16- and 24-bit input, then decoded 768 frames through the native
worker. PCM was byte-identical to the unmodified sibling decoder for these
fixtures. Both tone channels exceeded 0.98 correlation after searched alignment;
the same test replayed a valid fragmented packet per configuration and checked
frame timestamps across 32-bit wrap. These finite tone fixtures do not prove
music fidelity, external-peer behavior, acoustics, or end-to-end latency. The
32-bit selection has lifecycle coverage but no separately encoded 32-bit input.

The host lifecycle test exercised all 12 configurations and 60 selected open
allocation failure sites across four rates at 24-bit, plus fragment expiry,
allocation failure, and SBC/LHDC switch rollback. The 192 kHz/24-bit worker
live value was 59,960 bytes; its tested SBC-to-LHDC switch peak was 63,624
bytes, and a valid fragmented replay peaked at 62,928 bytes. These numbers are
host allocator measurements and do not establish ESP32 heap availability.

`test_lhdcv5_target_component.py` compiles the actual worker recipe and codec
core with the existing ESP32 compiler, checks that the second component path
and modified source are rejected, and makes a relocatable link. The measured
object is 63,091 text, 0 data, and 237 BSS bytes; it has unresolved SDK and
toolchain symbols and is not a complete firmware link. No flash, radio, device
playback, target stack low-water, codec performance, or external-peer test has
been performed for this integration.
