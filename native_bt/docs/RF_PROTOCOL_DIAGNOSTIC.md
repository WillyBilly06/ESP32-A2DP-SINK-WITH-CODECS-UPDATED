# Finite radio/profile isolation

This test mode isolates Bluetooth transport, negotiation, decoding, security and
product commands from the A2DP output failure recorded in
[the RF report](BLUETOOTH_RF_2026_11_04.md). It is not a replacement audio path.
The full streaming failures remain release blockers.

`build_probe.ps1 -RfProtocolDiagnostic -WithProductIo` retains the ordinary
codec endpoints, decoder worker, Classic/SMP/trust policy and product providers.
Select all applicable codec build flags as in the full product build. This mode
uses the bounded synchronous stream callback only to publish decoder state;
decoded PCM is counted/checksummed and is not forwarded to I2S. Normal product
forwarding and backpressure are unchanged when the option is off.

Product IO, including sound storage/playback, remains compiled and may still use
the output provider. No sound, DSP, meter accuracy, analog quality, DMA continuity
or full-workload CPU/SRAM advantage is qualified by this mode. An unrelated
output fault does not terminate the diagnostic A2DP profile. Codec-worker faults
and protocol/security failures retain their ordinary failure handling.

The build rejects asynchronous output, other diagnostic entries and persistent
boot selection. Deployment requires both `--finite-rf` and
`--rf-protocol-diagnostic`; its value must match the cache, build receipt and
actual compiler definitions. Readiness explicitly reports
`"i2s_output":false,"rf_protocol_diagnostic":true`. The preserved baseline is
verified again after the finite trial. Never publish this image as product audio.

The host test compiles the actual callbacks in normal synchronous, normal
asynchronous-PCM and diagnostic configurations. It checks equal PCM accounting
across 24 format/rate cases per mode, normal forwarding/backpressure, invalid
input handling, five incompatible source configurations and five actual CMake
configurations under ASan/UBSan. These are host checks, not radio evidence.

Before a firmware swap, close the BDK app's old GATT connection. Restart it for
the new image; record stale-handle errors separately. Only the verified COM6
phone entry may be re-paired. LE security metadata logs expose state/key size and
durable completion, never keys. A link or app screen alone does not establish
full security, command, lifecycle or stress qualification.
