# Independent ESP32 Bluetooth host

The project goal is a small, independently authored Bluetooth Classic + BLE host,
with simultaneous operation through the original ESP32's Espressif dual-mode
controller. BTstack is inspected only in `../reference/btstack`.

Current local candidate includes the independently authored HCI/ACL, Basic and
credit-based L2CAP, required-mode ERTM, GAP, ATT/GATT, Classic security, SMP,
SDP, AVDTP/A2DP and AVRCP control/metadata paths. The codec worker selects SBC,
AAC-LC, aptX/HD/LL, Android Opus/Opus05, LDAC and LHDC V5 without retaining the
other decoder working sets. Fixed-point EQ and adaptive clock correction run
outside the host owner; immutable FIR/codec tables remain in flash.

The owned BLE service now carries the baseline app UUIDs and EQ/control/status/
meter protocol. Each accepted EQ update has its own immediate NVS commit;
there is no debounce or coalescing. Other product providers and profile roles
remain incomplete. See [the feature matrix](docs/FEATURE_MATRIX.md) for the
implemented, host-tested, target-linked and missing pieces.

**This is not a complete or production-qualified replacement.** The last device
candidate passed image readback but only 19/37 codec checks; five comparisons
failed and thirteen later Opus cases did not run, so Bluetooth stayed disabled.
The AAC host-math and Opus arithmetic-profile oracles have since been corrected
locally and remain unconfirmed on device. Historical RF/audio evidence belongs
to its stated earlier firmware identities.

The user has reserved the ESP32. All further testing is off-board until explicitly
released: the serial reader is released, and deployment/reset/automatic reconnect
is held. Local builds and checksum/partition checks establish flashable image
format and capacity, not runtime, coexistence or acoustic performance. Current
local evidence is in [LOCAL_CANDIDATE_EVIDENCE.json](docs/LOCAL_CANDIDATE_EVIDENCE.json).
The full project goal remains active; no equivalent-feature advantage over
Bluedroid or BTstack is claimed. BTstack remains source-reference-only.

| Path | Purpose |
| --- | --- |
| `include/`, `src/` | Independently authored portable core |
| `tests/` | C behavior tests, mocked controller/transport, real concurrent queue test |
| `esp32_probe/` | Controller-only ESP32 build, explicit source dependencies |
| `tools/` | Fixed-path build, tests, size/dependency checks and Linux validation |
| `docs/` | Architecture, upstream layer review, coverage and measurement criteria |
| `_build/` | Ignored, reusable generated outputs; no SDK copies |

From the workspace root:

```powershell
python native_bt/tools/test_host.py
.\native_bt\tools\build_probe.ps1
python native_bt/tools/check_workspace.py
```

The test runner compiles and executes the actual C code, including a 20,000-packet
test using separate producer/consumer threads. Its one current receipt is
`_build/host/results.json`; it includes input hashes. Windows testing uses Clang
without requiring a Visual Studio CRT installation. The target build uses the
existing adjacent SDK and the already installed ESP-IDF toolchain.

The probe performs eight bounded, asynchronous identification commands: Reset,
version, supported commands/features, address, Classic buffer capacity, and LE
features/buffer capacity. `IDENTIFIED` means these responses were validated, not
that pairing, GATT or A2DP works. It next completes receive-buffer/flow setup,
then configures event masks, host support,
name/class, SSP enable and legacy advertising parameters/data using the general
command queue. It does not enable advertising or Classic scanning.
The probe is not automatically flashed and uses its own partition layout.

`dbt_runtime` joins the link manager, lazy reassembly, Classic Basic channel
negotiation and gated ACL TX. Tests connect two instances over a simulated
controller transport, transfer fragmented data, and disconnect. They also stall
Classic credits while LE data/control continue. `dbt_gap` has explicit incoming
admission and security-owner hooks; no bonding/security/profile implementation is
implied by enabling SSP in the controller.

There are 136 native groups, including the concurrent queue test. Run
`python native_bt/tools/test_stack_linux.py --host HOST --user USER` for Linux
normal/ASan/UBSan checks; the password prompt is session-only. The default probe
does not require the separate LHDC checkout. `build_probe.ps1 -WithLhdc` enables
its optional decoder compilation without adding it to the host loop.

`build_probe.ps1 -WithGatt` enables the independent diagnostic LE ATT server.
Live Linux testing passed discovery, 200 writes with periodic readbacks, payloads
through 512 bytes, a 512-byte reliable write, and 208 notifications. The measured
BLE timings are in [ATT/GATT evidence](docs/ATT_GATT.md); no audio, bonding or
simultaneous Classic/BLE pass is implied. Product control/OTA binding is pending.

`build_probe.ps1 -WithGatt -WithClassicSecurity -TestPeer AA:BB:CC:DD:EE:FF`
explicitly enables restricted diagnostic Classic pairing, asynchronous NVS bonds,
encrypted Basic echo and public SDP discovery. Select the actual test adapter;
the example address is a placeholder. Just Works pairing/encryption and live
[SDP discovery](docs/SDP.md) passed. Simultaneous Classic/LE traffic still fails;
these diagnostic services contain no audio profiles.

Receive buffers default to three 1024-byte ACL packets in the existing bounded
FIFO. See [receive flow and hardware limits](docs/RX_FLOW.md). A short concurrent
run passed; the longer untraced workload still failed. The default probe has no
per-packet ingress fingerprints; `-TraceIngress` enables the bounded diagnostic
snapshots. `-WithoutRxFlow` is an explicit diagnostic comparison option.

`-WithSbc` compiles the standalone SDK codec and owned lazy worker without the
Bluedroid host. `-WithAudioProbe -WithGatt -WithClassicSecurity -TestPeer <address>`
adds the restricted direct-AVDTP diagnostic. It hashes PCM and has no I2S output
or Audio Sink SDP advertisement. The finite 48 kHz stereo test decoded 8000 frames
while 166 ATT writes/notifications completed; this does not qualify other codecs,
Firefox playback or resolve the large-payload generic dual-mode failure.

`-WithAudioOutput -WithGatt -WithClassicSecurity -TestPeer <address>` additionally
enables the native integer renderer, I2S on GPIO27/25/26, an Audio Sink SDP record
and queue-estimated delay reporting. Its linked ISR roots/direct references are
audited for IRAM/internal SRAM; this does not qualify physical NVS continuity.
Codec/input/output storage is released on close, and session retirement gates
readmission until the old worker and channels drain. This remains experimental.

See [release gates](docs/RELEASE_READINESS.md) before publishing a replacement
firmware. This milestone remains an incomplete development implementation.

See [architecture and layer review](docs/architecture.md) for the implementation
order, actual gaps, lazy codec requirements, and performance acceptance criteria.

The latest state is in [current progress](docs/PROGRESS.md), including the LHDC V5,
integer DSP and immediate-NVS requirements. The audio application now has staged
DSP changes authorized separately from the host migration.
