# Independent ESP32 Bluetooth host

The project goal is a small, independently authored Bluetooth Classic + BLE host,
with simultaneous operation through the original ESP32's Espressif dual-mode
controller. BTstack is inspected only in `../reference/btstack`.

Current milestone: a portable C11 HCI/ACL/Basic L2CAP runtime, GAP setup, LE ATT/GATT,
Classic security and an SDP server, with an isolated ESP-IDF 5.5.2 probe.
Controller-to-host ACL credits and bounded SCO HCI framing are also implemented;
voice connection/profile ownership remains pending.
Owned AVDTP signaling, capability validation and an RTP media-header view are
implemented alongside an asynchronous sink acceptor and SBC configuration
validator. Production media/decoder integration and playback remain pending.
See [AVDTP scope](docs/AVDTP.md) and [sink lifecycle](docs/AVDTP_SINK.md).
This is **not yet a complete Bluetooth host or a product audio
firmware replacement**. The staged audio application still uses Bluedroid and
includes the separately requested integer DSP and codec-memory changes.

The GitHub host checkpoint includes the owned host/probe, native tests and their
portable DSP C test dependency. Audio-application/DSP-wrapper/SDK codec changes
described in the local progress report are separate work and are not a completed
application migration in this checkpoint.

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

There are 130 native groups, including the concurrent queue test. Run
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

See [release gates](docs/RELEASE_READINESS.md) before publishing a replacement
firmware. This milestone remains an incomplete development implementation.

See [architecture and layer review](docs/architecture.md) for the implementation
order, actual gaps, lazy codec requirements, and performance acceptance criteria.

The latest state is in [current progress](docs/PROGRESS.md), including the LHDC V5,
integer DSP and immediate-NVS requirements. The audio application now has staged
DSP changes authorized separately from the host migration.
