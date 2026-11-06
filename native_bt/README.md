# Independent ESP32 Bluetooth host

The local [diagnostics dashboard](docs/SERIAL_DASHBOARD_2026_11_05.md) provides
background collection, separate pipeline-stage views, raw/JSON inspection,
capture replay and export. `tools/start_serial_dashboard.ps1` prints its URL.
Release COM6 in the viewer or through `serial_monitor_ctl.py` before flashing;
the viewer remains open and follows the test capture while the port is free.

The [BTstack code audit](docs/BTSTACK_CODE_AUDIT_2026_11_05.md) now tracks
all 287 owned implementation/build inputs, confirmed layer defects, unwired modules and
remaining review coverage against a fresh pinned upstream checkout. The
comparison is in progress; file presence or compilation is not feature parity.
The [codec build-profile review](docs/CODEC_BUILD_PROFILE_2026_11_05.md) restores
-O2 for SBC/AAC/LDAC/aptX/Opus, matching the working baseline; LHDC remains -O3.
The [SBC CRC change](docs/SBC_CRC_COMPARISON_2026_11_05.md) keeps every validation
pass and reduces checksum arithmetic. The [FIR high-word change](docs/FIR_HIGH_WORD_2026_11_05.md)
preserves the filter exactly while reducing coefficient-interpolation work.
[LDAC working storage](docs/LDAC_RETAINED_WORKSPACE_2026_11_05.md) is now retained
across resets with bounded allocation and explicit teardown ownership.
[Over-limit signaling and dynamic L2CAP](docs/L2CAP_OVERSIZED_STREAM_2026_11_05.md)
now drain excess data without payload allocation and retain required signaling
MTU replies. [Fixed ATT/SMP length policy](docs/FIXED_PROTOCOL_LENGTH_2026_11_05.md)
now uses explicit owner errors and deferred teardown without fabricated short PDUs.
Wire qualification remains incomplete.
[PCM span copies](docs/AUDIO_SPAN_COPY_2026_11_05.md) now replace nested byte/frame
copy loops and single-writer counter retries while preserving publication order.
[Direct output-span processing](docs/AUDIO_DIRECT_RENDER_2026_11_05.md) now removes
the intermediate render block and post-DSP copy, with the same final publication
fence. The target render function own frame falls from 1120 to 112 bytes;
playback timing and full-chain headroom remain unqualified.
[Exact zero-tail FIR work](docs/FIR_ZERO_TAIL_2026_11_05.md) now omits two universally
zero taps’ work per output frame and fixes failed renderer initialization cleanup.
The full coefficient bank, filter delay, history and precision remain unchanged.
[DSP analysis averaging](docs/DSP_ANALYSIS_AVERAGE_2026_11_05.md) now uses exact
native-word division without changing analysis samples, cadence, state or tables.
[Classic setup retirement](docs/CLASSIC_CONNECT_RETIREMENT_2026_11_05.md) now keeps
sent requests and signaling pins associated with their lifetime during close, and
preserves the absolute Pending limit. Outgoing signaling source entry points are
available; automatic peer/paging/SDP/endpoint coordination remains unfinished.
The [owner connection coordinator](docs/A2DP_CONNECT_COORDINATOR_2026_11_05.md)
now joins caller-selected paging, SDP and signaling through normal polling,
with explicit cancellation/routing fences and separate per-layer results.
Default remembered-peer policy and local endpoint negotiation remain unfinished.
[Read-only endpoint queries](docs/AVDTP_PEER_QUERY_2026_11_05.md) now isolate
Discover/capability/configuration responses from Delay Report handling, with
caller-buffered complete results and explicit completion/cancellation ownership.
[Initial local Set Configuration](docs/AVDTP_LOCAL_CONFIGURE_2026_11_05.md) now
reserves its command until decoder READY and retains ownership through peer
acceptance or acknowledged cleanup. Configuration notifications precede metadata
release, and failed profiles still drain worker resources. No automatic caller
is enabled. [Local Open and outgoing media](docs/AVDTP_LOCAL_OPEN_2026_11_05.md)
now have explicit source APIs, preserving configured state on rejection and
binding the reserved media channel on its OPEN event. Final drain completion
wakeups are corrected. [Local Start](docs/AVDTP_LOCAL_START_2026_11_05.md) now
checks exact channel lifetimes, commits on peer acceptance and waits for
acknowledged renderer rollback on rejection/cancel. Worker, rollback and DROP
results stay separate. No automatic caller is enabled. Peer selection and the full crossed-control/recovery policy remain unfinished.
[Local Abort](docs/AVDTP_LOCAL_ABORT_2026_11_05.md) now joins independent wire,
decoder and bound/pending-media retirement. Existing procedures return BUSY
without accepting a second intent. No automatic Abort caller is enabled.
[Local Reconfigure](docs/AVDTP_LOCAL_RECONFIGURE_2026_11_05.md) now preserves
committed settings until peer acceptance and restores them through an explicit
worker operation on rejection/cancel, without two live decoder workspaces.
No automatic Reconfigure caller is enabled.
[Local Close](docs/AVDTP_LOCAL_CLOSE_2026_11_05.md) now joins decoder release and
media retirement after peer acceptance. Full channel reason/status reaches
the profile, so an error on a previously open channel cannot look like success.
No automatic Close caller is enabled.
[Local Suspend](docs/AVDTP_LOCAL_SUSPEND_2026_11_05.md) now sends first and
keeps media admission active while awaiting peer acceptance, then completes
local stop through an acknowledged worker job. Exact unsent cancellation
retains prior state; no automatic Suspend caller is enabled.
[Exact-format matching](docs/A2DP_EXACT_SELECTION_2026_11_05.md) now checks
current audio Source discovery/capability receipts against an explicit codec
CIE and both endpoint validators. It returns a proposal without changing app
preferences or transmitting. [Requested-family generation](docs/A2DP_CANDIDATE_GENERATION_2026_11_05.md)
now constructs supported candidates from strict rate/channel and codec-specific
constraints. [Ordered-request ranking](docs/A2DP_ORDERED_RANKING_2026_11_05.md)
now drives bounded asynchronous discovery/capability assessment with cancellation
fences and explicit preference order. It returns a proposal without configuring
a stream. Automatic peer/reconnect policy and sustained performance remain unqualified.

## Current development checkpoint — 2026-11-05

The owned Classic+BLE host is selected on the local COM6 board in ota1 at
0x390000, marked VALID, with automatic Bluedroid restoration disabled. The
verified image is `6b698779ec31828a722ba2a589f59e16919e2a1d8fa7445a2f8cd21b4750eac6`.
This remains development firmware with incomplete feature/runtime qualification.

Recent source corrections cover [codec payload boundaries and shared PCM
liveness](docs/CROSS_CODEC_PAYLOAD_AUDIT_2026_11_05.md), [exact FIR
interpolation](docs/FIR_INTERPOLATION_2026_11_05.md), [worker-fault session
recovery](docs/A2DP_WORKER_RECOVERY_2026_11_05.md), [AVDTP report
deadlines](docs/AVDTP_DELAY_SCHEDULING_2026_11_05.md), and [Classic
identity admission](docs/CLASSIC_PAGING_REVIEW_2026_11_05.md).
[Incoming setup ownership](docs/INCOMING_CLASSIC_OWNERSHIP_2026_11_05.md) now
retains decision/callback/final-event lifetimes and logs each stage separately,
using 88 additional static bytes and the unchanged log queue. The encoded
decoder queue remains 1800 bytes × 8 slots; every codec retains its own framing.

[PCM diagnostics](docs/PCM_DIAGNOSTIC_COST_2026_11_05.md) now omit full-byte
PCM hashing from normal stay-native playback. Use `-PcmHash` for explicit
fingerprint collection; finite legacy diagnostics retain it by default. The
legacy UART viewer also uses the actual codec name instead of labeling every
shared PCM record as SBC. Runtime performance improvement remains unmeasured.

The [DSP branch update](docs/DSP_ACTIVE_BRANCHES_2026_11_05.md) stops running
fully inaudible full-range/crossover paths after bypass transitions. Both
branches remain active during the existing fade. This removes unnecessary
filter work; playback headroom and acoustic transitions remain unqualified.

The [core layout and baseline comparison](docs/DECODER_CORE0_2026_11_05.md)
now reserve CPU0 for the shared codec worker at priority 21. Bluetooth host
priority 22, DSP/rendering priority 20, and app background workers use CPU1.
Separate decoder-wait records distinguish media starvation from PCM capacity
and control retirement. Playback improvement remains unqualified.

The new [Classic HCI paging procedure](docs/CLASSIC_PAGE_PROCEDURE_2026_11_05.md)
compiles for ESP32 and portable C11. It has a 48-byte caller context, no heap or
new task, separate admission/completion/cancel results and bounded deadlines.
[GAP routing and handover](docs/GAP_PAGING_INTEGRATION_2026_11_05.md) now retain
controller fences, reserve capacity and separate incoming/outgoing outcomes.
This adds 20 static bytes. Product peer selection and A2DP initiation remain
pending; no automatic paging context is instantiated.
The [SDP client and channel adapter](docs/SDP_CLIENT_2026_11_05.md) now compile
with bounded caller storage and continuation/transport retirement handling.
Their 108-byte/28-byte contexts are now included in a caller-owned
[Audio Source discovery operation](docs/SDP_DISCOVERY_2026_11_05.md), which
opens/routes the exact client channel, extracts a supported service and retires
transport ownership before returning a terminal result. Its context is 220 bytes;
default static storage grows by a four-byte routing pointer. SDP errors now close
only the affected channel through the owner task. Automatic policy and outgoing
AVDTP setup remain pending; no discovery instance is automatically started.

Builds use ccache. Recent checks establish compilation, source integrity,
flash readback and ordinary startup, including the firmware's built-in codec
self-checks; functional test campaigns are currently
stopped at the user's request. Sustained maximum-rate LHDC headroom, the
reported codec crashes, full feature parity and production readiness remain
unproven. See [the whole-stack review](docs/STACK_REVIEW_2026_11_05.md).
The experiment history below is scoped to its recorded images.

## Earlier ESP-IDF 6.1 experiments

The current work is on `codex/owned-bt-idf61`, based on the released Bluedroid
6.1 baseline. Read [the current audit](docs/IDF61_RESTART.md) and
[the full master goal](docs/MASTER_GOAL_2026_11_04.md) first. The imported owned
implementation is not a production replacement. Current tests and build evidence
are kept separate from historical observations below. The
[current target codec checkpoint](docs/IDF61_TARGET_CODECS_2026_11_04.md) records
37/37 configurations passing on COM6. The later
[LHDC profile and optimization](docs/IDF61_LHDC_PROFILE_2026_11_04.md) retains
identical PCM hashes and passes all 252 measured isolated 5 ms deadlines at
192 kHz/stereo/S24/1000 kbps, including validation. Full DSP/I2S/coexistence
qualification remains open. All finite runs returned to released 1.0.4 firmware. The user released
COM6 for testing in this chat; the historical reservation below is superseded.

The latest [host status fix](docs/HOT_STATUS_QUERIES_2026_11_04.md) removes
repeated diagnostic renderer-stack scans from the host's failure and running
checks, keeping complete periodic telemetry. Local equivalence/sanitizer/target
checks pass. Maximum-LHDC RF shows lower sampled audio-poll cost, but remaining
host polling, core1 idle starvation, underruns and a watchdog-adjacent DMA fault
still fail continuity. Fair host/decoder service and full-rate headroom remain
open; COM6 returns to released firmware and the phone is paused.

The opt-in [per-core scheduler measurement](docs/SCHEDULER_SAMPLING_2026_11_04.md)
now separates interrupted host/decoder/renderer/controller/logger/idle samples
and records tick-gap endpoints. The first maximum-LHDC startup/fault window
shows host work dominating core1 while decoder samples are scarce and core0
is mostly idle. Tick gaps on both cores correlate with the watchdog/DMA event;
earlier queue drops remain separate. Sampling is not exact CPU utilization.
Host phase profiling and fair service remain open; audio qualification fails.

The latest [1800-byte/eight-slot decoder queue trial](docs/DECODER_INPUT_1800_2026_11_04.md)
tests the requested larger input capacity with a matching media MTU. Maximum
LHDC negotiates and I2S runs briefly, but the observed largest media SDU is only
640 bytes and both size-rejection counters are zero. Queue saturation remains.
One core1 watchdog and a nearby 11.741 ms DMA EOF gap fail runtime qualification;
the released baseline is restored. Separate logs distinguish packet sizes,
queue admission, decoder work, PCM delivery, rendering and DMA faults.

Latest phone evidence separates the [maximum LHDC radio/decoder pass](docs/RF_LHDC_MAX_2026_11_04.json)
from the [failed full-output attempt](docs/RF_LHDC_MAX_OUTPUT_2026_11_04.json).
Android confirmed LHDC V5 at 192 kHz/S24/stereo with the 1000 kbps target, but
the full product could not allocate another 4096-byte PCM page (7540 bytes free,
largest block 3072). No maximum-rate DAC playback pass is claimed.
The [priority2 I2S test](docs/I2S_IRQ_PRIORITY_2026_11_04.md) avoided the prior
descriptor-order fault during a finite LDAC96 overload test; renderer overruns,
queue drops, underruns and core0 watchdog warnings remain unresolved. COM6
returned to released 1.0.4-idf6.1 ota0 and phone playback was paused.

The later [page-sized output ring](docs/OUTPUT_CAPACITY_2026_11_04.md) removes
28 KiB of required allocation at 192 kHz while retaining the 64 ms target and
prefill margin. LDAC96 DMA ran without descriptor errors, but underruns and
watchdog warnings remain. Maximum LHDC still fails memory admission. Separate
[CRC-validated records](docs/CAPTURE_CRC_2026_11_04.json) now identify its exact
requested format and allocation failure; damaged records are excluded.

The [core1 logger follow-up](docs/RF_LOGGER_CORE1_2026_11_04.json) retains live
snapshots during overloaded LDAC96 playback; the longest observed reporting
gap fell from 32.057 s to 2.492 s. Audio still underruns. The
[optional PCM-loan bridge](docs/PCM_RENDER_LOAN_BRIDGE_2026_11_04.md) now connects
the worker, renderer and sound paths behind `-PcmLoans`. Its first COM6 trial
removes 32 KiB of input allocation at LDAC96 and retires 4882 loans without
request/stale-ticket/descriptor errors. Underruns and watchdog warnings remain;
maximum LHDC still fails output-page allocation. The copied path remains the
default and no maximum-rate output/headroom or full feature-parity pass is claimed.

The [unused LHDC snapshot removal](docs/LHDC_UNUSED_OVERLAP_2026_11_04.md)
subsequently passes all 37 target codec configurations and three complete
maximum-rate decoder runs with unchanged target PCM hashes, saving 3840 payload
bytes. Its decoder-only deadline pass does not include DSP/I2S/BLE. The
[paired Xtensa FIR experiment](docs/FIR_PAIRED_PRODUCTS_2026_11_04.md) preserves
exact output/state across 27 target groups but is slower in every comparison
and stays disabled in production. The separate pipeline still fails before
DMA starts; maximum-rate playback is not qualified.

A separate [LHDC PCM storage reuse](docs/LHDC_PCM_REUSE_2026_11_04.md) removes
another 7680 bytes at 192 kHz by emitting into retired transform storage.
Three sets of 80 real-codec host cases pass, including copied and immutable
loan providers; complete PCM hashes and sample timestamps agree. Physical
decoder validation then confirms the 7680-byte saving, all 37 configurations,
unchanged maximum-rate hashes and zero measured isolated deadline misses.
The separate [full RF test](docs/RF_LHDC_PCM_REUSE_RESULTS_2026_11_04.json)
still fails maximum-rate output allocation (4096 requested, 6656 free,
largest block 1984). LHDC48 uses DMA but still underruns and triggers core0
watchdog warnings. Full-rate playback/headroom and feature parity remain open.

The later [allocation-progress test](docs/OUTPUT_ALLOCATION_PROGRESS_2026_11_04.md)
shows a real capacity deficit: 20 of 25 PCM pages allocate, leaving 20480 bytes
before DMA, against 10744 free 8-bit bytes. Word-only eligibility alone is also
insufficient. The processed-PCM queue needs a broader memory change; reducing
fragmentation alone cannot qualify the maximum-rate path.

An explicit, finite [16 ms PCM-target experiment](docs/PCM_TARGET_EXPERIMENT_2026_11_04.md)
then admits 192 kHz LHDC/I2S with a 31744-byte PCM/DMA payload. All codecs and
sample precision remain selected. Continuous playback still fails: render
wall time exceeds its frame budget, queues drop packets and core0 starves.
The 64 ms default remains unchanged, and jitter equivalence is unqualified.

The later [exact block-DSP renderer](docs/BLOCK_DSP_2026_11_04.md) is available
only with `-BlockDsp`. Controlled target kernels improve 12–13% with full EQ
and 33–36% with flat EQ. The full product confirms 192 kHz DMA and roughly
8.460 us/frame render wall time, still above the 5.208 us budget. Queue drops,
underruns and core0 starvation remain. EQ persistence and Balanced restoration
are hardware-verified; complete audio/jitter/product qualification remains open.

The [resumable PCM worker checkpoint](docs/IDF61_ASYNC_PCM_2026_11_04.md) adds
80 real-codec host comparisons with sanitizer coverage and 38 passing ESP32
deferred-worker checks. The render provider, stream/sound transactions and full
DSP/I2S/Bluetooth integration remain separate open gates.

The [ticketed renderer checkpoint](docs/IDF61_ASYNC_RENDER_2026_11_04.md) adds
nonblocking request/PCM APIs and an opt-in `-AsyncAudio` worker connection.
Host tests and the full ESP-IDF build pass; this renderer change has not run on
COM6 and the high-level sound client/render polling still need conversion.

The later [async sound-manager checkpoint](docs/IDF61_ASYNC_SOUND_2026_11_04.md)
converts that client and render waiting in the opt-in path, with host ownership,
continuity, cancellation and fault tests. Actual DSP/I2S/RF coexistence and RAM
qualification remain open; no Bluedroid fallback is retired.

## Historical pre-6.1 status


Current off-board evidence: [feature matrix](docs/FEATURE_MATRIX.md) and
[candidate receipt](docs/LOCAL_CANDIDATE_EVIDENCE.json). The 2026-11-03 build
fits the user-confirmed 8 MB COM6 layout, includes product IO and bonded
GATT subscription persistence, and has not been flashed or hardware-qualified.
The hardware hold remains active.

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

## Product-mode local build

From the workspace root, the existing SDK supports a build-only command:

```powershell
.\native_bt\tools\build_probe.ps1 -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -WithSbc
```

Product IO enables the owned product admission policy, so this command does not
require a hardcoded Linux test address. Explicit diagnostic Classic builds
retain the `-TestPeer` requirement. RFCOMM adds a bounded serial echo service;
its current single-DLC/conformance limits are in `docs/RFCOMM.md`. This command
never flashes. The ESP32 hold remains active, and future deployment still needs
fresh identity/layout admission through the guarded workflow.

### ESP-IDF 6.1 maximum-rate pipeline checkpoint

The [COM6 pipeline record](docs/IDF61_PIPELINE_2026_11_04.md) documents the SRAM-region allocation fix,
179 portable tests and exact-bounds ASan/UBSan checks. The full-quality LHDC
constructor now fits, but the render path misses its throughput budget before
DMA starts. The released fallback remains selected; production parity and the
master objective are incomplete.

The [render-stage profile](docs/IDF61_RENDER_PROFILE_2026_11_04.md) records the exact O3/FIR checks and observed
conversion improvement. DSP remains the main throughput bottleneck; the
maximum-rate fixture still fails before DMA playback starts.

The [controlled DSP comparison](docs/IDF61_DSP_VARIANTS_2026_11_04.md) compares identical Os/O3 input and
state on COM6. Os is selected for DSP after 18 matching, faster case pairs;
conversion/resampling retains O3. The maximum-rate pipeline remains unqualified.

The [exact DSP accumulation checkpoint](docs/IDF61_DSP_ACCUMULATION_2026_11_04.md) proves full-width arithmetic
and records the measured filter improvement. All 18 target output/state pairs
match; the maximum-rate pipeline still misses its deadline before DMA starts.

The [render reuse record](docs/IDF61_RENDER_REUSE_2026_11_04.md) retains exact FIR/input improvements and
records why slower stereo fusion and unhelpful IRAM placement were removed.
Maximum-rate playback and full feature parity remain unqualified.
