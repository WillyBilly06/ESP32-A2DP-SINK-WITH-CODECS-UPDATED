# Independent ESP32 Bluetooth host

The local [diagnostics dashboard](docs/SERIAL_DASHBOARD_2026_11_05.md) provides
background collection, separate pipeline-stage views, raw/JSON inspection,
capture replay and export. `tools/start_serial_dashboard.ps1` prints its URL.
Release COM6 in the viewer or through `serial_monitor_ctl.py` before flashing;
the viewer remains open and follows the test capture while the port is free.

The [Bluedroid PCM comparison](docs/BLUEDROID_PCM_COMPARISON_2026_11_06.md) identifies
the added always-on FIR work absent from the working product. Stay-native now
restores one-to-one PCM through integer DSP. The opt-in codec-aware core policy
shares Core0 with lower-priority DSP for lighter codecs; LHDC/Opus retain Core1
rendering. Read the measured hardware limits in that record before treating this
development image as a replacement. COM6 runs the owned host without automatic
Bluedroid fallback.

The [BLE/Classic investigation](docs/BLE_CLASSIC_COEX_2026_11_06.md) records
a reproduced SMP consent mismatch, explicit VHCI submission ownership and scoped
Classic re-pairing recovery. BTDM was already enabled. Selected BTstack sections
have been compared; encrypted GATT/audio coexistence and all-feature parity are
still being qualified. [First-starvation captures](docs/FIRST_STARVATION_2026_11_06.md)
keep media, decoder, renderer and DMA evidence separate.

The [receive-credit follow-up](docs/RX_CREDIT_BACKPRESSURE_2026_11_06.md) couples
Classic completions to downstream encoded-queue capacity. Two explicitly selected
maximum-quality LDAC/BLE EQ traces have no packet drops, but refill underruns
remain. The [scheduling/startup record](docs/SCHEDULING_STARTUP_GUARD_2026_11_06.md)
reports the40–47ms AAC/LDAC guard, unchanged buffers and limited passing AAC runs.

The [local Bluedroid codec comparison](docs/BLUEDROID_CODEC_ROUTING_2026_11_06.md)
distinguishes controller-only host linkage from reused codec libraries and
records the actual baseline wrappers. It corrects raw aptX Classic/LL framing,
selects encoded queue depth per codec, and separates PCM scratch from queued
packets. [DMA refill ownership](docs/DMA_REFILL_WINDOW_2026_11_06.md) records the
conservative recovery lease and its limits. The accompanying inventory leaves unreviewed files pending;
this is not a completed host-layer or all-codec qualification.

[aptX parity-phase recovery](docs/APTX_PHASE_RECOVERY_2026_11_06.md) now uses
bounded resynchronization after resetting a continuing stream. Focused loss,
parity-error and resume comparisons pass; live HD resume and raw Classic aptX
traces are recorded separately from acoustic and full-feature qualification.
The [BLE parameter coordinator](docs/BLE_PARAMETER_POLICY_2026_11_06.md) restores
the original requested policy through HCI or LE signaling and logs actual
controller parameters. Two HD/BLE settings runs have clean transport counters;
the user subsequently reports aptX HD and LDAC sound good on image92fc7e5705b7.
That is acoustic feedback, with other rates/features still unqualified.

The opt-in [PCM boundary capture](docs/APTX_PCM_BOUNDARIES_2026_11_06.md) matches
one live HD decoder call, its copied input, and an independent DSP/final block
against desktop replay. It also records a 440-frame payload versus 336-tick RTP
discrepancy. It is not a causal diagnosis of that sound report; capture is off by default.

[DSP completion wake order](docs/ASSIST_COMPLETION_WAKE_2026_11_06.md) now signals
the Core1 owner before notifying the higher-priority Core0 decoder. The pinned
kernel confirms the preemption path; a negative/two-core sanitizer regression
proves the owner can retire/close while the helper continuation is suspended.
COM6 normal image a9427d8cafde passes readbacks/37 cold cases, adds16 flash bytes
and no RAM. Short LHDC192/fixed900 traces have206848/179968 unfilled versus232192
in the fresh baseline; an isolated speedup is not established. LDAC96/observed
990kbps has1451264 source/DMA frames and zero shortages. Full continuity remains open.

[DMA refill during helper work](docs/ASSIST_DMA_REFILL_2026_11_06.md) now consumes
already-published PCM while the helper retains its DSP/lifecycle borrow, and
wakes the owner on a publication when a descriptor is pending. A valid negative
control reproduces the missed refill; sanitizer, two-core lifecycle and sound
regressions pass. Previous normal image6e4c4932cf28 adds68 flash bytes/no RAM and
passes guarded readbacks/37 cold cases. Two LHDC192/fixed900 traces record
182400/200320 unfilled frames versus237504 before; continuity still fails.
LDAC96/observed990kbps metronome delivers1443584 frames with zero shortages.

[Corrected host CPU attribution](docs/HOST_DEADLINE_CLOCK_2026_11_06.md) prevents
background boot logging from altering the host phase. The opt-in separate host
counter measures runtime/deadline work; old sampled console counts are not real
console CPU evidence. A shared-clock candidate gives no meaningful gain and is
reverted. First shortage has1920 decoded frames waiting and0 processed output.
Previous normal COM6 image ea790bbaaa24 passes readbacks/owned boot/37 cold codec cases,
adds72 flash bytes and no RAM versus6e7afd71e8a0, with temporary hooks/storage
absent. LHDC continuity remains unresolved; the last normal playback evidence
below remains associated with its earlier exact image.

[Sound-space wakeups](docs/SOUND_PROGRESS_WAKE_2026_11_06.md) now signal PCM
progress only for a live sound lease. Completion, failure, cancellation and
retirement signals remain unconditional; actual async-sound and two-core tests
pass. Previous normal COM6 image 6e7afd71e8a0 adds 48 flash bytes and no static RAM.
LHDC 192/fixed900 still has 262656 unfilled frames; normal-build improvement
is not established. LDAC 96 metronome has zero shortages, with sampled wire
data consistent with 990 kbps. Optional [task-cycle accounting](docs/RENDER_TASK_CYCLES_2026_11_06.md)
measures running tails and switched-away intervals, but is disabled in this
normal image. Full continuity and all-feature qualification remain open.

[Current instruction-RAM DSP measurement](docs/DSP_IRAM_CURRENT_2026_11_06.md)
retains the four-block/fixed-transfer baseline and places the existing integer
block kernel in instruction SRAM (+3756 bytes, unchanged data RAM). COM6 image
d2676c584056 passes readbacks/37 cold cases. Two LHDC 192/fixed900 metronome
traces have 254464/269824 unfilled frames versus 359552 in the baseline; local
elapsed DSP cost falls, but continuity still fails. LDAC 96 metronome regression
has zero DMA shortages; sampled wire timing is consistent with 990 kbps.
The [smaller-lease/copy-credit trials](docs/PCM_COPY_CREDIT_2026_11_06.md) show
no benefit and are removed. Corrected LHDC framing fixtures pass all 80 real-codec
host cases. Full maximum-rate and all-feature qualification remain open.

[Exact block analysis](docs/DSP_ANALYSIS_BLOCK_2026_11_06.md) reduces repeated
analyzer bookkeeping while preserving every PCM sample and meter/Goertzel state.
Two arithmetic modes pass 8.8 million frame comparisons. COM6 image b31d1013ab8b
adds no static RAM and passes full readbacks/37 cold codec cases. Two native
YouTube LHDC 192/fixed900 metronome runs repeat about 3.2% lower Core 0 elapsed
DSP cost, but Core 1 local elapsed cost does not improve and continuity still
fails (360064/359552 unfilled frames). This is a scoped optimization, with the
shared DSP work split and full qualification still unresolved.

[DSP consumer handoff](docs/DSP_RETIREMENT_HANDOFF_2026_11_06.md) records a
first gap with 1792 decoded frames waiting and zero processed output. A bounded
retirement change processes one local block before the next fair yield; the
actual two-core regression changes 512 ready frames to 640 at that boundary.
COM6 image 23b03035b61a passes readbacks and 37 cold codec cases, but music and
native YouTube metronome still have 382784/370432 unfilled frames at LHDC 192/fixed900.
There is no demonstrated playback speedup or continuity pass. PCM sizes and
priorities remain unchanged; the decoded-to-DSP consumer is still under investigation.

[Task accounting and UART output](docs/TASK_RUNTIME_UART_2026_11_06.md) now
separate logger overhead from PCM consumer timing. Whole-record interrupt-driven
UART lowers the measured central logger charge from 10.27% to 3.13% on Core 1
and post-drain charge from 59.58% to 1.37%. No audio storage or log suppression
is added. COM6 image 40805d7940ef passes guarded readbacks and 37 cold codec cases;
the 12.102-second LHDC 192/fixed900 music trace still has 352512 unfilled frames.
No transport or decode rejection is observed. This remains an incomplete
consumer-deadline investigation, with full qualification gates open.

[PCM buffer handoff and sizing](docs/PCM_BUFFER_HANDOFF_2026_11_06.md) records
the 256-frame decoder transfer versus 128-frame render/DMA units. Optional
Core1 early input release permits producer reuse during DSP; a fixed-memory
192k layout moves 8 KiB from processed output into decoded input. Host ownership
and allocation checks pass, but the live LHDC result still has 393856 unfilled
frames. COM6 image ae24c89e42eb remains development-only. A representative native
YouTube LDAC 96 metronome/key-volume trace has no recorded in-play shortage;
640 source-empty tail frames occur after pause. Remote-gain/acoustics remain open.
The [matched-feedback measurement](docs/DSP_MATCHED_FEEDBACK_2026_11_06.md) shows
no speed benefit; that arithmetic candidate and its recorder are disabled.

[Core1 fair-sharing measurements](docs/RENDER_FAIR_SHARE_2026_11_06.md) record
Earlier image7a1c4cbfc441: controller23, host/main-renderer22, decoder21/Core0
and helper20/Core0. One LHDC192/fixed900 comparison reduces unfilled frames
from506112 to318848; continuity still fails. LDAC 96 real-key volume regression
has zero recorded gaps. The explicit scheduling option defaults OFF; maximum
LHDC, metronome reproduction, acoustics and all-feature qualification remain open.

[Exact volume-ramp reuse](docs/VOLUME_RAMP_BLOCK_2026_11_06.md) keeps stable EQ
sections on the block path while only compensation coefficients ramp. Host
PCM/state remains exact; real-audio192k measurements observe29.4% fewer Core0
elapsed cycles in that scoped case. Earlier image24f9ec682af8 selects the faster
path without A/B instrumentation or the experimental host budget. One LDAC 96
rapid-key regression has zero recorded gaps; full LHDC continuity remains open.
Use -VolumeRampBlock with the existing full native/block/word build recipe.

The [audio scheduling/volume follow-up](docs/AUDIO_SCHEDULING_VOLUME_2026_11_06.md)
adds stream-bound delay attribution on both cores, immediate renderer wakeups
for accepted controls, and an opt-in host work budget. One real-key LDAC 96
volume trace has no in-play gaps; its source-empty gap occurs after pause.
LHDC192/fixed900 still fails, including with the corrected budget. These are
scoped results, not all-codec or maximum-rate qualification.

The [LHDC throughput work](docs/LHDC_V5_THROUGHPUT_2026_11_06.md) adds an exact
bounded S32 PCM-copy path and records high-rate scheduling/IRAM experiments.
[Bounded DSP assistance](docs/LHDC_DSP_ASSIST_2026_11_06.md) now uses the existing
Core0 worker for up to four canonical blocks, publishing each span directly.
Focused ownership/arithmetic tests pass; the measured fixed900 run still has
262528 unfilled frames and long Core1 dispatch/retirement intervals.
192k/24-bit/fixed900 playback still underruns; those experiments are not a
maximum-rate fix, and live1000kbps qualification remains open.

The [BTstack code audit](docs/BTSTACK_CODE_AUDIT_2026_11_05.md) now tracks
the dated 288-input implementation/build inventory, confirmed layer defects, unwired modules and
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
[LHDC quality visibility](docs/LHDC_QUALITY_VISIBILITY_2026_11_05.md) now separates
successful-return channel silence and frame concealment from ordinary decoder
rejection, with bounded metadata and separate UART records. Live quality events
and sustainable playback remain unqualified.
[LHDC PCM boundaries](docs/LHDC_PCM_BOUNDARIES_2026_11_05.md) now define S32
saturation and nonfinite-frame recovery and snapshot raw-session configuration
before old-workspace release. Live boundary/fault/audio behavior remains unqualified.
[Configured codec packet storage](docs/CODEC_PACKET_STORAGE_2026_11_05.md) now
removes Opus media/expiry/reset heap activity with a 6144-byte pool reserved only
while Opus is configured. It adds 6152 accounted bytes in that mode; LDAC staging
uses a bounded span copy. Live fragment/audio/memory-failure behavior remains unqualified.
[Transport discard deadlines](docs/TRANSPORT_DISCARD_DEADLINE_2026_11_05.md) now
wake for oversized PDUs even without a payload buffer. TX fragment assembly uses
at most three source-span copies with unchanged admission/commit timing.
Live timeout/prefix/credit/backpressure behavior remains unqualified.
[Shared output recovery](docs/SHARED_OUTPUT_RECOVERY_2026_11_06.md) follows a
failed LDAC 96k attempt with PCM backpressure, renderer over-budget timing and a
latched DMA order fault. Verified-pool EOF gaps now have bounded recovery and
separate accounting; passive role/stage counters retain the complete product.
Throughput, starvation and active recovery qualification remain unfinished.
[AAC LATM version1](docs/AAC_LATM_VERSION1_2026_11_06.md) now matches the observed
Redmagic transport mode and produces PCM without decoder rejection. Restored
bounded PCM reserve does not yet eliminate AAC startup gaps; aptX HD also has
a remaining small underrun. These short traces do not qualify every codec or
BLE/OTA coexistence.
[AAC parser bounds](docs/AAC_PARSER_BOUNDS_2026_11_06.md) now reject zero-progress
sections, out-of-range band/TNS/pulse fields and truncated bit consumption through
source-pinned SDK units. All four existing cold AAC cases preserve their reference
output; reported live crashes and shared low-bitrate behavior remain unqualified.
[ERTM channel storage](docs/ERTM_CHANNEL_STORAGE_2026_11_05.md) now reserves the
encoded frame buffer before OPEN and retains it through commit/cancel, with
provider callback fences. Active ERTM retains that capacity between frames;
Basic A2DP has no added buffer, and live ERTM/audio behavior remains unqualified.
[Callback-safe abort](docs/CALLBACK_ABORT_OWNERSHIP_2026_11_05.md) now defers
borrowed RX/channel/TX reclamation until callbacks return and prevents further
ACL submission after a callback failure. Host/runtime target layout is unchanged;
live callback/fault/audio behavior remains unqualified.
[LHDC window setup](docs/LHDC_WINDOW_SETUP_2026_11_05.md) now prepares the target
window and checks transform/entropy readiness during configuration, moving the
remaining required ramp allocation out of the first frame. Steady working
storage is unchanged; live setup/failure/audio headroom remains unqualified.
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
verified image is `4c24457a6aba94837aee242eae3c43b7513abbb6aeef4d2bd521b673d6f60413`.
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
descriptor-order fault during a finite LDAC 96 overload test; renderer overruns,
queue drops, underruns and core0 watchdog warnings remain unresolved. COM6
returned to released 1.0.4-idf6.1 ota0 and phone playback was paused.

The later [page-sized output ring](docs/OUTPUT_CAPACITY_2026_11_04.md) removes
28 KiB of required allocation at 192 kHz while retaining the 64 ms target and
prefill margin. LDAC 96 DMA ran without descriptor errors, but underruns and
watchdog warnings remain. Maximum LHDC still fails memory admission. Separate
[CRC-validated records](docs/CAPTURE_CRC_2026_11_04.json) now identify its exact
requested format and allocation failure; damaged records are excluded.

The [core1 logger follow-up](docs/RF_LOGGER_CORE1_2026_11_04.json) retains live
snapshots during overloaded LDAC 96 playback; the longest observed reporting
gap fell from 32.057 s to 2.492 s. Audio still underruns. The
[optional PCM-loan bridge](docs/PCM_RENDER_LOAN_BRIDGE_2026_11_04.md) now connects
the worker, renderer and sound paths behind `-PcmLoans`. Its first COM6 trial
removes 32 KiB of input allocation at LDAC 96 and retires 4882 loans without
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
never flashes. Only COM6 has been explicitly released for the current development
work. Deployment still requires identity/layout admission through the guarded
workflow; other boards remain excluded.

### ESP-IDF 6.1 maximum-rate pipeline checkpoint

The [COM6 pipeline record](docs/IDF61_PIPELINE_2026_11_04.md) documents the SRAM-region allocation fix,
179 portable tests and exact-bounds ASan/UBSan checks. The full-quality LHDC
constructor now fits, but the render path misses its throughput budget before
DMA starts. That historical checkpoint used the released fallback; current COM6
development selection is described above. Production parity and the master
objective remain incomplete.

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
