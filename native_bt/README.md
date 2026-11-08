# Independent ESP32 Bluetooth host

The [stream-boundary checkpoint](docs/A2DP_STREAM_BOUNDARY_2026_11_08.md) adds
separate accepted-job, decoder-handoff and AVDTP-state records within the existing
trace storage, with unchanged linked SRAM/IRAM. It also reads queue geometry
atomically in the background logger. Six app variants and37 cold target cases pass.

COM6 image9048cff45172 is readback-verified at **921600 baud**. A metronome
excerpt has0 unfilled frames, but music has960 with an active gap165.6ms after
Streaming and13.07s before accepted Suspend. **LHDC continuity remains unresolved.**
An early host/RTP cadence deficit near48ms motivates a bounded startup-reserve
trial; full1Mbps and sustained BLE/control qualification remain open.

The [packed LHDC input checkpoint](docs/LHDC_PACKED_MEDIA_2026_11_08.md) stores
small encoded records within the existing eight-slab pool. Raw566-byte packet
capacity rises from8 to24; full1800-byte admission and borrowed-input lifetime
remain. Physical HCI credit reservations are preserved. It adds16B BSS with
unchanged IRAM/pool/tasks.191 host checks,80 fixed+80 packed real-codec cases,
six app variants and37 cold target cases pass.

Checkpoint image3b246522d523 is readback-verified at **921600 baud**. Target queue
high-water reaches11 versus7 in fixed controls. Three short YouTube windows
retain3264/2304/1152 final unfilled frames, first captured after pause acknowledgment.
No pre-pause finalized gap was recorded in those finite candidate windows;
**sustained LHDC continuity,1Mbps and BLE/control/full parity remain unqualified**.

The [idle logging checkpoint](docs/UART_IDLE_CADENCE_2026_11_08.md) keeps firmware
and the raw-UART web viewer at **921600 baud**. Idle detail runs once per five
counter cycles while events, first-gap traces and metadata retain their existing
paths. Measured idle traffic after LHDC playback fell from about14KB/s to3KB/s,
with zero trace/UART losses. Linked SRAM/IRAM are unchanged. Checkpoint image2492e98a66c3
passes readback and37 cold codec cases. The routed metronome excerpt retains256
final unfilled frames, first captured after pause acknowledgment; background
logging was sparse under load. **LHDC continuity remains unresolved.**

The [startup-guard comparison](docs/LHDC_STARTUP_COMPARISON_2026_11_08.md) rejected
30ms: same-image50/30/30/50 windows contained512/6208/7872/1600 unfilled frames.
Normal50ms and trace-off policy are restored. The supplied local Bluedroid code
actually applies18 LHDC queue entries; the owned queue has8 uniform1800-byte
payload slots. That storage-efficiency difference is under review, not proven
as the cause. Full1Mbps and sustained BLE/control/feature parity remain open.

The [frozen-dispatch and early-refill checkpoint](docs/FROZEN_DISPATCH_EARLY_REFILL_2026_11_08.md)
puts local DMA refill immediately after fully mixed PCM publication, ahead of
bookkeeping. An actual render-step deadline test fails without the correction.
An optional bounded task trace now identifies controller/host/render/decoder
contexts; it adds352B DRAM and488B IRAM when enabled and is off by default.
The checkpoint diagnostic imageaf37df00e68c runs the trace at **921600 baud**.

**LHDC continuity remains unresolved.** Corrected metronome windows have
**2112/11712 unfilled PCM frames**; music has0. The active repeat waits for encoded
input with empty PCM queues, separately from an earlier decoded-PCM handoff miss.
Core191/191, output/sound/trace checks and37 cold codec cases pass. Full1Mbps,
BLE/control, acoustics and complete feature/security/OTA qualification remain open.

The [retired-packet scheduling checkpoint](docs/RETIRED_PACKET_SCHEDULING_2026_11_08.md)
adds conditional owner yield points between fully retired HCI packets, returning
safe receive credits before yielding. Linked SRAM/IRAM and all priorities/buffers
are unchanged.191/191 host tests and37 cold target cases pass. Short corrected
LHDC192/S24/fixed900 windows still have **256/704 music and3456 metronome unfilled
PCM frames**. Playback-time misses remain; this is not a continuity fix.
Checkpoint imagee7948405b95b and the raw-UART monitor run at **921600 baud**.

The [encoded-duration checkpoint](docs/LHDC_ENCODED_DURATION_2026_11_08.md)
counts grouped LHDC frames and partial PCM delivery correctly for negotiated
AVDTP delay reports, costing **16 B BSS** without increasing buffers. Redmagic
does not negotiate that report; normal mode skips unused parsing. Later fixed900
traces still have **576 / 8832 unfilled PCM frames**, with confirmed playback-time
misses and decoded PCM waiting for DSP. **LHDC continuity remains unresolved.**

Firmware and the raw-UART web logger remain at **921600 baud**. Formatting/writes
run in the low-priority background logger; expected PCM backpressure is counted
without per-event output. Repeated unchanged unknown-duration rows are suppressed.
All195 records in a fresh6s passive window pass CRC verification. Checkpoint
image9bced750ed38 passes readback and37 cold codec cases. This is development
firmware;1Mbps, sustained BLE/control headroom and full parity remain unqualified.

The earlier [IRQ refill dispatch correction](docs/IRQ_REFILL_DISPATCH_2026_11_08.md)
requests scheduling for useful equal-priority refill work while preserving the
controller/host/render priorities. Three fixed-900 and two Auto LHDC192/S24
excerpts end with **zero finalized unfilled frames**; one initial 64-frame
shortage is recovered. Auto actually carries **404.8 kbps of payload**, so
**1 Mbps and sustained BLE/control headroom remain unqualified**. The change
adds 228 B IRAM with unchanged linked DRAM data/BSS. COM6/UART use **921600 baud**.

The [pause-boundary and deadline investigation](docs/LHDC_PAUSE_DEADLINES_2026_11_08.md)
confirms a playback-time miss with decoded PCM waiting for DSP. An urgent idle-
helper trial showed no consistent benefit and was rejected; prior program bytes
are restored on COM6. First-gap timing is bounded separately from phone pause,
and all shortage counts remain. UART stays **921600 baud**. LHDC continuity,
1 Mbps and simultaneous BLE/control headroom remain unqualified.

The [LHDC register-scan checkpoint](docs/LHDC_REGISTER_SCAN_2026_11_08.md)
uses documented Xtensa FP-to-integer register moves for S32 classification.
Exhaustive scalar/frame checks and 80 real-codec deferred cases pass. Two short
matching music intervals measure about **9.7–9.8% lower scale/output time** and
**0.9–1.2% lower whole-frame elapsed time**; IRAM is 32 bytes smaller.
LHDC continuity remains open: music has **1152 / 0 / 1088** and metronome **5568
unfilled PCM frames**. UART remains **921600 baud**; this is development firmware.

The [DMA refill priority checkpoint](docs/DMA_REFILL_PRIORITY_2026_11_08.md)
fixes a reproduced bookkeeping-before-refill deadline defect, keeps urgent
DSP at 64 frames, and permits at most two extra ready refill steps before a
host yield. The execution policy adds no linked SRAM/IRAM. Separate owner and
credit gap snapshots cost 120 bytes SRAM and 392 bytes IRAM. Current short
LHDC192/S24/900 windows still have **768 / 1152 / 192 unfilled PCM frames**.
Continuity remains unresolved; UART stays **921600 baud**.

The [lightweight arrival checkpoint](docs/LIGHTWEIGHT_MEDIA_ARRIVAL_2026_11_08.md)
correlates real HCI callbacks with admitted RTP packets without the scheduler
profiler. It costs 112 bytes of DRAM and 96 bytes of IRAM. Short LHDC192/S24/900
windows have **0 / 768 final unfilled PCM frames**. The finalized metronome gap
has no pending ACL metadata, a decoder waiting 11.7 ms for encoded input, and
no VHCI callback for 28.4 ms. Credit/owner/controller attribution remains open.
UART remains **921600 baud**; this is unqualified development firmware.

The [runtime logging checkpoint](docs/UART_NECESSARY_LOGS_2026_11_08.md) keeps
firmware and the raw-UART web logger at **921600 baud**. Expected PCM backpressure
is counted without individual normal-mode records; explicit detailed tracing
retains them. Two live windows have zero worker trace losses and **6528 / 0
final unfilled PCM frames**. The logging change is verified; LHDC continuity
remains unresolved. Linked SRAM and IRAM are unchanged.

The [partial HCI completion checkpoint](docs/HCI_PARTIAL_COMPLETIONS_2026_11_08.md)
returns safe individual controller credits as encoded slots retire, preserving
the three-packet ingress limit and all queue sizes. Host checks pass 188/188.
Short LHDC192/S24/fixed900 YouTube windows have **576 / 0 / 0 unfilled PCM frames**
versus a fresh prior-image music control of 1152. Continuity and full feature
qualification remain open. COM6 and the raw UART monitor run at **921600 baud**.

The [reserved Core0 DSP checkpoint](docs/CORE0_RESERVED_DSP_2026_11_08.md)
permits the existing helper during decode only with at least8ms of processed
audio reserved. Observed helper workshare rises from53% to74% with unchanged
linked RAM/IRAM, cores, priorities and audio format. Current short LHDC windows
still have **2592 / 864 / 1120 unfilled PCM frames**; continuity remains open.

The [DMA/lightweight-gap checkpoint](docs/AUDIO_DMA_LIGHT_GAPS_2026_11_07.md)
keeps separate media, decoder, renderer and finalized-gap records without the
heavy tick/owner profiler. COM6 runs **921600 baud** with **2 x 192 DMA frames**,
the same3,072-byte DMA payload as before. Its earlier short LHDC192/S24/900 windows
have **768 / 2,880 unfilled PCM sample frames**; continuity remains unresolved.
The37 existing cold codec cases pass. This remains development firmware.

The [UART logging follow-up](docs/UART_LOG_CADENCE_2026_11_07.md) verifies COM6
and the live monitor at **921600 baud**. In detailed tracing mode, two repetitive cumulative diagnostics
report once per second; separate errors, first-gap records and full raw UART
are retained. The flashed build passes all 37 existing cold codec cases. This
logging check does not qualify LHDC playback; its remaining underruns are open.

The [earlier arrival/owner checkpoint](docs/MEDIA_ARRIVAL_2026_11_07.md) binds
VHCI callback times to admitted RTP packets and skips fourteen redundant deadline
queries when the runtime is already ready. Current-source sanitizer checks and
all 37 cold target cases pass. Short LHDC192/S24/900 traces have **5,696 / 3,200 /
0 unfilled PCM frames**; one clean repeat does not qualify continuous playback.
UART remains **921600 baud**. The custom host remains development firmware.

The [earlier queue/transport review](docs/QUEUE_ERASE_REVIEW_2026_11_07.md) rejects
a word-erase candidate after worse phone windows and restores the prior firmware
implementation. New ASan/UBSan queue/SPSC regressions pass; reverse controls still
have **1,920 / 4,736 unfilled PCM sample frames**. Input timestamp jumps and late
rendering are recorded separately. LHDC continuity remains unresolved.

The [host RX checkpoint](docs/HOST_RX_HANDOFF_2026_11_07.md) removes
duplicate deadline collection, reduces sensitive-scratch erase stores and adds
a fenced RX-to-render handoff. Host regressions and all 37 cold codec cases pass.
Two short LHDC 192 kHz/S24/900 kbps traces still have **4,416 / 9,024 unfilled
PCM sample frames**. Continuity, 1 Mbps and full feature/coexistence qualification
remain open. The UART monitor is live at **921600 baud**; a fresh 842-record
window is entirely CRC verified. This remains development firmware.

The [UART configuration checkpoint](docs/UART_921600_2026_11_07.md) runs the firmware and
live monitor at **921600 baud**, keeps full raw records, and suppresses unchanged
configuration output. All 37 cold codec cases pass; a current-source window has
4,998 valid CRC records with no invalid/malformed records.

The [PCM-prefix/build checkpoint](docs/URGENT_PCM_PREFIX_2026_11_07.md) adds guarded
partial refills and prevents restored source timestamps from leaving stale objects.
Builds recheck compiler inputs through ccache. That checkpoint’s short LHDC 192 kHz/S24/
900 kbps trace has 6,080 unfilled sample frames; continuity, 1 Mbps and full
feature/coexistence qualification remain open. This is a development host.

The [LX6 Rice-reader checkpoint](docs/LHDC_RICE_CURSOR_2026_11_07.md) preserves
valid PCM and adds checked malformed-magnitude rejection. Actual ESP32 linking
uses 496 B less instruction RAM with no persistent data/BSS increase. All 37
existing target cases and 96 clean full-PCM cases pass. A short 192 kHz/S24/900 kbps
music check still has 4,928 unfilled frames; continuity qualification remains open.

The [integrated audio checkpoint](docs/INTEGRATED_AUDIO_2026_11_07.md) corrects
LHDC24 output level against the real phone: measured48k/192k levels match LDAC
within0.014dB. The192k decoded-input ring is8KiB smaller. Fresh-stream EQ policy
is applied before DSP starts; live controls retain integer ramps.

The earlier integrated image has2432/6080 unfilled frames in two short192k/S24/fixed900
traces. Startup buffering, credit reserve and bounded owner checkpoints are
integrated, but uninterrupted playback and the complete feature/1Mbps/coexistence
qualification remain open. This is development firmware.

[Per-query host timings](docs/OWNER_QUERY_COST_2026_11_07.md) separate the15
deadline getters and host poll categories. Their elapsed cycles include
interrupts/preemption; they are not pure CPU costs. The earlier
[host-pressure handoff](docs/OWNER_PRESSURE_HANDOFF_2026_11_07.md) and AVRCP bitmap
experiments are not retained as proven performance fixes.

The [stereo S32 scan/emission fusion](docs/LHDC_SCAN_EMIT_2026_11_07.md)
preserves PCM and quality behavior and measures about1.8–1.9% lower whole LHDC
decoder time in two short192k/S24/fixed900 runs. It costs92B instruction RAM,
with no new buffers or data/BSS. LHDC render underruns remain; this is not a
continuity or full-feature qualification.

An [immediate DSP resubmission experiment](docs/LHDC_RETIRE_RESUBMIT_2026_11_07.md)
showed no repeatable continuity advantage and is removed. The exact previous
image is restored; the reverse comparison also documents substantial run
variation. LHDC continuity and full-feature qualification remain open.

The [idle-decoder DSP admission](docs/LHDC_IDLE_DSP_2026_11_07.md) lets the
existing lower-priority Core0 helper work while LHDC waits for encoded input.
Short matched-setting runs show fewer unfilled frames with no added SRAM;
LHDC continuity still fails. LDAC96 codec-switch checks show zero shortages,
but variable wire framing leaves fixed990kbps qualification open.

The [S32 output specialization](docs/LHDC_S32_OUTPUT_2026_11_07.md) preserves PCM
and reduces measured LHDC192k/S24/fixed900 decode time about9.8% in two short
YouTube runs. Remaining refill shortages and full-feature qualification are open.

The earlier [LHDC entropy candidate](docs/LHDC_MODEL_TRAFFIC_2026_11_07.md)
preserved complete model/history and PCM in host checks. Its measured decoder
time remained within baseline variation; the unproven candidate was restored
to the original entropy path before the current output optimization.

The [exact limiter division](docs/LIMITER_WORD_DIV_2026_11_07.md) replaces the
clipped-sample wide division with a bounded word implementation. The complete
integer curve and DSP state pass exhaustive/regression checks; candidate timing
and remaining audio continuity still need device evidence.

The [LHDC PCM-level investigation](docs/LHDC_PCM_LEVEL_2026_11_07.md) confirms
and corrects a rate/depth gain mismatch before decoder saturation. Independent
level/bitrate-transition checks and80 sanitized host codec cases pass; device
max-volume auto-bitrate continuity remains unqualified.

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

[Deferred terminal failure diagnostics](docs/HOST_FAILURE_ASYNC_2026_11_07.md)
copy all24 original fields atomically through three existing trace slots. Both
failed/parser_failure rows remain; initialized/full queues never fall back to UART.
Actual-source tests cover partial publication, slot reuse during blocked output,
all space/wrap boundaries and exact JSON values. Full ccache/admission passes
for unflashed image2ade0c873b73; SRAM sections unchanged (+716 flash text,+496
rodata). Consumer local copy grows64 bytes; live stack checks remain pending.
COM6 physical recovery and the complete master qualification remain open.

[Deferred visibility/GAP logs](docs/HOST_STATE_ASYNC_2026_11_07.md) now publish
immutable snapshots through existing trace storage after logger initialization.
The host does not wait for UART, including on a full queue; startup output is
preserved. Actual source/blocked-consumer tests and full ccache build/admission
pass. Unflashed imagec2ed33307c2c has unchanged SRAM sections (+532 flash text,
+368 rodata). COM6 recovery and physical regression remain pending. Other direct
logging paths and all maximum-rate/coexistence/feature gates are still open.

[Experimental full-range finalizer](docs/DSP_FULL_OUTPUT_2026_11_07.md) preserves
exact PCM/state and measures716–717 versus778–779 helper elapsed cycles/sample.
LHDC192/fixed900 still has32832/17280/22528 unfilled frames. IRAM+288 bytes;
8-bit free heap is324 bytes lower in this boot despite unchanged DRAM sections.
Last verified COM6 imageb309817ce3d6 passed readbacks/owned boot/37 cold cases.
The subsequent LDAC regression aborted before play because UART became silent;
reader reopen and one ROM attempt did not recover it. Physical COM6 replug is
pending. Current runtime and LDAC regression are unverified; this is not a release.

[Idle DSP ramp-call guard](docs/DSP_IDLE_RAMP_2026_11_07.md) preserves exact PCM
and complete DSP state while skipping three no-op calls per idle sample frame.
Previous image950428ba68dd passes full readbacks/owned boot/37 existing cold cases.
Repeated LHDC192/fixed900 helper cost is778–779 versus833–834 elapsed cycles/frame;
unfilled47744/30592/34048 still fails continuity. IRAM+8 bytes, data RAM unchanged.
LDAC96 with real80 ->0 ->80 volume keys has1443584 source frames/zero shortages.
The full maximum-rate/coexistence/acoustic/feature gates remain open.

[Six-block DSP development selection](docs/ASSIST_SIX_BLOCKS_2026_11_07.md)
retains an explicit 768-frame cap with unchanged memory, arithmetic and priorities.
Fresh LHDC192/fixed900 unfilled totals are116608 with five blocks and
107648/74368/72320 with six; continuity still fails. Previous imagef2637213bb69
passes full readbacks/owned boot/37 existing cold cases. LDAC96 has1451264
source frames and zero shortages. Default4 remains; the retained build explicitly
uses `-DspAssistBlocks 6`. Full maximum-rate/coexistence/feature gates stay open.

[Pre-decoder RTP clock telemetry](docs/RTP_INPUT_CLOCK_2026_11_07.md) now records
an exact consecutive-sequence timestamp pair before queue admission/decode, with
a coherent one-attempt snapshot and sparse host-writer events. Previous image7fbb6ccc5146
passes full readbacks/37 cold cases; diagnostic cost is24 static bytes. LHDC
sequence717->718 has27840 RTP ticks instead of960, an extra140ms already at input.
No local packet/decode rejection occurs, but LHDC still has122880 unfilled frames.
LDAC resets to its normal512-tick maximum and has1443584 source frames/zero shortages.
Native phone queue dumps were unavailable; no cause or controller-policy fix is claimed.

[Five-block DSP development selection](docs/ASSIST_FIVE_BLOCKS_2026_11_07.md)
uses an explicit640-frame cap without adding RAM or changing precision/priority.
This previous selection used `-DspAssistBlocks 5`; the later six-block record above supersedes it.
Actual sanitizer/lifecycle and82 admission checks pass. Previous image041f25beb2ef
passes guarded readbacks/37 cold cases. LHDC192/fixed900 unfilled199296 ->
124672/108800 in repeated short segments, with zero descriptor/contract errors.
LDAC96/observed990k delivers1445632 frames with zero shortages. Sender RTP clock
spans also exceed received frame content; the handset mechanism remains unproven.
Full LHDC continuity and all-feature qualification remain open.

[DSP completion wake order](docs/ASSIST_COMPLETION_WAKE_2026_11_06.md) now signals
the Core1 owner before notifying the higher-priority Core0 decoder. The pinned
kernel confirms the preemption path; a negative/two-core sanitizer regression
proves the owner can retire/close while the helper continuation is suspended.
Previous normal image a9427d8cafde passes readbacks/37 cold cases, adds16 flash bytes
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
