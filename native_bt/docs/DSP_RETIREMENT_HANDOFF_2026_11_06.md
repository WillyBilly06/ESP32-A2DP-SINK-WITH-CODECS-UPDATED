# DSP retirement and the decoded-PCM consumer — 2026-11-06

The master goal remains active and incomplete. This follows the fixed-memory PCM sizing experiment and the measured UART logger correction. The user asked whether decoder output size and consumer handoff explain I2S starvation.

## Evidence before the change

The whole-record console image 40805d7940ef23cb500946a6f2e3b8fbd2d1c5f7c44b93e8bbe7aad8aa138aeb records a coherent first-starvation event at MCU 433777818 us. Decoded input still holds 1792 frames, processed output holds zero, accepted/rendered/DMA source counts are 5760/3840/3584, and the renderer is inside a local render operation. The decoder is decoding with four encoded packets queued. There is no descriptor error or observed input sequence break. This particular event is downstream of decoding; it does not prove every later gap has the same cause.

At 192 kHz, one 5 ms LHDC frame is 960 stereo frames. The decoder uses S32 containers, so that is 7680 output bytes. It is transferred in at-most-256-frame chunks (2048 bytes), rendered in at-most-128-frame blocks (1024 bytes) and sent through three 128-frame DMA descriptors. The actual 2048-frame/16384-byte decoded ring can hold more than two LHDC frames. PCM scratch, encoded packet capacity, ring capacity and DMA descriptors are different resources.

All 2515008 accepted source frames eventually reach DMA in that capture, but 352512 descriptor frames remain unfilled. Zero packet/decode rejection does not establish on-time consumption. Core 0 decoder/helper work and Core 1 host/render/controller work remain substantial; idle time is not proof of a sustainable 192 kHz deadline.

## Bounded change

The old render step returns immediately when a completed helper job is retired. With equal-priority host/main-renderer fair sharing, that return yields before controls, refill service or the next local PCM block. Retirement itself does not process new PCM.

The change still returns immediately for READY/BUSY helper ownership. After DONE is fully retired, it continues through the ordinary controls, cancellation, stop and refill path, and may process one local block of at most 128 frames. It suppresses another helper submission in that one step so the decoder on Core 0 can resume. The existing fair yield remains after the bounded step. No priority, affinity, audio storage, DSP arithmetic, precision, clock policy or task is changed. There is no wait for input or another core.

The linked data, BSS, IRAM and rodata are unchanged; flash text increases 8 bytes. Image 23b03035b61ab25d8feab70ce789d393dee741f3caac4a4e86c565313332889e builds with ccache and passes exact source/image admission. Guarded hardware receipts and scoped playback results are recorded below.

## Regression

A two-core actual-output test holds the helper after publishing four blocks (512 frames) but before publishing DONE. While the borrow remains held, the producer queues another 256 frames and decoder admission remains fenced. The test captures the first Core 1 fair yield after retirement. Before the fix it sees 512 ready frames and fails the required 640-frame assertion. After the fix it sees exactly 640 frames. A test-setup failure from an already-signaled reused event is retained separately and corrected by resetting that event.

Existing held-borrow stop/cancel, lifetime, output-before-DONE, copied-input early reuse, fixed-memory allocation, five unaffected format/rate cases and quiescent restart checks also pass under ASan/UBSan. Main-task waits, blocking sends, pending timed polling and mock errors remain zero. Ordinary non-fair-share mode passes as well. Fair-sharing admission passes 4 header, 7 CMake and 16 image cases. No new boot test is added.

Hardware performance, live maximum bitrate, all-codec regressions, remote volume, BLE coexistence, stress and acoustic qualification remain distinct open gates.

## COM6 deployment and playback

Guarded deployment passes full app/selector readbacks at 921600/115200, owned VALID boot, original-app preservation and the existing 37 cold codec cases. Archive: runs/1794024446014926900-23b03035b61a. No automatic fallback occurs. The first Android reconnect reports a completed connection failure with HCI disconnect reason 0x13; the receipt and UI are retained. A subsequent explicit reconnect succeeds. This is not a pairing/coexistence qualification pass.

The native YouTube music trace commands 12.257 seconds with an independent 14-second pause guard. Actual UI restores LHDC 192 kHz/24-bit/stereo/fixed900, phone volume 80/150; sampled DSP remains volume 68, EQ 2/8/12, flags 12. Accepted/rendered/DMA source counts all equal 2537280 frames. Initial shortages are 585408, late recoveries 202624, unfilled 382784, descriptor errors zero. First starvation still has 1856 decoded input frames and zero processed output. Decoder/queue rejection, sequence breaks, ingress rejection, formatting losses, UART write failures and watchdog events remain zero. PCM backpressure is 7491; detailed worker-ring loss is 3364. Central 4.665695-second scheduler accounting shows Core 0 decoder/helper/idle 54.67/32.72/12.62% and Core 1 host/renderer/controller/logger/idle 25.54/43.21/12.68/3.13/10.91%. ISR/current-quantum caveats from the task-runtime report apply.

This does not demonstrate a playback improvement over the preceding whole-record UART image (352512 unfilled frames in a different 12.102-second music interval). The bounded handoff removes the demonstrated bookkeeping-only yield, but does not fix sustainable throughput. No speedup or continuity pass is claimed.

A second source uses the user-requested native YouTube app, 120 BPM - Metronome by Drumset Fundamentals, https://www.youtube.com/watch?v=xpiVrEET-YQ. Source selection auto-plays for 11.403 seconds under its own 12-second pause guard; a failed UI observation is retained, then the selected title and paused state are independently verified without replaying that selection. The focused metronome trace commands 12.216 seconds with a separate 14-second guard. The same actual codec, wire 566-byte samples, volume and EQ are confirmed.

Metronome stream source accepted/rendered/DMA counts equal 2544256; initial shortages 534144, late recoveries 163712, unfilled 370432, descriptor errors zero. First starvation holds 1600 decoded frames and zero processed output. Between its pre/post cumulative observer snapshots, PCM backpressure rises 8213 and detailed worker-ring loss 3629; packet/decode rejection, sequence breaks, host trace loss, formatting and UART write failures do not rise. All 282 snapshot records have verified CRCs; 87 raw trace/state records are unverified by design. Cumulative diagnostic totals after earlier playback are not misreported as this single trace's deltas.

The central 4.592154-second metronome interval charges Core 0 decoder/helper/idle at 48.01/37.15/14.83% and Core 1 host/renderer/controller/logger/idle at 25.96/40.16/11.89/3.17/14.49%. These averages still coexist with missed DMA deadlines. They do not prove an exclusive computational bottleneck or unused real-time capacity; the shared DSP lane and scheduling intervals remain the next investigation.

The phone is left paused on the metronome, volume 80/150, and the UART dashboard is listening. Full events, metadata, UI/failure observations, action times, section deltas and analyses are retained under native_bt/_build/experiments/task-runtime-1794019945314. Main/release qualification is not granted.
