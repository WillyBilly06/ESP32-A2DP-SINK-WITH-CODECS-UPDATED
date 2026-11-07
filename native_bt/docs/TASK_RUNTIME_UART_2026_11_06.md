# Task run-time accounting and UART output — 2026-11-06

The full Bluetooth/audio goal is active and incomplete. High-rate LHDC still fails continuity. All codec, product, security and RFCOMM options, affinities, priorities, EQ, PCM precision and fixed-memory buffer layout are retained.

## Latest result

COM6 runs owned image `40805d7940ef23cb500946a6f2e3b8fbd2d1c5f7c44b93e8bbe7aad8aa138aeb`, archived under `runs/1794023432424998000-40805d7940ef`. The ccache build and exact source/image admission pass. Full application and selector readbacks match at write/read rates of 921600/115200. Owned VALID boot, the existing 37 cold codec configurations and original-application preservation pass. No automatic Bluedroid restoration occurs. These are development checks, not production qualification.

The Redmagic native YouTube music capture commands playback for 12.102 seconds with an independent 14-second pause guard. LHDC V5 is restored to 192 kHz, 24-bit stereo and fixed 900 kbps in the actual phone UI. The wire samples carry 566 media bytes per 5 ms frame, consistent with 899200 compressed bits/s after two transport header bytes. Live 1000 kbps remains unqualified. Phone volume is 80/150; sampled DSP volume is 68, EQ 2/8/12, flags 12.

| Observation | Polling VFS | IRQ VFS, per character | IRQ, whole record |
|---|---:|---:|---:|
| Central logger scheduler-charged share, Core 1 | 10.27% | 9.56% | 3.13% |
| Post-drain logger scheduler-charged share, Core 1 | 59.58% | 9.46% | 1.37% |
| Central idle scheduler-charged share, Core 1 | 3.69% | 5.96% | 10.42% |
| Unfilled DMA frames in complete capture | 460416 | 369920 | 352512 |

The windows use the same selected music and restored controls, but different source positions, startup/tail intervals and lengths. They show lower logger CPU charge, not an isolated audio-throughput speedup. Whole-record central window: MCU 440716241 to 445350708 us (4.634467 seconds). Decoder/helper/idle on Core 0 are 55.03/33.21/11.76%; host/renderer/controller/logger/idle on Core 1 are 25.53/44.42/12.20/3.13/10.42%. About 4.3% of Core 1 is outside these tracked roles/current-quantum accounting. No task-number/core changes or clock wrap occur. ISR time is charged to interrupted tasks; these are not exclusive CPU measurements.

The last whole-record capture accepts, renders and supplies DMA with exactly 2515008 source frames. Initial shortage is 499712 frames; 147200 are recovered before the descriptor deadline, leaving 352512 unfilled. Descriptor errors, packet-queue drops, decoder rejects, PCM failures/timeouts, input sequence breaks, ingress rejects, watchdog events, formatted-record losses and UART-write failures are all zero. PCM backpressure occurs 6425 times. The detailed worker event ring loses 2880 events; cumulative counters and separately CRC-verified snapshots remain available. The capture contains 282 verified snapshots and 83 raw/unverified trace/state records; it is not a claim that every raw record has a CRC. Logger stack low-water is 1488 bytes.

The actual active geometry is 2048 input frames/16384 bytes, 2560 processed output frames, target 2048, and three 128-frame DMA descriptors/3072 bytes. Total ring/DMA payload remains 39936 bytes. A 5 ms LHDC frame at 192 kHz is 960 stereo PCM frames/7680 bytes; transfer limit is 256 frames and render limit 128 frames. No extra audio storage is introduced. Equal end-of-capture counts establish eventual consumption, not on-time consumption. The remaining gaps are not cured by size changes or console changes.

Full UART events, before/after states, actions, metadata and runtime analyses are retained under `native_bt/_build/experiments/task-runtime-1794019945314/{music-runtime,irq-music,record-music}`. The hardware run archives preserve their exact images and receipts.

## Whole-record console contract

`-ConsoleIrq` initializes UART0 before workers/controller startup, reuses an existing driver and chooses the public interrupt-driven VFS backend. It preserves console pins, baud and newline policy. A newly created driver has a 256-byte RX ring, no TX ring, no event queue and no new task. Initialization failure is explicit.

The background snapshot logger now sends each existing at-most-1024-byte formatted record in one public UART driver call. It preserves stdout newline conversion, record CRC bytes and stdout/stderr record serialization in caller-owned storage. Invalid records and short writes increment a separate `snapshot_uart_lost` counter; formatting losses remain distinct. No log rate or record is suppressed. Relative to the per-character IRQ image, linked DRAM data increases 16 bytes, BSS and IRAM are unchanged, flash text increases 288 bytes and rodata 16 bytes. Driver heap and SDK accounting metadata are separate from static section sizes.

This is event-driven background output, not a claim that all runtime logging is nonblocking. Whole-record output may sleep on driver/stdio serialization; it is never called from controller, decoder, renderer or ISR callbacks. Rare host state/error printf calls and locks remain a review item. [ESP-IDF UART VFS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/vfs.html), [standard I/O](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/stdio.html).

Actual initializer/writer tests pass ASan/UBSan in LF, CRLF and CR modes, including initialization success/reuse/failure, optional status pointer, exact one-call bytes, length/capacity validation, internal-newline rejection, short writes and lock release/order. Actual console admission passes 4 CMake, 14 image, 5 SDK and 1 CLI rejection cases. Runtime observer admission passes 5 CMake, 14 image, 13 SDK, 4 private SDK ON/OFF transitions and 1 CLI rejection.

## Observer contract and historical baselines

-TaskRuntimeStats enables ESP-IDF's task accounting with the 1 MHz ESP timer and uint32 counters. It selects the trace/core-ID metadata needed by the public API; ordinary builds explicitly restore those SDK options OFF. Five actual CMake cases, fourteen actual image-admission cases, thirteen SDK-header cases, four private SDK ON/OFF transitions and an early CLI rejection pass. The real probe SDK file is unchanged by the transition tests.

The background logger calls vTaskGetInfo with stack scanning disabled and a supplied state that avoids task-state lookup. Eleven fixed-lifetime registered roles are read, including the two renderers, host, decoder, controller, logger, idle, IPC and ESP timer tasks. Handles live until app reset. No TaskStatus name pointer is dereferenced, allocation performed, new task created or observer call made from a decoder/render/controller/ISR path. Each call uses the SDK kernel lock; collection and maximum individual query wall times are reported. This is a sequential read interval, not a simultaneous atomic snapshot. [ESP-IDF FreeRTOS API](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/api-reference/system/freertos_idf.html).

Sanitized tests cover all eleven roles, absent handles, zero task number, counter/time wrap, no stack/state scans, inactive/failed registration and ordinary tick-sampler regression. Counter deltas require matching task number/core and a nonwrapping observation window. Run time is scheduler-charged elapsed time: ISR time remains attributed to interrupted tasks and the current unfinished quantum is not yet in its TCB counter. Other untracked tasks are not silently called idle.

## Polling-console baseline

Image84b10c785655a096aea32b074ba89a654a7fe4e3e63cf4085bf1d2c034528ec9 built with ccache, passed exact source/image checks, full app/selector readbacks, owned VALID boot and the existing37 cold codec cases. Original application remains preserved; no automatic host fallback occurs. Native YouTube is paused on '4 IN 1 - HIYA x LBI (PROD. GAZ) | JERK DRILL TYPE BEAT' by Gaz Official before a12.209-second commanded play/pause interval with independent14-second pause guard and post-pause collection. LHDC192k/S24/stereo/fixed900 and phone volume80/150 are restored.

One central3.735027-second window (MCU101046098 to104781125us), stable task numbers and no clock wrap:

| Core | Task | Scheduler-charged share |
|---|---|---:|
|0|Decoder|55.67%|
|0|DSP helper|32.57%|
|0|Idle|11.76%|
|1|Owned host|23.33%|
|1|Renderer|44.79%|
|1|Controller|13.56%|
|1|Logger|10.27%|
|1|Idle|3.69%|

About4.4% on Core1 is outside these tracked roles/current-quantum accounting. Timer/IPC task deltas are zero in this window; this does not exclude interrupt-level effects. Query spans at the window endpoints are88/50us. Across the capture, one collection spans1997us while its maximum individual query remains14us, showing preemption between reads rather than proof of a2-ms kernel-lock hold. Logger stack low-water remains1188 bytes.

After source drain, the logger alone is charged59.58% of Core1 over2.828 seconds. Source inspection confirms the default UART VFS character writer loops until FIFO space exists. That work is low priority, so the measurement alone does not prove it caused decoder starvation. Its CPU/peripheral traffic is nevertheless avoidable. [ESP-IDF UART VFS modes](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/vfs.html).

Music counters still fail: accepted/rendered/DMA source2439616 frames; initial shortage625792, late recovery165376, unfilled460416, descriptor errors0. Different content/timing and observer overhead prevent an isolated performance comparison with earlier images. This is not a full-rate or sustained-headroom pass.

Observer linked deltas relative to ae24c89e42eb: DRAM data+32, BSS+352, IRAM+72, flash text+1896 and rodata+704 bytes, plus the SDK's dynamic TCB/queue trace metadata. The temporary observer has a footprint; it is not a claimed memory optimization.

## Interrupt-driven console experiment

-ConsoleIrq initializes UART0 before controller/worker startup, reuses an already installed driver, then selects the SDK interrupt-driven VFS backend. It preserves pins, baud and newline modes; a newly created driver has256-byte RX, no TX ring and no event queue. No task is added and no logging frequency or record is suppressed. [ESP-IDF standard I/O](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/stdio.html).

This permits background output to sleep instead of polling. It does not make every printf call asynchronous: stdio/driver locking and rare host state/error logging remain a review item. Initialization failure is explicit. Sanitized actual-initializer tests cover success, reuse, allocation failure and optional status output; actual admission passes4 CMake,14 image,5 SDK and1 CLI rejection cases. A first target compile exposed a missing explicit UART component/include dependency; corrected component admission follows the existing codec/I2S pattern and the failed receipt is retained.

Candidate4a05c5a813a016bfd09f0ff0cfe364f33a9980ec11c73a01f3f4e96a4ddfcd6a passes exact source/image validation. Relative to the observer-only image: unchanged DRAM data, BSS+8, IRAM+80, flash text+8032 and rodata+1912 bytes. Driver heap/synchronization cost is separate from those static sections. The completed playback comparison is recorded above. All-codec, maximum1Mbps LHDC, remote-gain, coexistence, stress and acoustic gates remain open.
