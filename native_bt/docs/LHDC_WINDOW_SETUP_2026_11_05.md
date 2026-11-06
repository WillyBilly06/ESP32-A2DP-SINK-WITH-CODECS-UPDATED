# LHDC configuration-time window and readiness — 2026-11-05

The selected decoder already built rate-specific IMDCT tables and allocated FAC models during configuration. However, the window descriptor could still allocate/copy its DRAM ramp during the first decode. Its void preparation API could not propagate failure. The IMDCT initializer always returned success even when required reference-cache creation failed, and its caller ignored that return.

The source-pinned port now prepares the target window during decoder initialization, before initialized/worker READY is published and before streaming begins. Window preparation returns status; missing/invalid window extent or ramp allocation failure is an error. A valid window that does not match the structured fast shape still selects the same full generic-window math. That prepared generic choice is cached for the rate instead of rescanning every frame. A failed ramp allocation clears the prepared marker so no incomplete descriptor is accepted.

IMDCT initialization now ensures the implementation it will actually select has resources: verified fast tables or the selected reference cosine cache, including an explicitly forced diagnostic reference mode. Reference-cache failure returns error rather than success. Decoder initialization checks it, prepares the target window, allocates/checks entropy storage and only then sets initialized. Failed initialization releases rate-sized tails; the existing canonical/raw adapter close paths release remaining process caches and their containing workspace. Repeated cleanup is safe because released pointers and alloc_mdct_size are cleared.

The normal target frame still sees cached resources for its fixed configured rate; its window preparation check now propagates error. No table/coefficient, transform/overlap arithmetic, codec CIE, sample rate, depth, bitrate limit, PCM format, FIFO size or CPU-affinity decision changes. Host-only window-file overrides retain first-frame lookup/preparation timing. Reference-mode selection remains available; successful resource readiness is not a real-time guarantee for the slower diagnostic/reference implementation. Diagnostic mode changes must remain serialized with the owner.

Current source-window dimensions were inspected without executing the decoder:

| MDCT size | Leading zeros | Ramp floats | Ramp payload | Codec-accounted ramp |
| --- | --- | --- | --- | --- |
| 480 (44.1/48k) | 76 | 88 | 352B | 360B |
| 960 (96k) | 152 | 176 | 704B | 712B |
| 1920 (192k) | 304 | 352 | 1408B | 1416B |

These are existing allocations moved to setup, not new steady working storage. The selected lhdc_tables.c source hash is bdeae531ddc5bda36807bd6f2ae2d4d8226840806790c1d77bc9360c7840dca3. This shape census is source-data inspection, not a decoder/bit-exact/acoustic test or full audit of the unmodified sibling algorithms. The generator still checks its existing selected source/header pins; it does not claim that all external dependencies are pinned by those guards.

The modification is in the owned transformer; the user-owned sibling source is unchanged. Allocation/validation happens on the existing codec worker during asynchronous configuration, not in the HCI callback. No task, queue, fallback Bluetooth host, polling wait or persistent metadata is added. Moving allocation changes its order and timing; tight-heap/codec-switch/failure behavior remains unqualified. Existing generic and diagnostic math paths are preserved without lowering requested format or silently claiming their throughput meets the target.

Target generated decoder/IMDCT compilation and full ccache/source/workspace checks pass. Portable host Clang analysis reports no decoder diagnostics and one unchanged dead-store warning: the final unused local p increment in the original IMDCT table-carving block. That warning is retained, not represented as a clean analyzer run. The target-only cold window call is target-compiled and source-reviewed; the host diagnostic macro excludes that specific call. Object relocations confirm decoder initialization calls window preparation, transform initialization and entropy readiness.

Linked text grows 72 bytes to 882811; aggregate data 397569, BSS 50412, IRAM 122255 and DRAM data 23893 are unchanged. Existing retained tables/buffers and configured task stacks are unchanged. No throughput or full-chain stack-headroom improvement is established.

No separate setup-failure, generic-shape, reference-mode, low-memory, codec-switch, functional, packet, playback, pairing, app, RF, stress or acoustic campaign was invoked. Existing intrinsic transform checks and 37 cold codec checks remain part of normal startup; no new tests were added. Ordinary startup does not establish live first-frame latency, interrupted lifecycle, output equivalence for all streams or maximum 192k/24-bit/1000kbps headroom. Full feature/security/source/lifecycle/crash/underrun qualification remains unfinished.

The current per-frame bitrate-band selection was also read: all low/mid/high 44.1/48k descriptors retain MDCT 480, 96k retains 960 and 192k retains 1920. The prepared size therefore matches the supported 5 ms configured rate even when packet bitrate/band count changes. This is source evidence, not an active bitrate-switch test. BTstack contributes no decoder math here; the existing user-owned C99 implementation is retained, with no BTstack implementation integrated.

## COM6 readback and ordinary idle boot

Image 4c24457a6aba94837aee242eae3c43b7513abbb6aeef4d2bd521b673d6f60413 was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 match; original application full-region MD5 is unchanged. Owned host remains selected VALID with automatic Bluedroid restoration disabled. Ordinary idle startup is ready with no watchdog/backtrace; decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected.

Idle heap is 137912 free/67356 reported minimum/110592 largest bytes. The minimum is an aggregate of regional low watermarks, not proof of saved live memory. Text is 882811 (72 larger), BSS 50412, IRAM 122255 and DRAM data 23893 unchanged. All 37 existing cold codec checks pass (RF false); four LHDC cases have zero tracked-peak allocation delta, and overall cold peak remains 56899. UART source 127 is connected. No separate live setup/failure/reference/generic/low-memory/codec-switch/playback/RF/app/acoustic/headroom campaign ran. Archive: native_bt/_build/hardware/runs/1793947670582980800-4c24457a6aba/.
