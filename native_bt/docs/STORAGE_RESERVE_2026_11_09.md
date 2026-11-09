# Storage-aware PCM reserve experiment — 2026-11-09

The live reserve prototype failed coexistence acceptance and is **disabled by default**. Its zero-shortage excerpts deferred persistent BLE control work until playback stopped. They do not establish an audio improvement. The default remains the independent HCI/VHCI host on ESP-IDF 6.1; Bluedroid/NimBLE/BTstack host code is not linked and automatic Bluedroid return remains disabled.

## Default and experimental boundaries

The root preset explicitly selects `StorageReserve=false`. The lower-level builder always writes `DBT_PROBE_STORAGE_RESERVE=OFF` unless its experimental `-StorageReserve` switch is supplied. The CMake option defaults OFF and requires the complete native asynchronous copied-PCM integration. The experimental GATT admission, output lifecycle changes, cross-core publication and diagnostic row compile only with that option. Original storage operations, durable acknowledgments and application interfaces remain available in the default image.

Default image `d2092d7504e9dd0c75a5a422038929fb3e5921b4a6936e1dd76cec66e5366d98` has no reserve macro in any compiler command and no Bluedroid/NimBLE host runtime source in the compilation database. Compared with checkpoint `b566da3f908f`, linked IRAM/data/BSS are unchanged; flash text increases by four bytes. IRAM headroom remains only 65 bytes. This is not a RAM or playback-performance improvement claim.

COM6 is readback-verified at 921600 upload / 115200 verification baud. Its existing 37 cold codec configurations/173 callbacks pass; the minimum self-test stack margin is 3,452 bytes, with no boot watchdog/backtrace faults. All 134 boot snapshot records pass CRC validation. The original application region is unchanged and the owned development image remains selected VALID. UART/web logging resumes at 921600 baud.

A final default-image 12.755-second guarded excerpt completes all 32 descriptor ACK/readbacks, EQ/settings checks and original-CCCD restoration before pause. It retains **2,560 unfilled frames** (3,456 initial shortages, 896 recovered), with minimum free internal heap **19,480 bytes**, stopped free61,504/largest34,816 bytes, decoder/render stack margins5,912/3,772 bytes, and no decoder/media/sequence/descriptor/refill-contract/watchdog faults.98 host diagnostic events are lost; worker/snapshot/UART losses are zero. The183 checksummed captured records pass CRC validation (13 other records remain unchecked).

The first finalized miss is observed over UART before the requested phone pause. Its frozen IRQ and completed-peak records match token4714/serial589: Core0 cache-off begins203,305,397us, ends203,321,984us (**16.587ms**), and contains the gap at203,313,060us. The scheduler is suspended,832 decoded raw frames are waiting, and the renderer is inside audio rendering. Decoder state is wait-for-PCM-space with10 encoded records at the last state change. This confirms a remaining storage/cache-off scheduling failure with decoded PCM available. It is not an underrun-free or sustained qualification result.

## Implemented experimental mechanism

The portable audio API can append and return full 512-frame processed-PCM pages without copying or changing queued samples. The renderer must quiesce the actual DMA IRQ consumer on its own CPU and hold no DSP borrower. Full/wrapped or otherwise unsafe geometry defers without changing ownership. Pages returned by trim are freed by the background worker. Close frees attached pages exactly once.

A separate bounded protocol reuses the existing CCCD/EQ persistence worker, renderer and notifications; it creates no task or queue. At 192 kHz it attempts three temporary 4,096-byte pages, extending processed capacity from 2,560 to 4,096 frames and requesting 3,840 processed frames (20 ms). Total PCM/DMA payload can temporarily rise from 31,744 to 44,032 bytes. Lower rates reuse sufficient existing capacity. Allocation is attempted once per page on the background worker. Failure performs no storage operation and uses existing resource-error completion. A permission expires two milliseconds after the occupancy observation. The worker executes the backend once, then waits for safe detach and page release before completion. Cookies and waiter handles remain alive through retirement.

Actual-source tests cover startup gating, a storage request arriving after the start precheck, accepted primed-stop clips and cancellation, ordinary post-publication grant checking, and DMA consumption between assisted publication and Core 1 DONE retirement. The latter records a timestamp before the occupancy load through two lock-free words; permission keeps the original age bound. Wrong stream epochs, stale observations and old request tickets cannot grant permission. These fixes pass deterministic sanitizer tests, but did **not** resolve the measured live prefill wait.

## Hardware results and rejection

Each excerpt uses the authorized COM6/Redmagic fixture, native YouTube metronome, LHDC V5/192 kHz/24-bit/stereo with the phone set to fixed 900 kbps, preconnected BLE, and an independent 14-second pause guard. Fixed 900 is not 1 Mbps qualification. BLE connection during playback was not tested here.

| Image | I2S elapsed upper bound | Final unfilled frames | All 32 controls before pause | Maximum firmware prepare wait | Minimum free internal heap |
| --- | ---: | ---: | --- | ---: | ---: |
| Previous `b566da3f908f` | 12.648 s | 2,496 | Yes | No reserve gate | 19,480 B |
| Prototype `163553779a73` | 12.782 s | 1,536 | No | 12.285 s | 7,376 B |
| Local-publication correction `cb4364e38e38` | 12.636 s | 0 | No | 11.289 s | 7,376 B |
| Assisted-publication correction `eb696b4251aa` | 12.773 s | 0 | No | 10.430 s | 7,328 B |

These are finite, unmatched windows, not a controlled A/B improvement. In the last prototype, the active snapshot is PREFILL with three requests/two claims and 12,288 temporary bytes. After stop, all 36 requests claim permission with maximum grant age zero (no active-audio permission), return to IDLE and release all temporary pages. Every descriptor write eventually receives ACK plus independent readback; unchanged EQ/settings and original CCCD restoration also complete, **after pause**.

The last prototype has zero media/decode/sequence/descriptor/refill-contract faults, but loses 31 host diagnostic events. Worker/snapshot/UART losses are zero. The completed 17.341 ms flash peak is after playback, so it does not verify reserve coverage during audio. The first prototype retains 1,536 unfilled frames; its first gap is not proven to be a flash-overlap or stop-tail event. All failures remain in the record.

The earlier checkpoint independently confirmed a 17.383 ms cache-off slice at a playback-time miss. Full normal processed PCM plus DMA covers only 15.33 ms at 192 kHz. The new protocol passes a real-PCM host model consuming 3,338 frames over that measured interval; the hardware prototype never demonstrates the corresponding active grant. Present evidence does not establish whether its live target is unattainable under current production/dispatch pacing or observed too late. That occupancy/dispatch distinction is the next unresolved measurement, rather than justification for further blind buffer growth.

## Validation and retained evidence

All C compilation uses ccache and separate compile/link steps. ASan/UBSan checks cover real PCM resize, wrap/ownership/OOM/close, reserve cancellation/expiry/epoch/ticket behavior, actual renderer/helper interleavings at 100/1,000 Hz, and the actual persistence worker with modeled permission. The default ATT/GATT schema/protocol suite and real-codec app/worker integration also pass. Mocked RTOS/I2S/NVS tests are not hardware timing, RF, acoustic, flash power-loss or full feature-parity proof.

The three experimental images pass the existing 37 cold codec configurations/173 callbacks. The final experimental boot has 134 CRC-verified snapshot records and no boot watchdog/backtrace faults. Cold codec checks do not qualify all live codecs or switching. Build admission verifies stable compiler inputs, image checksums, controller-only dual mode and partition capacity; the ISR audit passes. The 512 MiB rebuildable-output gate is retained. Completed host/compiler artifacts and closed logs are compressed with hashes and verified round trips; failed receipts and exact flashed images/source snapshots are retained locally.

[The machine-readable receipt](STORAGE_RESERVE_2026_11_09.json) records image/source/test hashes, retained local artifact identities and deployment scope. Raw captures, ELF files and diagnostic APKs are local evidence under the ignored build directory, not GitHub release assets.

The prototype covers only the serialized CCCD/EQ provider. General name/OTA work, SMP storage and other flash providers are not gated by it. Existing active audio gaps, BLE establishment variability, low-memory overlap, sustained LHDC 1 Mbps, all codec/security/lifecycle/product/OTA parity, acoustics and matched Bluedroid RAM/CPU qualification remain open. This checkpoint is development work, not a stable custom-host release.
