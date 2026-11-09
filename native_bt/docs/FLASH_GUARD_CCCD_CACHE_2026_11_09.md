# Flash-cache stall and durable CCCD cache, 2026-11-09

COM6 runs readback-verified independent-host image `297aae153578cb627fa366b187aca530d83558296613eb71f57c2067b10a3c69`. The build uses ESP-IDF 6.1's BTDM controller through VHCI, with Bluedroid and NimBLE disabled. This checkpoint adds a bounded flash-guard observer and a 48-byte verified durable CCCD cache. **LHDC continuity is still unresolved; the goal remains active and incomplete.**

## Confirmed blocker

The observer-only image `0f7a15f5823a` caught a finalized I2S miss with 832 decoded raw frames waiting, processed output empty, and the SDK flash cache-disabled slice active. Its frozen token 6978/serial 872 came from Core 0; the local scheduler was suspended. The confirmed slice began 5967 us before that miss. The decoder's phase was wait_pcm_space, not decoding. This directly establishes a flash/scheduler obstruction at this event rather than insufficient decoder arithmetic throughput. It does not explain every earlier gap.

The cache candidate repeats the obstruction. At 161408493 us, 832 decoded raw frames remain and processed output is empty. The decoder has been waiting for PCM space for 8126 us. Flash token 3626/serial 453 is active from Core 0, with a confirmed slice beginning 7117 us earlier and scheduler state SUSPENDED. Renderer intent is inside audio rendering; that marker is not an execution sample. A CRC-verified first_unfilled record was already read by the PC before its pause request (event 3509145, observed 1794220679.071955 s; pause requested 1794220681.1903 s). **This is a confirmed pre-pause audio miss, not a stop-tail classification.**

The precise CCCD job at either first miss was lost from the finite host event ring. No individual save token, total guard duration or erase type is inferred from the frozen flash token. A separate observer-image excerpt had an empty-input miss while the flash guard was inactive and the scheduler running; that distinct path remains under investigation.

## Smallest safe storage change

The existing serialized CCCD/EQ persistence worker now retains one verified durable snapshot: identity type, all six address bytes, full 32-byte bond-incarnation tag and four descriptor values. Static size is 48 bytes. Every load still reads and validates NVS. Only a matching successfully loaded or committed record can populate the cache; empty/default values do not count as verified durable state.

For that exact identity/incarnation, a changed save omits the redundant provider-level blob read but still performs nvs_set_blob and commit before reporting success. An unchanged verified save makes no storage calls. Cache misses keep the original checked-read path. Failed writes/commits invalidate the cache and preserve the existing poisoned-boot failure behavior. New-incarnation zero values still overwrite the old tag. Schema, keys, access checks, acknowledgments and FIFO ownership are unchanged. No task, stack, PCM buffer, quality setting or scheduling priority is added or changed.

The driver may perform internal reads during nvs_set_blob. This optimization does not eliminate durable writes, garbage collection, cache-off intervals or their worst-case latency. The hardware result below does not establish an audio improvement.

## Observer boundary and cost

GNU linker wrappers call each original SDK flash-disable/enable function exactly once and leave the SDK guard intact. A lock-free three-word observer publishes one token and begin time. It arms once after audio initialization; diagnostic token exhaustion becomes unavailable without blocking flash. No heap, UART, task, retry or spin is added to the wrappers.

Active means the conservative confirmed cache-disabled slice from the disable function's return until entry to the enable wrapper. It excludes both-core coordination before that return and restoration after that entry. Sampling makes one attempt; incoherence is reported rather than retried. First starvation and first finalized-unfilled IRQ latches independently retain this metadata and the public scheduler-state sample. Separate CRC-protected UART records expose both.

The linked-image audit checks observer code/data/literals, required timer and scheduler dependencies, and actual SDK callback dispatch to both wrappers. It is not a whole-SDK callgraph or runtime proof. Relative to the prior 0e1684820f1c image, the observer adds 332 B IRAM and 40 B DRAM data, with BSS unchanged. The cache adds 48 B BSS and 244 B flash text versus the observer image, with no additional IRAM. Only 145 B of the linked IRAM region remains. Normal 2x192 DMA and the 31744 B PCM/DMA payload remain.

## Validation

Actual output/observer code passes ASan/UBSan at 100 and 1000 Hz simulated RTOS ticks, including early/unarmed state, active/finished slices, both cores, incoherence, exhaustion, frozen IRQ captures and existing refill/cancel/order behavior. Actual app/profile/real-codec worker tests pass for the observer revision. A later cache ownership comment changes probe_cccd_store.h; that broad app receipt is not presented as fully source-matched to the final cache candidate.

The final actual CCCD provider passes source-bound ASan/UBSan with mocked NVS and opaque synthetic tags: 64 changed saves add zero provider-level blob reads while retaining 64 sets/commits; verified no-op saves perform no storage calls. Tests cover full identity/tag differences, new-tag zero revocation, malformed records, open/read/set failures and both uncertain-commit outcomes. Cryptography is deliberately outside that mock test. The existing lifecycle/reconnect/feature tests also pass using the real SDK 6.1 PSA implementation and NVS shim, with assertions enabled. These do not qualify physical pairing or power-loss behavior.

The ccache target build rechecks 10440 inputs and passes exact image/source/partition and owned-ISR admission. Flash upload uses 921600 baud and full readback 115200. The existing cold target checks pass 37 codec configurations / 173 callbacks: stack margin 3452 B, tracked peak 56899 B, minimum free after allocation 105584 B, maximum cold decode 13164 us. Cold decode timing is not live headroom. Boot watchdog/backtrace/idle-starvation counts are zero. All 133 snapshot records pass CRC; 52 other boot records are outside that audit.

## Bounded native YouTube observations

Fresh Redmagic settings show LHDC V5, 192 kHz, 24-bit, stereo, phone-fixed900 kbps. **This is not the required 1 Mbps qualification.** Each native YouTube excerpt has an independent 14-second phone pause guard, with actual MCU stream-start acknowledgement through renderer-stop upper bound below 15 seconds. BLE connects before audio; these sweeps do not test connection during playback.

| Image | I2S upper bound | Initial / recovered / unfilled frames | Controls |
|---|---:|---:|---|
| Observer 0f7a15f5823a |12.819577 s|3072 /384 /2688|32 descriptor ACKs +32 independent readbacks, unchanged EQ ACK/settings readback, original CCCD write ACK/readback before pause|
| Cache 297aae153578 |12.741581 s|2880 /448 /2432|Same checks completed before pause; final helper passed and closed GATT|

These are separate short windows, not a matched A/B result. The smaller count does not establish a cache-caused performance gain. Both retain a frozen cache-off miss with decoded PCM waiting. The cache window has a verified pre-pause UART witness; the observer window lacks that direct witness, so its requested-pause phase is not classified from stale peer state.

Host diagnostic event losses within the sweeps are92 and94 respectively (boot-cumulative 105 and 94). Worker/snapshot/UART loss counters are zero. Missing individual CCCD timings remain missing. No media drops, decoder faults, sequence breaks or descriptor errors are reported. Stopped heap snapshots are 61584/52568 B free, with boot-to-date minima 19420/19500 B; the runs have different boot/workload histories and these are not an isolated RAM comparison or a leak finding. Full parity and minimum-memory claims remain open.

## Evidence and handback

[Structured results and source digests](FLASH_GUARD_CCCD_CACHE_RESULTS_2026_11_09.json) bind the image, modified source, tests, footprint, UI and trace receipts. Exact source/image/ELF, raw UART, Android helper sources/APK and per-image harnesses remain locally under native_bt/_build/hardware/runs/1794216959160762400-0f7a15f5823a and runs/1794220359436191900-297aae153578. Large hardware artifacts are not published in Git. Completed rebuildable artifacts are compressed with integrity/roundtrip receipts; the 512 MiB cap is unchanged.

BDK Audio is restored to the foreground. YouTube is PAUSED, I2S stopped, API-confirmed coexistence bits cleared, and identity-verified COM6 UART remains 921600 baud. Owned ota1 remains VALID with no automatic Bluedroid return; the previous ota0 application is preserved. COM7 is untouched.

The next audio task is to bound the full continuous flash obstruction and provide an event-driven way to preserve processed PCM across legitimate durable storage, within the existing RAM limits. Waiting inside the SDK cache guard, weakening durable ACKs, skipping security validation or silently reducing codec fidelity are not solutions. Sustained 192k/S24/1Mbps plus BLE, other codec/security/lifecycle/OTA qualification, acoustics and equivalent-feature RAM/CPU comparisons remain open.
