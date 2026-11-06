# Shared output failure and bounded DMA recovery

## Observed failure

One focused playback trace was explicitly approved. The Redmagic paired with COM6 and started its existing native-player session using LDAC96k on commitcd4185de/image975e2c84582b. Playback failed during setup before the timed60-second observation began. That interval therefore observed frozen failure counters, not60 seconds of continuous playback. The already-running UART history preserves the preceding attempt; no second playback was started.

The retained counters show415 media packets, zero input RTP sequence breaks, zero decoder rejects, zero oversize rejects, maximum payload661 bytes,178 encoded-queue full drops and2025 PCM backpressure returns. The1800×8 encoded queue was not rejecting overlarge LDAC packets. These figures locate a post-decoder/output pressure problem in this attempt; they do not prove every codec has the same cause or establish RF retransmission statistics.

Rendering produced229120 frames in1790 blocks of128. Aggregate elapsed render time4216429us gives2355.55us/block, against1333.33us of playback at96k. Elapsed time includes preemption, so this is not pure FIR/DSP CPU time. An I2S order fault (expected0, observed2, size1024, gap13252us) latched output failed. The watchdog reported IDLE1 starvation; its sampled CPU1 task was main at probe_main.c1110. A single backtrace is not proof of a host busy-spin, nor proof the renderer alone caused the IRQ gap.

Private evidence: native_bt/_build/experiments/shared-path-trace-1793953786783/{setup-uart-events.ndjson,uart-events.ndjson,observation.json,analysis-summary.json}. UART trace queues lost detail under load; aggregate counters and verified records are retained. The watchdog also interrupted a UART JSON line, so malformed/unverified rows must not be used as counters. Player state was paused after the attempt.

## Recovery policy

The original ESP32 SDK ISR (esp_driver_i2s/i2s_common.c:919–947) clears the interrupt status and reports the latest completed descriptor. It does not replay each intervening EOF callback. Treating any known-address order gap as permanent output corruption was too strict.

The owned callback now requires the full three-buffer pool and a complete subsequent ordered traversal before recovery is eligible. A known, correctly sized descriptor may resynchronize only with that verified ring and no existing fatal flag. Shape errors, unknown/excess buffers, unverified ordering and existing fatal state retain their stop policy.

Recovery touches no skipped hardware buffer and returns no PCM loan: these buffers already contain DMA-owned copies. It credits each pending skipped copy once and clears only its metadata credit, preventing double accounting. Missed callbacks are a lower bound; additional complete laps/replays are not inferable from the latest address. Stop drainage resets its silent streak and still needs a fresh complete consecutive silent traversal. Work is bounded by the three-descriptor pool, with at most two skipped entries. No retry, allocation, lock, delay or logging is added to the ISR.

Original error counters remain. Separate i2s_order_recovery records expose recoveries, missed_callbacks_min and source_frames_recovered, with explicit complete-lap ambiguity. Recovering a notification gap does not erase an underrun, reconstruct missing time, prove continuity or repair renderer overload.

## Passive profiling

Stay-native now permits the existing passive tick sampler and conversion/FIR/DSP cycle counters in the complete asynchronous product. PowerShell, root/component CMake and offline image validation agree on this mode. Synthetic pipeline, codec-only and kernel-comparison entry points remain excluded. Actual compile commands contain one sampler and no benchmark units. There is no automatic playback, new task or test stimulus.

Scheduler samples are interrupted-task observations at1000Hz, biased by masking/aliasing and higher-level interrupts; they are not exact CPU utilization. All five permanent roles are recognized (mask31). Stage cycles include preemption and wrap modulo uint32; use consecutive observations and output-lifetime boundaries. If logging is starved for too long, a total cannot safely infer the number of complete counter wraps. The profiler is instrumentation, not a speed improvement.

FIR length/bank, arithmetic, integer EQ, sample rate/depth, codec configuration, payload1800×8, clock policy, affinities and priorities are unchanged. Controller23/host22 stay on CPU1, decoder21 on CPU0, renderer20 on CPU1.

## Verification

Exact-product target compilation passes before/after; sent own frame remains64 bytes (finish_sent32), excluding SDK interrupt frames and full-chain headroom. Full ccache build passes with10349 stable source hashes; rebuildable storage466.376MiB. Text890271, aggregate data399113, BSS50420, IRAM122943, DRAM data24173. Relative to cd4185de, DRAM data+BSS rises288 bytes and IRAM688; no payload pool or task is added.

COM6 image 0d068bd01a5308bca8c77e217eb626ba427ec0df94dbb6754d6cdbe08386793f was flashed at921600; image and selector readbacks at230400 match. Original application full-region MD5 is unchanged, owned ota1 is VALID/kept_active and automatic Bluedroid restoration remains disabled. Ordinary idle startup is ready with no watchdog/backtrace. All37 existing cold codec checks pass RFfalse (tracked peak56899). Idle heap is137624 free/67068 regional minimum/110592 largest,288 bytes lower than before profiling. The minimum sums regional low watermarks, not a simultaneous live peak. UART source133 is connected and both new records are visible. Archive: native_bt/_build/hardware/runs/1793956137265849400-0d068bd01a53/.

The recovery branch has not yet been exercised on the new image. No follow-up playback, synthetic stress/fault campaign, all-codec regression, acoustic test or sustained maximum-LHDC qualification has run. Shared throughput and CPU1 starvation remain unresolved; the new counters are intended to guide that work. The complete host/feature/security/lifecycle goal remains open.
