# LHDC recovered-error and concealment visibility — 2026-11-05

The supplied decoder can return LHDC_DEC_OK after replacing a failed right channel with silence, and after concealing a complete frame whose decoded peak exceeds its corruption threshold. The worker's ordinary decode-reject counter therefore did not describe all decoder-generated silence. Normal trace-disabled builds also suppressed the legacy diagnostic printf. These code paths exist; their comments and historical rates do not prove their frequency or RF cause on the current image.

The pinned source transformer now observes primary-channel failure, right-channel silence and the existing frame-quality decision without changing their return values, retry policy, overlap handling or PCM samples. The worker adapter associates each completed C99 decode attempt with its codec constructor generation, decoder frame index, RTP sequence/timestamp/SSRC and total encoded-frame size (including its two-byte frame header). Allocation failures and wrapper format/length mismatches have separate flags.

The product enables counts after the existing cold codec checks and before decoder admission. Counts cover this boot's decoder-owner attempts, wrap as uint32_t, and survive codec close/reopen. Constructor generation also covers earlier cold opens; reset may repeat a decoder frame index. RTP metadata and the cumulative event-attempt ordinal provide additional correlation. Packet preflight rejection before a C99 decode attempt remains in existing worker reject/queue telemetry.

Three separate raw-UART JSON records are emitted by the existing lower-priority logger after at least one live attempt:

| Record | Meaning |
| --- | --- |
| lhdc_decode_result | Attempts/rejections and the latest notable attempt's constructor/frame/RTP identifiers, raw decoder return and flags. |
| lhdc_channel_recovery | Cumulative primary-channel errors and right-channel silence, plus the latest notable attempt's channel results. |
| lhdc_frame_quality | Cumulative peak-concealed/clipped frames and decoder clipping checks, plus the latest notable peak's binary32 bits. |

All three share one coherent snapshot revision. Metadata is the latest rejected, clipped, concealed or channel-error attempt, not the latest normal frame and not necessarily the latest event of each individual kind. A later clean frame does not erase an anomaly. These are cumulative counters plus the latest notable tuple, not a lossless event replay or proof of radio loss. A false last_event_valid means no notable tuple has been recorded. Channel return values apply only when the corresponding flag is set; peak bits apply only when the quality scan was reached. Peak units are the output container amplitude, not bitrate or final DSP level.

| Flag | Meaning |
| --- | --- |
| 1 | Primary channel decode failed. |
| 2 | Right channel decode failed and its PCM was silenced. |
| 4 | Existing frame-peak concealment selected. |
| 8 | Existing decoder clipping checks found over-limit samples. |
| 16 | An allocation hook failed during the decode call. |
| 32 | C99 returned success but output length/configuration validation failed. |
| 64 | Frame-quality scan completed. |
| 128 | The wrapper rejected this decode attempt. |

No quality callback formats text, writes UART, allocates memory, waits or retries. Its notices update worker-owned transient fields. A single decoder writer publishes lock-free atomic words; a compile-time assertion requires lock-free integer words. The reader makes at most three attempts and leaves its output unchanged on failure. Clean frames update the attempt/version counters without rewriting the last event or zero-valued error counts. No queue or task is added. All sample/rate/depth/bitrate limits and other codecs stay unchanged.

The selected BTstack comparison is its codec-health accounting, including distinct good/bad/zero-frame counters in src/classic/btstack_sbc_bluedroid.c:423–431/526–535 and btstack_sbc_decoder_bluedroid.c:335–343/441–450. Those mSBC/SBC paths are a reference for separating recovery from successful delivery, not an LHDC implementation or proof of profile parity. No BTstack implementation is integrated.

Strict adapter Clang analysis and target compilation pass. Source-pinned generation and the full ccache/source/workspace build guards pass. Target quality storage is 84 bytes plus a 4-byte constructor serial; active LHDC state grows 32→56 bytes. Snapshot ABI is 80 bytes; resume own compiler frame remains 112 and snapshot own frame is 80, excluding callees/full-chain headroom. Target object imports no atomic library helper. Default linked text grows 1816 bytes to 882687; aggregate BSS grows 88 to 50404. IRAM 122255 and DRAM data 23893 are unchanged; aggregate data grows 864 bytes from flash-backed metadata/strings. Monitoring cost and live error behavior remain unmeasured.

No separate functional, packet, playback, pairing, app, RF, stress, acoustic or fault-injection campaign was run. Existing startup checks and ordinary idle boot do not exercise live recovered-error records or establish max LHDC headroom. Full source/feature/security/lifecycle, reported codec crashes, underruns and sustained quality remain unfinished.

The unrelated legacy raw-session adapter remains unlinked in the product. Its source review identified an input-borrow edge to finish: open closes an old workspace before it has finished reading the config pointer, without copying or explicitly excluding aliasing with that workspace. This is not established as a cause of any reported product crash; that adapter's review remains partial.

The idle heap snapshot is 137920 free/67092 minimum/110592 largest bytes. Compared with the prior image, free falls 88 bytes, the cold-check allocation snapshot falls 112, and the aggregate reported minimum falls 2544. ESP-IDF esp_heap_caps.h:223–241 and heap_caps.c:306–315 define minimum as the sum of regional low watermarks that may have occurred at different times. The larger minimum change is not attributable from the captured stage snapshots and is not proof of 2544 extra simultaneous live bytes. No lower-RAM or timing improvement is claimed.

## COM6 readback and ordinary idle boot

Image 48b1a0c785af3f6da99fe34e19158401d01be1b0d76837bea721ef3ff0554051 was flashed at 921600 baud to ota1 at 0x390000. Image and selector readbacks at 230400 match; original application full-region MD5 is unchanged. Owned host remains selected VALID with automatic Bluedroid restoration disabled. Ordinary idle boot is ready with no watchdog/backtrace; decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20 remain selected.

Idle heap is 137920 free/67092 minimum/110592 largest bytes. Static storage increases 88 bytes and active LHDC state 24; the cold-check allocation snapshot falls 112. SDK minimum sums regional lows at potentially different times; its additional aggregate transient drop remains unattributed. Text 882687, BSS 50404, IRAM 122255 and DRAM data 23893. UART source 117 is connected. The existing 37 cold codec cases pass (RF false). No live LHDC quality records were observed at idle, and no separate functional/packet/playback/app/pairing/RF/stress/acoustic/fault campaign ran. No max LHDC headroom or codec crash/underrun fix is established. Archive: native_bt/_build/hardware/runs/1793942165766316900-48b1a0c785af/.
