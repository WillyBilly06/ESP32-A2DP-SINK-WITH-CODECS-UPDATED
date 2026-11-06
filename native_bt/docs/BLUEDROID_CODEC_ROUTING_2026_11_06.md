# Bluedroid codec-boundary comparison — 2026-11-06

This is an incomplete, explicitly scoped audit of the actual local tree at `D:/bt-owned61/esp-idf/components/bt`. It is a reference comparison for the independently authored host. It does not claim that all Bluetooth layers or features have been reviewed or qualified.

## What the current candidate actually builds

The ESP-IDF configuration selects `CONFIG_BT_CONTROLLER_ONLY=y` and `CONFIG_BTDM_CTRL_MODE_BTDM=y`. The linked host is the owned HCI/VHCI, L2CAP, AVDTP, A2DP, AVRCP, ATT/GATT and SMP implementation. The map has no `esp_bluedroid_init`, `esp_bluedroid_enable`, `btc_init` or `bta_sys_init` entry point. The Bluedroid CMake host list returns early when `CONFIG_BT_BLUEDROID_ENABLED` is unset.

The compile database contains 153 translation units under Bluedroid's **external codec library** directory, and zero non-external Bluedroid host units. Some other codec sources are textually included or generated through the standalone codec components. Reusing those decoder libraries does not select the Bluedroid host. The source-stable candidate image is `86e7bbe9720ab1f6861487638f7c9195043f0cfb23de57463530bd45cba004bd`.

## Reviewed source versus pending source

[Local inventory](BLUEDROID_LOCAL_AUDIT_2026_11_06.json) lists 1,193 non-external C/C++ source/header files under the local BT component, excluding NimBLE. Five have a complete codec-boundary review recorded in this audit; 1,188 remain pending **in this audit**. Build inclusion is recorded independently from review status. This is not an assertion that every external decoder implementation, protocol layer, or earlier review has been re-audited.

The released Bluedroid build's actual compile database at `D:/bt-idf61/bt_audio_sink/build-idf61/compile_commands.json` selects `a2dp_vendor_ldacbt_decoder.c` and `a2dp_aac_decoder_custom.c`. The FDK AAC wrapper and older LDAC wrapper are inactive alternatives. Comparing only their filenames or buffers would give the wrong baseline.

| Original local path within Bluedroid | Owned counterpart | Finding / change |
| --- | --- | --- |
| `btc/profile/std/a2dp/btc_a2dp_sink.c` | `codecs/common/dbt_codec.c` | Original queue retains variable-sized BT_HDR packets. Owned queue was a universal eight-slot allocation. Selected-codec queue policy now replaces that universal depth. |
| `btc/profile/std/a2dp/btc_a2dp_sink_ext_coedc.c` | Owned codec dispatch | External encoded-callback route handles SBC/AAC; released sdkconfig leaves that route disabled. It is not the general aptX/LDAC decoder path. |
| `stack/a2dp/a2dp_vendor_aptx_decoder.c` | `src/dbt_avdtp_sink.c`, `codecs/aptx/dbt_aptx.c` | Original Classic/LL header callback strips nothing; HD strips RTP. Owned sink incorrectly forced all codecs through RTP. Endpoint framing now preserves Classic/LL raw codewords and retains RTP for HD. |
| `stack/a2dp/a2dp_vendor_ldacbt_decoder.c` | `codecs/ldac/dbt_ldac*.c` | Original skips RTP plus one LDAC media-header octet, then emits one decoded frame at a time. Owned generic RTP view leaves the codec header for the LDAC parser, which removes it once. Both use the LDACBT library family. |
| `stack/a2dp/a2dp_aac_decoder_custom.c` | `codecs/aac/dbt_aac_decoder.c` | Both own their PCM output; original generic scratch requirement is zero for AAC. Owned Helix workspace is allocated only for the selected codec. No additional common decoded PCM scratch is allocated for AAC. |

The normal sink, aptX, active LDACBT and custom AAC files have identical bytes in the local and released reference trees. The external-codec file differs; it is not used by the released baseline configuration. File SHA-256 values are recorded in the inventory and private reference receipt.

## Buffer meanings and memory

The user's `BT_A2DP_SINK_BUF_*` constants are **decoded PCM scratch**, not encoded queue slot sizes. Original queue limits are AAC 18, LDAC 20, SBC/default 18; its 10 KiB AAC / 6 KiB other pressure targets are minimum free internal heap thresholds, not encoded queue budgets.

The owned policy keeps a full 1,800-byte payload capacity in every slot. It never shrinks packet capacity based on an assumed encoded bitrate:

| Selected codec | Slots | Accounted queue allocation | Credit-release threshold |
| --- | ---: | ---: | ---: |
| SBC | 4 | 7,272 B | queued < 1 |
| aptX / HD / LL | 4 | 7,272 B | queued < 1 |
| Opus variants | 4 | 7,272 B | queued < 1 |
| AAC | 8 | 14,536 B | queued < 4 |
| LDAC | 8 | 14,536 B | queued < 4 |
| LHDC V5 | 8 | 14,536 B | queued < 4 |

Each slot has 16 bytes of metadata and is aligned to eight bytes. Totals include the eight-byte allocator accounting header, not allocator-internal overhead. Four-slot codecs save **7,264 bytes** relative to the former universal allocation. Eight-slot codecs keep their previous burst/startup allowance; these depths still need playback qualification, especially Opus.

The receive-credit upper bound is three ACL packets. Small queues release credits only when empty; the consumer now wakes the owner at the matching selected low-water crossing. A queue of four therefore remains large enough for the three already credited packets. BLE/control/teardown completion eligibility is retained. A failed reconfiguration reconstructs the previous codec and its queue policy after releasing the failed allocation; it does not hold two decoder working sets.

Target ELF DWARF sizes are AACDecInfo 96 B, PSInfoBase 20,560 B, aac_state_t 576 B, aptx_state_t 24 B and aptx_context 4,404 B. Stereo AAC's selected decoder allocation is **26,876 B**, including its 4,096-byte PCM output and 1,540-byte padded raw-unit copy. With the eight-slot queue its accounted worker total is **41,412 B before any active fragment buffer**. This is not total system RAM: renderer rings, task stacks, transport, BLE and allocator overhead are separate. aptX's selected decoder allocation is 5,464 B; with four slots its worker total is 12,736 B. No precision reduction was made.

## Framing and verification limits

Raw transport is selected by endpoint metadata; compressed data is never sniffed for an apparent RTP version. Raw sequence/timestamp values are explicitly marked host ordinals/sample counts, and do not increment upstream RTP-loss counters. Worker sequence checks can still detect locally dropped queue entries. HD and all other RTP endpoints keep the bounded CSRC/extension/padding parser.

[AOSP's aptX implementation](https://android.googlesource.com/platform/packages/modules/Bluetooth/+/7bf239adf5/system/stack/a2dp/a2dp_vendor_aptx.cc) independently returns false from `A2DP_VendorUsesRtpHeaderAptx`. No content-protection service is negotiated by these endpoints.

The focused `tests/codecs/transport_policy.c` check uses the production parsers and worker with fake codec/I/O. It passes raw bytes that resemble RTP, whole 1,800-byte ownership, four/eight-slot allocation, low-water notification, teardown and allocation-failure rollback. It proves those local invariants only. The candidate builds with ccache and stable inputs. Hardware deployment/playback results must be read from the deployment/trace receipts; a build or decoder fixture is not an acoustic or all-feature pass.

aptX HD static, maximum LHDC V5 headroom, AAC fragmentation hot-path allocation, complete host-layer comparison, and full feature/security/lifecycle/stress qualification remain open.

## aptX HD follow-up after the user's 2–3 second static report

The first candidate was flashed to COM6 at 921600, read back at 115200 with matching app and selector bytes, and booted the owned VALID image without automatic Bluedroid restore. Its existing 37 cold codec configurations passed. The protected original application remained unchanged.

A bounded native YouTube trace negotiated aptX HD 48 kHz / stereo / 24-bit, transported as S32 PCM. Within one stream epoch, measured encoded payload was 575.03 kbit/s. It recorded no new queue drops, decoder rejects, RTP sequence breaks, descriptor faults, or initial DMA refill shortages. The selected worker resident allocation was 12,736 B. This does **not** establish that the reported audible static is resolved; no acoustic recording was made. The wire RTP timestamp increment differs from the sample count implied by payload size, but actual payload throughput is consistent with 576 kbit/s, so that discrepancy is not by itself evidence of corruption.

A separate source defect was found in the aptX adapter: if freeaptx stops after a valid prefix on a parity error, its failed codeword has already advanced predictor/sync state. Retrying the same unconsumed bytes can emit corrupted samples. The adapter now detects a remaining whole codeword while at least one output codeword still fits, rejects the packet, and lets the worker report/drop/reset once. Six existing aptX/HD/LL reference comparisons still pass. A targeted HD parity-bit mutation verifies partial failure without replaying corrupted state. This defect has not been observed in the clean-counter YouTube trace; it is not yet established as the cause of the user's symptom.

A separate `dsp_signal_quality` UART record now reports the render owner's actual EQ gains, volume, flags, rendered frames, peak Q27 amplitude, limited channel samples and channel samples above full scale. Publication is requested by the logger, performed by the single active render owner, and read once with a generation fence. There are no retries, waits, extra PCM scans, allocations, callback printing, or reads of live DSP state from the logger/ISR. The retained snapshot/request fields use 44 bytes. It does not alter gain, limiter, precision, or output samples.

The queue-memory record calls its codec label `last_started_codec`: queue construction can precede the next stream START, and it must not falsely label the new allocation with the previous stream's codec.

The active baseline Helix wrapper also reserves a static `s_helix_buffer` (`sizeof(AACDecInfo) + sizeof(PSInfoBase) + 64`) even when AAC is not selected. The owned decoder does not link that wrapper; its corresponding workspace belongs to the selected AAC allocation. The original comment’s approximate 26/27 KiB estimate must not replace the actual target type sizes above. This comparison is about that allocation, not a complete system RAM benchmark.

The follow-up source-stable parity/DSP-logging candidate is `731b7664d73659a0595c9f0517cb6be4110920ea245613135ee5860e5cabe282`. Its implementation checks are separate from the earlier YouTube trace. Packet-loss recovery also needs review: resetting freeaptx’s parity phase does not establish synchronization with a continuing source. The library synchronization API must not be adopted blindly; its local input-size calculation and capacity-exit handling require review before integration. No radio-loss recovery or audible-static fix is claimed here.
