# Media arrival versus host timing, 2026-11-08

COM6 runs owned-host image `77101dff57957ff24ca8ea4eb11fbd6ea0462a1d0682e3f51d8e672e46a03de4`, verified by full application readback. This checkpoint adds timing diagnostics only. It does not change codec arithmetic, credits, buffer geometry or scheduling policy. The master goal remains active and incomplete.

## Implementation

The existing media tuple now records the interval between two adjacent RTP packets' associated HCI ingress observations, separately from their host callback interval. It uses existing timestamps and queue identities; it adds no packet clocks, queues, allocation, UART work or reader retry. Same epoch/SSRC, adjacent sequence, valid associations and a forward low32 microsecond interval are required. Missing/fragmented associations, discontinuities and incoherent snapshots report unavailable rather than inventing an arrival time. Two-phase admission completion preserves an interval only while its packet association still matches.

The background `media_ingress_timing` record publishes the latest coherent tuple at the existing detail cadence. It is not a per-packet history. Frozen `starvation_media` and `unfilled_media` records retain the HCI interval at the first relevant event. HCI ingress is an observation at the host/controller boundary, not RF reception. Delays before that point can include source pacing, controller scheduling and host credit backpressure.

Static changes versus normal image `b302e29ecaed`: DRAM data +16 B including alignment, BSS unchanged, IRAM text +24 B, flash text +708 B. Normal 2x192-frame DMA, 1024 raw PCM frames, 2560 processed frames and the 31744 B PCM/DMA payload budget remain. No performance improvement is claimed from these section sizes.

## Validation

Actual output/owner/audio/DSP source passed ASan/UBSan cases at 100 and 1000 Hz virtual ticks. New cases cover distinct host/HCI intervals, split admission, changed associations, duplicate/skip, epoch/SSRC changes, unavailable input, sequence/time wrap, backward time and in-progress publication. Existing bit-exact split/full DSP EQ/volume and refill ordering/cancellation cases remain passing. Source digests still match the tested files.

The default ccache build admitted 1036 object compilation units and rechecked 10433 source inputs. Existing target cold checks passed 37 configurations / 173 callbacks, with 56899 B tracked peak codec allocation, 106088 B minimum free after allocation, 3572 B codec-test stack margin and 12697 us maximum decode. These are cold tests, not live real-time headroom. Boot watchdog, idle-starvation and backtrace counters were zero.

## Bounded Redmagic observations

Only Redmagic `9125299103B7` and COM6 were used. Fresh phone UI readback showed LHDC V5, 192 kHz, 24-bit, stereo, fixed 900 kbps. This is not the required 1 Mbps configuration. Native YouTube was guarded by an independent 14-second phone-side pause process. The duration bounds below use verified MCU stream-start acknowledgement through confirmed renderer stop, including startup.

| Excerpt | Duration upper bound | Initial / recovered / unfilled frames | BLE connect delay | Controls completed before pause |
|---|---:|---:|---:|---|
| Music | 12.635856 s | 0 / 0 / 0 | 3144 ms | EQ ACK, independent settings readback, original CCCD write ACK |
| Metronome | 12.797496 s | 3264 / 0 / 3264 | 8119 ms | No; controls completed after pause |

Both helper receipts completed with the requested nonce. Neither excerpt reported media drops, decoder faults or descriptor errors. That does not establish acoustic quality, sustained coexistence or complete feature parity. CCCD restoration means acknowledged descriptor write; an additional descriptor readback was not performed. Stopped free heap was 62044 B, largest block 59392 B; boot-to-date minima were 20132 B after music and 18860 B after metronome, not isolated per-excerpt allocation measurements.

At the metronome's first unfilled event (MCU 725148023 us), both PCM queues were empty. Decoder intent was `wait_encoded_input` since 725138706 us with zero encoded packets; renderer was outside audio render and the DSP helper was idle. The latest associated packet was RTP sequence 5033 / timestamp 4832640 / SSRC255, observed at HCI 725125937 us and admitted at host 725127487 us. Its adjacent HCI interval was 11258 us versus host interval 10362 us; host begin followed this packet's HCI observation by 1270 us. Thus this frozen event does not show the decoder occupied computing while input waits.

There was no verified pre-pause UART witness for this event, and cached AVRCP remained PLAYING without a new peer PAUSED update. Its phase remains unclassified. Do not label all 3264 frames stop-tail, infer radio loss, or use this one tuple as proof that HCI/credits are healthy throughout the stream. Earlier confirmed active gaps remain relevant.

## Separate integration gap

The preserved Bluedroid configuration enables `CONFIG_BT_BLUEDROID_ESP_COEX_VSC`. Its A2DP start path sets STREAMING and clears PAUSED; stop clears both. Equivalent signalling was not found in the owned host. Espressif describes the [scenario-status vendor HCI command](https://github.com/espressif/esp-idf/issues/12434) as an RF coexistence input for upper-layer use cases. That documentation does not establish that this omission caused the current Classic/BLE gaps.

The checked local wire definitions give opcode `0xFC82` and parameters `type,operation,status`: BT type2, SET1, CLEAR0, STREAMING0x10 and PAUSED0x20. This checkpoint has not submitted this command or implemented the missing integration. An owned asynchronous implementation must retain command ownership/backpressure and verify controller completion, without linking Bluedroid or silently falling back.

## Evidence and handback

[Structured results and digests](MEDIA_INGRESS_TIMING_RESULTS_2026_11_08.json) bind the source, tests, image, footprint and excerpts. Raw UART/helper receipts, exact capture scripts, phone UI, ELF and application readback are retained locally under `native_bt/_build/hardware/runs/1794209900538208000-77101dff5795`; these large local artifacts are not published in Git. The image was built from the recorded parent commit plus the four source/test digests in the receipt.

BDK Audio's main app was restored to the foreground. YouTube is explicitly PAUSED, I2S stopped, Classic AVRCP connected, and the identity-verified UART viewer remains at921600 baud. The owned ota1 selection remains VALID with no automatic Bluedroid restore; the prior ota0 application is preserved. COM7 was not accessed.

Remaining work includes the missing coexistence-status integration, variable BLE establishment, unexplained audio gaps, peer playback-state updates, sustained192k/S24/1Mbps headroom and complete codec/product/security/lifecycle/OTA/acoustic qualification.
