# Bounded six-block DSP assistance — 2026-11-07

The full Bluetooth/audio goal remains active and incomplete. The retained development build increases the explicit Core 0 helper cap from five to six canonical 128-frame blocks. This does not enlarge a PCM or packet buffer, change audio arithmetic, or alter priorities/core affinity. Four blocks remain the source/build default.

## Ownership and admission

`-DspAssistBlocks 6 -DspAssist` permits at most 768 frames per lease. The helper publishes every completed span and returns on empty input/output or any failure; it never waits for new PCM. The owner may service the guarded DMA refill from already-published PCM while the helper retains its DSP/lifecycle borrow. DONE still precedes lifecycle retirement and the separate local block. Decoder controls can progress at a zero media budget. Attached sound retains its per-block readiness restriction.

CMake, the compiled header, PowerShell CLI, build receipt, image gate and actual per-unit compiler definitions now admit only 4, 5 or 6. Nondefault values require assistance; the explicit default 4 argument clears a previous selection. Actual admission checks pass 21 CMake, 54 image, 10 header and 3 early CLI cases. The existing ASan/UBSan two-core output/lifecycle checks pass in fair/early/rebalanced mode at all three caps and ordinary six-block mode. They cover the cap plus one extra local block, page crossings, full-ring reuse, held BUSY ownership, stop/cancel, completion before producer preemption, sound signals and unchanged other formats. No new boot test cases are added.

## Exact image and memory

Baseline image `7fbb6ccc514675bf201b2ed07b26268b0a8938965169014dd9ce48a4324025d3`; candidate `f2637213bb695ca5123f591a6575d35ac75be0c7065a1c456cac127d92003dc8`, archive `runs/1794045912385558800-f2637213bb69`. The full product builds with ccache and all codec/product/security/RFCOMM options. Temporary task-cycle accounting is OFF. The app and selector full readbacks match after write 921600/read 115200, owned VALID boot and the existing 37 cold codec checks pass; original app/table/bootloader are preserved and no Bluedroid fallback occurs.

ELF DRAM data/BSS, IRAM, flash text and rodata sizes are unchanged. The raw/processed/DMA payload remains 39936 bytes: 2048 decoded frames, 2560 processed capacity with a 2048-frame target, and 3 x 128 DMA frames. One LHDC 5-ms decoded unit remains 960 stereo frames / 7680 S32-container bytes. Cold-check tracked peak is 56899 bytes, minimum free after allocation 103608 and stack low-water 3520 bytes; these are cold checks, not active RF heap qualification.

## Guarded native YouTube observations

Redmagic 9125299103B7, native YouTube 120 BPM metronome. UI LHDC V5 192 kHz/24-bit/stereo/fixed 900, actual phone volume 80/150, sampled DSP volume 68/EQ 2,8,12/flags 12. Observed maximum media payload stays 566 bytes, with no oversize rejection. Every commanded interval has an independent 14-second pause guard. Final aggregates include startup and post-pause drain; they are not acoustic durations or a complete per-sample deadline proof.

| Mode | Command s | Accepted/rendered/DMA frames | Initial shortage | Recovered | Unfilled | Copy/mask max us |
|---|---:|---:|---:|---:|---:|---:|
| Five-block baseline | 12.248 | 2803712 | 234752 | 118144 | 116608 | 6/9 |
| Six-block run 1 | 12.086 | 2779200 | 186944 | 79296 | 107648 | 6/9 |
| Six-block run 2 | 12.251 | 2825280 | 125504 | 51136 | 74368 | 4/7 |
| Six-block run 3 | 12.211 | 2844288 | 129280 | 56960 | 72320 | 4/9 |

Unfilled totals are 7.7%, 36.2% and 38.0% lower than the fresh baseline. Short successive segments, reboot/history and run variation prevent an isolated throughput or sustained-headroom guarantee. All three exercise the 768-frame cap. Helper elapsed cycles per DSP frame stay about 833.6–834.4 versus 833.5 in the baseline; no arithmetic speedup is claimed. Complete active runtime windows show Core 0 decoder about 50.4–54.8%, helper 34.9–38.3% and idle 9.4–12.8%. These task counters include IRQ and exclude running tails; observed idle is not proven deadline headroom.

No descriptor/contract/helper faults or new local queue/decode/sequence rejection occurs. PCM-busy observations are 6159 in the baseline and 5129/5250/5620 with six blocks. Detailed worker-ring losses are 2581 versus 2138/2209/2373, retained separately from zero host-detail, aggregate-record and UART losses. Those losses prohibit complete call-by-call claims.

Every captured first shortage still has decoded input waiting and empty processed output: 1856 raw frames in the baseline, then 1088/1728/1984. Consumer timing remains unresolved. Each run also has a verified pre-decoder maximum RTP step of 27840 rather than nominal 960 ticks. Aggregate RTP-span excess is 107520 ticks in the baseline and 80640/53760/53760 in the candidate. This correlation does not prove sender flushing, receiver credit pressure or RF scheduling as the cause. No decoder reset or radio packet-type policy is added.

## LDAC regression and remaining gates

On the same image, LDAC 96 kHz/stereo with 32-bit phone input and optimized-quality selection runs for a 12.246-second command interval. Accepted/rendered/DMA counts equal 1451264, with zero initial shortages, unfilled frames, descriptor errors or local queue/decode rejection. The clock resets to 512 ticks. Sampled 661-byte media packets minus the one-byte LDAC header at 512 ticks/96 kHz are consistent with 990000 compressed bits/s; this is wire-sample inference. The helper is idle for LDAC. Phone Bluetooth PID 8879 stays stable; native/proto/HAL dumps are not retried.

The phone is left paused and COM6 raw UART monitoring is listening. Exact test/build/readback/traces and analyses are under `native_bt/_build/experiments/assist-six-1794045681796`; [structured results](ASSIST_SIX_BLOCKS_RESULTS_2026_11_07.json) bind the image and separate raw records. Sustained maximum 1000 kbps LHDC, continuous loss-free deadlines, encrypted coexistence/control/storage/OTA stress, all-codec acoustics and complete feature parity remain unqualified.

## Retained build selection

```powershell
.\native_bt\tools\build_probe.ps1 -DspAssistBlocks 6 -DspBlockIram -ConsoleIrq -TaskRuntimeStats -HighRatePcmRebalance -EarlyInputRelease -RenderFairShare -VolumeRampBlock -DspAssist -UnityPcmCopy -StayNative -WordDsp -PriorityOrder -CodecAwareRender -SchedulerTrace -AudioStageProfile -PcmCopied -BlockDsp -PcmTargetMs 16 -DecoderCore 0 -LoggerCore 1 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir 'D:\ESP32 Bluetooth\LHDC-V5-Decoder-main'
```
