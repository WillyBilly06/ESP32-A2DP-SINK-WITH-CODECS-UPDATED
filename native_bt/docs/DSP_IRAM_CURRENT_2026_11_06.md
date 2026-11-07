# DSP instruction-RAM placement with the current scheduler — 2026-11-06

The complete Bluetooth/audio goal remains active and incomplete. The two-block lease and capacity-copy candidates are rejected in PCM_COPY_CREDIT_2026_11_06.md. Their extra runtime paths are removed. This placement comparison restores the four-block/fixed-transfer baseline and retains the measured exact analyzer accumulation.

The earlier placement trial predates the current fair-sharing, interrupt-driven console, copied-input release and helper pipeline. It did not establish a benefit. This is a fresh measurement of the existing opt-in -DspBlockIram option under the current full product configuration, not a claim that the earlier result passed.

## Exact placement and footprint

Candidate d2676c5840562111b56f2f790b0fb2441d798cb111627cb780cfef1ffdc99723 passes ccache build, source stability, quota and offline image admission. ELF symbols place analysis at 0x400899cc, filter_steady_block at 0x40089d4c, filter_volume_ramp_block at 0x40089e58, limit at 0x40089cf0 and fixed_dsp_process_block at 0x4008a46c. The fused analysis helper is incorporated into the compiled block path rather than requiring a standalone symbol.

Relative to b31d1013ab8b, IRAM text increases 3756 bytes and flash text decreases 3776 bytes. DRAM data, BSS, vectors and rodata are unchanged. This consumes instruction SRAM; it is not a memory saving. Codec, quality, buffer geometry, priorities, core affinities, fixed-point arithmetic and control behavior are unchanged. No new task, queue, audio allocation or boot case is added.

All 80 maintained real-codec deferred cases pass again after removing the credit API, including corrected valid LHDC multi-frame packets and malformed count/header rejection. Existing worker/loan ownership checks pass. These host checks do not establish ESP32 placement speed, acoustics, maximum throughput or complete feature parity.

Guarded COM6 deployment and same-configuration native YouTube metronome measurements are recorded below. The operator must restore 192 kHz/24-bit/stereo and fixed900 after reconnect, verify actual volume80/150 and sampled DSP68/EQ2,8,12/flags12, preserve independent14-second pause guards, and keep compressed wire bitrate separate from the unqualified 1000 kbps target.

## Current physical result

Guarded deployment at write/read rates 921600/115200 passes full app/selector readbacks, owned VALID boot, original application preservation and the existing 37 cold codec cases. No automatic Bluedroid restoration occurs. Exact archive: runs/1794028094679160100-d2676c584056.

Actual Redmagic UI restores LHDC V5 192 kHz/24-bit/stereo/fixed900 and volume80/150. Both native YouTube metronome traces sample DSP volume68, EQ2/8/12 and flags12. Wire framing remains566 media bytes/one5 ms frame, consistent with899200 compressed bits/s. These are not live1000 kbps passes.

| Same selected metronome | Flash DSP baseline | IRAM first | IRAM repeat |
|---|---:|---:|---:|
| Commanded play/pause interval (s) |12.208|12.212|12.195|
| Accepted/rendered/DMA source frames |2553600|2654912|2640000|
| Helper elapsed cycles/frame |847.392|831.077|832.526|
| Local elapsed DSP cycles/frame |1488.527|1359.097|1376.032|
| Helper share of source frames |60.42%|54.92%|56.25%|
| Initial shortage frames |529024|404480|412416|
| Late recovered frames |169472|150016|142592|
| Unfilled frames |359552|254464|269824|
| Descriptor errors |0|0|0|

Each play interval has an independent14-second pause guard and post-pause collection. This repeatable scoped improvement supports retaining instruction-RAM placement in the current development image. Separate intervals/boots, preemption, startup and drain prevent presenting these aggregate numbers as an isolated overall speedup or sustained-headroom guarantee. Local elapsed DSP cost is lower in both runs, but LHDC still misses deadlines. The first placed-code gap has1856 decoded frames waiting and zero processed output.

First-run PCM backpressure/detail-ring loss are7850/3435. Repeat deltas are8094/3542. Queue/decode rejection, PCM failures/timeouts, input sequence breaks, ingress rejection, watchdog events, descriptor/contract errors, formatted-record losses and UART-write failures remain zero in these observations. The detail-ring losses prohibit complete call-by-call timing claims. Active internal8-bit heap free/largest stay19604/18432 bytes; the candidate lifetime minimum is19604 and decoder stack low-water5960. Instruction SRAM cost remains3756 bytes; no data-heap saving is claimed.

The repeat starts and ends inside CPU-record collection groups. An initial private analysis incorrectly treated missing edge roles as changed task identity. The completed playback was not repeated for that observation error. Analysis now excludes explicitly incomplete groups, requires every role indicated by the group mask, rejects real identity changes, and handles uint32 runtime deltas. Two partial edge groups are retained in the analysis report; complete groups show stable task numbers/core assignments and verified CRCs. Synthetic raw-counter wrap and identity-rejection checks pass. No firmware observer was changed.

## LDAC regression

The same image switches through Android codec selection to LDAC96 kHz,32-bit phone input/stereo, without re-pairing. A12.220-second native YouTube metronome command interval plus guarded drain accepts/renders/delivers1442048 source frames. Initial shortages, late-recovery demand, unfilled frames and descriptor errors are all zero; no captured first starvation exists. Sampled DSP remains68/EQ2,8,12/flags12. This is a finite digital regression, not an acoustic or all-feature qualification.

Observed consecutive wire packets have661 media bytes, one media-header byte and RTP increments512 at96000 Hz. The calculated compressed rate is660*8*96000/512 =990000 bits/s for those samples. The UI quality labels are not used to assert a bitrate. The phone is left paused on the metronome with LDAC96 selected and volume80/150. Future LHDC tests must explicitly restore its rate AND fixed quality.

## Reproduction and remaining gates

Use the existing full native product recipe, adding -DspBlockIram to the existing ConsoleIrq, TaskRuntimeStats, HighRatePcmRebalance, EarlyInputRelease, RenderFairShare, VolumeRampBlock, DspAssist, UnityPcmCopy, StayNative, WordDsp, PriorityOrder, CodecAwareRender, SchedulerTrace, AudioStageProfile, PcmCopied and BlockDsp selections; keep target16 ms, decoderCore0/loggerCore1, AsyncAudio and every product/security/RFCOMM/codec option. Do not enable the rejected smaller-lease/copy-credit prototypes or arithmetic comparison recorders.

The master goal remains active. Sustained maximum LHDC, complete BLE/control/storage coexistence, remote-volume acoustics, all-codec lifecycle/stress and full feature parity remain open. The next investigation should separate local DSP execution from host/controller preemption and renderer service overhead, rather than repeating rejected buffer-policy changes or claiming observed idle time is proven deadline headroom. Full evidence is under native_bt/_build/experiments/dsp-lease-1794026333302 and protected hardware run archives.

The exact retained build selection in this workspace is:

```powershell
.\native_bt\tools\build_probe.ps1 -DspBlockIram -ConsoleIrq -TaskRuntimeStats -HighRatePcmRebalance -EarlyInputRelease -RenderFairShare -VolumeRampBlock -DspAssist -UnityPcmCopy -StayNative -WordDsp -PriorityOrder -CodecAwareRender -SchedulerTrace -AudioStageProfile -PcmCopied -BlockDsp -PcmTargetMs 16 -DecoderCore 0 -LoggerCore 1 -AsyncAudio -WithProductIo -WithRfcomm -WithLhdc -WithOpus -WithAac -WithAptx -WithLdac -LhdcSourceDir 'D:\ESP32 Bluetooth\LHDC-V5-Decoder-main'
```
