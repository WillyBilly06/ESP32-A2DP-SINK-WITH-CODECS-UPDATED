# DMA cadence and lightweight first-gap diagnostics

This is an unqualified development checkpoint. LHDC continuity and the full
Bluetooth/audio master goal remain **open**. The current COM6 image is
`de1ef69d17fbc4ad0eeeb56c7a4f683d8b4fcca2896fb02ac43839724f982167`, archived at
`native_bt/_build/hardware/runs/1794122315321907100-de1ef69d17fb/`.

## Runtime changes

- DMA geometry changes from 3 x 128 to **2 x 192 stereo sample frames**.
  Both use 384 frames / 3,072 payload bytes. The installed IDF6.1 driver admits
  two descriptors. At192kHz the EOF cadence changes from1,500 to1,000Hz and the
  conservative refill interval from666 to1,000us. Deadline guards, prefix order,
  cancellation, bit depth, codec math and total DMA payload are preserved.
- Native runtime now keeps the bounded startup, media, decoder, renderer and
  initial/final gap latches independently of the optional scheduler profiler.
  IRQ capture uses internal SRAM and lock-free words; separate JSON records
  are formatted only by the existing priority1 Core1 logger. There is no new
  task, audio buffer, synchronous callback output or polling wait.
- Current build disables `SchedulerTrace` and `TaskRuntimeStats`, retaining
  per-stage health counters, `AudioStageProfile`, `LhdcStageProfile`, all codec,
  Bluetooth, security, DSP and product build options. The original priorities
  and cores remain. UART and the full raw web logger remain **921600 baud**.
- Exact current-HCI-to-RTP association and the experimental VHCI readiness
  observer remain available in detailed tracing mode. Lightweight records
  explicitly mark unavailable association/pending-ACL fields invalid. They do
  not infer zero pending packets. No transport readiness decision or fence is
  changed. The earlier5182 read-only readiness observations found no confirmed
  held slot or SDK-ready-while-held mismatch in their two windows.
- The high-rate LHDC startup guard is50ms, measured at50,234us in the current
  metronome window, with seven encoded slots including the active slot. This
  adds10ms versus the earlier40ms policy without buffer growth. It remains a
  provisional jitter policy; an isolated benefit is unproven. Lower-rate LHDC
  retains40ms. Short accepted clips still use the existing safe drain path.

## Hardware observations

Approved Redmagic9125299103B7, native YouTube120BPM metronome and official
Get Lucky audio at30s, COM6/PCM5102A, no encoders. UI was read back as LHDC V5,
192kHz,24-bit,stereo,fixed900kbps,volume150. This is **not live1Mbps proof**.
Every owned window has an independent phone-side14-second pause guard and
ends with YouTube paused. Totals include startup and postpause drain.

| Image/mode | Fixture | Unfilled PCM sample frames |
| --- | --- | ---: |
| e792,3x128, detailed tracing | Metronome control | 5248 |
| 03f6,2x192, detailed tracing | Metronome / music / metronome repeat | 512 / 3168 / 704 |
| b1e0,2x192, profiling off, no gap latches | Metronome / music / metronome repeat | 1344 / 192 / 3968 |
| de1e,current lightweight gap capture | Metronome / music | **768 / 2880** |

These short sequential windows are not reversed or randomized A/B qualification.
They do not prove an isolated continuity improvement. Removing profiling did
not fix underruns. Callback charge per million source frames was about51,324us
in the control and41,222–41,355us in the three03f6 windows, an observed lower
callback cost for that scope, not whole-pipeline headroom.

Current windows deliver all2,526,976 and2,528,640 accepted frames through rendering
to DMA. Descriptor and refill-contract errors are zero; copy/mask maxima are
5/7us or lower. Captured maximum adjacent RTP steps remain960 ticks. No corrupt
checked record is observed. All four separate finalized-gap records and startup
metadata are present; heavy tick/host-cost/task-runtime rows are absent.

Both current first finalized gaps have768 decoded frames, zero processed output,
a decoder waiting for encoded input and local rendering already in progress.
Other earlier gaps have empty input as well. This distinguishes several observed
states; it does not assign every gap to a single cause or rule out RF retries.
Active task-counter intervals from the03f6 comparison show Core1 only1.8–3.9%
idle while Core0 has about15–22%. Those counters include interrupts and exclude
running tails. The next scheduling experiment must preserve protocol/control
progress and demonstrate a benefit rather than simply raising priorities.

## Memory and checks

Compared with03f6 detailed tracing, the current ELF uses876 fewer IRAM bytes,
1,320 fewer data bytes and648 fewer BSS bytes, with9,248 fewer flash-text and
4,160 fewer rodata bytes. The current existing37-configuration /173-callback
cold check passes: tracked peak56,899B, min free after allocation106,740B,
stack low-water3,532/8,192B. Min free is4,808B higher than03f6; these are cold
checks, not active RF heap. Near paired running-I2S snapshots, observed free
internal heap is31,152B, largest block28,672B and codec resident60,704B. Lowest
reported internal free since boot reaches28,592B. These adjacent heap/output
observations are not an atomic whole-system snapshot.

Full ccache build, compiler-input rechecks,10,413-source manifest, image admission,
written-region readbacks and owned VALID boot pass. Original application/table/
bootloader preserved; automatic Bluedroid fallback false. Runtime logging is
PID34852/source83 at921600 after the final deployment.

Actual-code ASan/UBSan tests pass at100/1000Hz in lightweight and detailed modes
for initial/final latches, partial refill ordering/expiry/cancel, startup/wrap and
exact split/full DSP equality with EQ/volume changes. Helper ownership/bounded
batch, all compact format/rate checks and media-arrival identity/SPSC checks
pass. Six phone guard lifecycle/reset tests pass. IRQ root/direct-reference
audits pass, including the gap-capture/ACL helpers and internal data contexts;
this is not a complete transitive-callgraph or timing proof.

The broad format fixture had stale256-frame expectations at8/48k. The same
failure was reproduced with both DMA geometries before correction to the
existing512-frame minimum and corresponding allocation expectations. No runtime
allocation policy was changed to satisfy the test. Test PDBs and fixed configure
logs are losslessly archived; completed AMD64 COFF intermediates are retired
only after executable success with hash manifests. Failed receipts/binaries,
sources and tested executables remain. The512MiB build cap is unchanged.

Reproducible source checks:

```powershell
python native_bt/tools/test_probe_output_format.py --compact-copy --block-dsp --priority-order --word-dsp --starvation-trace --light-gap
python native_bt/tools/test_probe_output_format.py --compact-copy --block-dsp --priority-order --word-dsp --starvation-trace
python native_bt/tools/test_dsp_assist_output.py --fair-share --early-input --rebalance --assist-blocks 6
python native_bt/tools/test_media_arrival.py --out native_bt/_build/media-arrival-check
python native_bt/tools/test_phone_boot_guard.py
python native_bt/tools/audit_audio_isr.py
python native_bt/tools/validate_local_image.py
```

Evidence folders: `native_bt/_build/experiments/dma-interval-1794119819663/`,
`profiling-off-1794121065059/`, `light-gap-1794121696386/`, and the earlier
`vhci-ready-1794114626999/`. Guarded deploys use expected-image admission before
port access,921600 writes,115200 full readback and finally logger resume.

Continuous1Mbps/headroom, simultaneous maximum BLE load, long-duration clocks,
every-codec acoustics, EQ acoustics, complete feature/security/lifecycle/OTA
parity and comparative whole-host RAM qualification are not proven. No production
release or main-branch replacement is justified by this checkpoint.
