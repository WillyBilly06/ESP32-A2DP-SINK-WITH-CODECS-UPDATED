# Exact volume-ramp block reuse - 2026-11-06

Volume-only changes no longer need to force every stable filter through the scalar ramp prefix. The opt-in -VolumeRampBlock path advances the changing compensation section (filter3) once per stereo frame using the original filter, reuses the existing block kernels for the other stable sections, and switches the compensation tail to its steady helper at the exact ramp endpoint. General EQ/boost coefficient ramps retain their canonical prefix. Gain, coefficient values, fractions, histories, limiter, routing and20ms ramp duration remain unchanged.

There is no new PCM buffer, coefficient array, DSP instance, task or heap allocation. Existing bounded128-frame spans and split scratch are reused. The DSP object layout is unchanged. This is a fixed-point DSP optimization; the LHDC transform is not converted or degraded.

## Exactness and control coverage

ASan/UBSan comparisons to a separately compiled canonical scalar reference pass67040 blocks /3833465 PCM frames in EACH proven/split-fallback mode, across13 rates. The additional pure-volume cases exercise17934 ramp blocks, all16 routing/analysis flags, interrupted and settled ramps, variable sizes0..128, silence, impulses, random values and rails. Staggered EQ/volume/bypass/flip changes test unequal ramp completion times. Full PCM and every DSP-state byte match after every block. The2,001,042-case independent signed64 quantizer oracle also passes. Both comparison modes are explicitly exercised; host timing is a stub and is not used as speed evidence.

The AVRCP app queue is already16 entries; it was not enlarged. A new full16-label SetAbsoluteVolume burst passes both output modes through the actual AVCTP/AVRCP code and current SDP discovery dependencies, with channel/runtime models. Every request receives its own reply; repeated polls while a runtime borrow is held preserve that buffer and apply each volume exactly once. Earlier test-harness dependencies and the obsolete20ms close-poll expectation were corrected to the current event-driven contract. Failed harness receipts are retained. These are offline exchange checks, not RF flood qualification.

## On-device arithmetic comparison

Image5ddc0eaa078e7b358c125a701a8f0269239dcb2b602dfeb8dfb6d34f466d6077 enables -VolumeRampBlock -VolumeRampCompare, with OwnerPacer OFF. All product/security/RFCOMM and codec options remain enabled. Native YouTube on the approved Redmagic, COM6 only:192k/S24/stereo/fixed900, received566 media bytes/one5ms frame (899200 compressed bits/s). Real keys change phone110 ->80 ->110 and DSP target93 ->68 ->93. The starting volume is independently restored. Play/pause interval12.073s, with an independent14s pause guard and post-stop UART capture.

The comparison alternates ABBA on real audio. Each block executes once; there is no duplicate decode or shadow audio processing. Timing covers only full128-frame compensation-only ramps with stable routing. Clipped blocks are counted separately and excluded from timing; none are excluded in these four rows. Per-core records are keyed by rate and EQ/flag policy. Results are printed only after audio stops. Disassembly verifies cycle reads immediately surround the kernel call. Source/frame arithmetic equality is independently covered by the host oracle above.

| Core | Calls per mode | Canonical mean cycles | Candidate mean cycles | Observed mean reduction | Canonical minimum | Candidate minimum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 62 | 187879.27 | 132626.66 | 29.41% | 183045 | 127976 |
| 1 | 27 | 255564.26 | 207240.26 | 18.91% | 200853 | 150131 |

All rows are192k, EQ2/8/12, flags12, and have verified UART CRCs. Cycles include preemption, cache and normal real-time interference; these are observed elapsed-cycle results for rotating neighboring real-audio blocks, not identical-waveform isolated CPU replays. In particular Core1 averages include larger scheduling outliers. Do not convert this table into a full-pipeline headroom guarantee or assume all rates/configurations have the same speedup.

The full LHDC run still has539136 unfilled frames after late recovery. The mathematical optimization is useful but does not fix sustained maximum-rate playback. Metronome-specific acoustic behavior and fast continuous app-slider changes remain open.

## Selected path and remaining gates

The selected full-product build keeps -VolumeRampBlock and removes -VolumeRampCompare and OwnerPacer. This removes the A/B selector, optional136-byte timing recorder and post-stop comparison logging from the active arithmetic path. The selected compile mode independently passes the same67040-block /3833465-frame-per-mode PCM/state gate and quantizer oracle. Image24f9ec682af8567717df7759bddc54f460bc7ec13e459378aa7bb97a08125929 selects the candidate with both comparison and OwnerPacer OFF. It passes app/selector readback and owned VALID boot with the same37 cold cases. A selected-path LDAC96k regression sends10 down and10 up key presses, with phone readback110 ->10 ->110 and DSP target93 ->8 ->93. All1440000 accepted frames are rendered and delivered to DMA; initial shortage, recovered, unfilled and descriptor-error counters are all zero, including the captured stop/drain tail. Max EOF gap1382us; post-stop free heap97756/minimum62800/largest81920, decoder stack low-water6152. The kernel's linked image has125255 IRAM-text,25189 DRAM-data and50664 BSS bytes.

The phone only enables32-bit input for this LDAC mode (16/24 are gray), so that supported setting is recorded without reducing depth. Its LDAC quality choices render as blank labels; the dialog is cancelled and existing quality retained. This is not a verified maximum-bitrate LDAC test. Although the input command requests20ms between keys, actual phone AudioService history shows down commands about177–183ms apart and up commands22–27ms apart (ten up changes over223ms). The exact action timestamps and phone history are preserved, rather than assuming requested delay equals RF cadence. The12.143s play/pause interval uses an independent14s pause guard. This one passing digital stream is not acoustic certification or proof all codecs, continuous slider patterns and maximum LHDC now pass.

Exact source/config/ELF/image and app/selector921600-write/115200-readback archives are under _build/hardware/runs. Experiment data, old/new kernel rows, disassembly, unit receipts and failed test-harness logs stay private under _build/experiments/lhdc-dsp-assist-1794002414693. The comparison image boots our owned host VALID with37 existing cold codec cases; no new boot cases or Bluedroid fallback are introduced. Readback/cold checks do not qualify audio continuity.

The option-disabled block path independently passes the same extended PCM/state checks; baseline selection remains available.

Full1000kbps, sustained192k pipeline headroom, BLE/control/storage/OTA coexistence, repeated security/lifecycle, all-codec rapid-volume/EQ acoustics and the full feature inventory remain incomplete. Fixed900 is still the Redmagic's highest fixed UI choice. No codec, sample precision, DSP section, security or product feature is removed.
