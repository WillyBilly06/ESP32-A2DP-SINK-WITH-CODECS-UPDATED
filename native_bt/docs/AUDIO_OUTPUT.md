# Experimental native SBC audio output

This is an incomplete development candidate, not a replacement audio release.
BTstack remains a reference only. The controller and standalone SBC codec come
from the existing SDK; the host protocols and output ownership are independent.

## Ownership and memory

The host owner routes copied encoded media to the decoder worker. The decoder
submits complete PCM blocks to a lazy input ring. A separate render task owns
integer DSP history and feeds an internal-SRAM Q1.31 stereo output ring. The I2S
callback only copies ready samples, applies the stop fade and wakes the renderer.
It does not decode, render filters, allocate, log or wait.

At 48 kHz stereo the output requests 44,032 bytes for its input/output rings and
three 128-frame DMA buffers. The SBC worker separately reports 8,114 requested
bytes. Both counts return to zero after close. These exclude allocator overhead,
static metadata, task stacks and the controller/SDK; they are not total heap use.
Only active stream storage is allocated. The rendered target is about 64 ms.

Controls use atomic snapshots; only the renderer mutates filter state. Integer
20 ms ramps handle control/start transitions, and an IRAM stop fade drains before
I2S teardown. Optional fractional clock correction stays within 500 ppm. Delay
reports estimate queued frames, DMA and codec delay; they are not measured latency.

The native build requires CONFIG_I2S_ISR_IRAM_SAFE. audit_audio_isr.py checks the
owned ISR roots/direct linked references and SRAM context. That audit does not
prove uninterrupted output during a physical NVS write. Every product EQ change
must still commit immediately; no debounce or deferred persistence is permitted.

## Verified finite diagnostic

Image SHA-256:
06fcd690a722a0c6a5397fd67a2875fc66fc1d3b6ba18b25a3af2af895ea8490.
Trial image/selector readback, boot and original application preservation passed.
Linux completed a fresh Classic pairing. The encrypted raw AVDTP source then sent
1,600 packets containing 8,000 SBC frames at 48 kHz stereo over 21.320 seconds,
with five frames per packet. This exercises larger packets than the earlier
four-frame diagnostic, but does not resolve the earlier generic corruption.

The ESP32 reported:

- 4,096,000 pre-DSP PCM bytes, matching expected FNV-1a-32 value 4109840158.
  This rolling diagnostic is not cryptographic proof of every PCM byte.
- I2S active, 8,090 DMA completions and 1,023,956 rendered/source frames completed.
  Fractional resampling explains why this differs from 1,024,000 input samples.
- Zero recorded decoder drops, sequence resets, descriptor errors or output failures.
- Concurrent 171 ATT writes, 17 readbacks and 171 notifications; 22 delay reports
  acknowledged. The final 75.5 ms report is an estimate.
- Maximum reported render block time 1,312 us and DMA callback gap 2,677 us.
  These finite counters do not qualify CPU performance, sample clock or acoustics.
- 10,156 underrun silence frames. The first occurred after all 1,024,000 input
  frames had been accepted; the sender deliberately waits 250 ms before Suspend.
  These terminal silence counters are retained and are not called zero underruns.

verify_sbc_output.py binds the source report, complete bounded UART window and
exact firmware. The portable suite passes 144 groups on Windows/Linux, including
ASan/UBSan, TSan, 20,000 concurrent HCI packets and 16,384 audio frames through
three real producer/render/consumer threads with concurrent control snapshots.

Ordinary BlueZ Audio Sink discovery/connection created a PipeWire 48 kHz stereo
output and negotiated joint-stereo SBC (configuration 11150235). A subsequent
reconnect failed before Firefox startup: an unexpected media-channel close
caused the platform to permanently stop admission. The session-retirement API
now requests cleanup without disabling the service, gates new sessions until
channels and worker DROP drain, and allows readmission. Its paired-runtime
regression passes. On the corrected image Linux completed two ordinary profile
sessions separated by a full disconnect without rebooting the ESP32.

The live drain trace also exposed an SBC worker lease bug: Close/Abort freed all
buffers but retained the previous signaling binding. A later configuration could
therefore fail under a new connection. Close/Abort now release that binding, and
an unsuccessful initial configuration also releases its empty lease. The actual
decoder regression failed before this change and passes afterward for Close,
Abort and allocation failure followed by a new binding, with ASan/UBSan and TSan.
These checks supplement the generic simulated worker tests.

## Firefox music and remaining shutdown failure

Image 044771f07904d6adce47434ef82463593d5d6650d684235e966a67990d785d54
passed trial readback/boot verification. An isolated Linux Firefox profile played
an original generated melody through the actual PipeWire Bluetooth sink. BlueZ
negotiated 48 kHz joint-stereo SBC, configuration 11150235. Browser time advanced
continuously during 22.119 seconds of overlapping ATT traffic: 184 writes,
18 readbacks and 184 notifications, with no browser waiting/pause/error event in
that overlap. The preexisting audio route and user browser profile were preserved.

The ESP32 reported 8,910 decoded SBC frames / 4,561,920 PCM bytes, zero recorded
decoder drops/resets, zero I2S underrun frames or descriptor errors, and output
active during the stream. Requested worker storage was 8,123 bytes for this joint
stereo configuration; requested output storage remained 44,032 bytes. Both were
released afterward. These are finite digital counters, not acoustic measurements.

The overall run FAILED: the host reported DBT_FAULT_BAD_PACKET and Linux's profile
disconnect timed out. The UART window has no per-row wall timestamps, so it does
not establish an exact fault time relative to the browser overlap. Do not call
this a clean playback/teardown qualification. verify_firefox_output.py records
browser progress separately, retains the fault, and returns failure for this run.
The malformed-packet origin and shutdown recovery remain release blockers.

After testing, the newly created Linux diagnostic peer was removed and the
original Bluetooth daemon command restored. The Bluedroid audio trial passed
full image/selector readback and boot verification on restoration; the original
application remained intact. A read-only SDK parse of live NVS found the owned
native_bonds namespace with zero written entries. The short cleanup UART window
missed the delete event, so the persisted flash state supplies the deletion check.

## Unqualified behavior

No physical I2S capture, acoustic test, physical NVS stress, power-loss persistence,
long-duration or all-codec qualification is represented here. Short streams below
the prefill threshold can be discarded on
Suspend; product short-sound behavior remains incomplete. AVRCP, product BLE/OTA,
BLE bonding, other requested codec workers and broader profiles remain pending.
No equivalent-feature RAM, CPU or latency advantage over BTstack is established.
