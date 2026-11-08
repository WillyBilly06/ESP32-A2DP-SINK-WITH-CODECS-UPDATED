# Lightweight packet-bound LHDC gap evidence

The existing arrival tracker now runs with the native first-gap capture without
enabling SchedulerTrace. CMake links probe_arrival.c in that mode; the tick
sampler, owner cost profiler and readiness observer remain absent. The change
adds 112 bytes of persistent DRAM, 96 bytes IRAM, 1,048 bytes flash text and
240 bytes rodata versus image 0510a1c220e4. It adds no audio capacity, allocation,
task, payload copy or per-packet UART record. The existing cumulative arrival
report runs once per second in the background logger at 921600 baud.

The controller callback publishes cursor/time/size/ACL flags before the real
HCI queue publication. The owner matches the actual post-pop cursor and header.
Media association requires a complete current Basic L2CAP/RTP packet with the
same sequence/timestamp/SSRC and a payload view inside the current HCI buffer.
Fragmented, deferred and raw views remain explicitly unavailable. Metadata
rejection, overflow or mismatch invalidates timing instead of guessing.
Published/unretired metadata includes current callback/dispatch ownership; it
is not solely the real HCI queue occupancy. VHCI callback time is distinct from
RF reception inside Espressif's controller.

## Verification

- Existing actual-source ASan/UBSan checks pass: 36,864 cursor/header/time cases
  and 20,000 actual host SPSC packets, including events, three credits, wrapping,
  repeated peek, ownership, invalidation and complete RTP identity. Removing
  cursor identity fails the intentional negative control. Its FAIL output is
  expected; no hardware timing is inferred from this host test.
- The first run passed functional checks but failed the 512 MiB output gate at
  512.417 MiB. Receipt is preserved. 222 typed AMD64 COFF intermediates were
  archived with on-disk gzip/hash round-trip verification; test executables and
  failed receipts remain. Source hashes still match the 188/188 host checkpoint.
  The complete rerun passes. Target build finishes at 511.834 MiB rebuildable.
- ccache target build, compiler-input rechecks, 10,414-entry source stability,
  offline image admission and existing actual-ELF ISR audit pass. The new
  72-byte pending getter is in IRAM, has no calls or retry loop, and its three
  literals and 112-byte state are in internal memory. Detailed disassembly and
  symbol/literal checks are retained separately from the limited ISR-root audit.
- COM6 image **e34ab6ea606c5517d379e36f3f7053c7ad0f6406b93cec05338b25fb4f4291ad**
  is uploaded at 921600 and matches full readback. Original app is preserved,
  owned selection is VALID and automatic Bluedroid restore is disabled.
  Existing 37 cold configurations / 173 callbacks pass: tracked peak 56,899 B,
  minimum free after allocation 106,620 B and decoder stack low-water 3,572 / 8,192 B.
  The 112 B lower cold minimum matches the added metadata state.

Redmagic 9125299103B7 uses native YouTube, LHDC V5 / 192 kHz / S24 / stereo,
fixed 900 kbps UI selection and volume150. Each excerpt has an independent
14-second phone pause guard and remains within 15 seconds owned playback.

| Image / window | Initial short frames | Recovered frames | Final unfilled PCM frames |
| --- | ---: | ---: | ---: |
| Fresh0510 music control |1152|768|384|
| Lightweight tracker music |320|320|0|
| Lightweight tracker metronome |1024|256|768|

All 5,368 ACL metadata entries retire with status/lost zero. Both candidate
windows have zero queue/decode/reset/PCM timeout/sequence, ingress, descriptor
and DMA contract errors. Worker/host/snapshot trace losses are zero. Checked
records have valid CRC; one music and two metronome legacy records are explicitly
Unverified. A fresh paused six-second capture contains 195 CRC-Verified records.
The phone ends PAUSED and I2S stopped; live monitor PID 34852 / source 91 is
identity-verified on COM6 at 921600.

## What the finalized metronome gap proves

At 200406730 us, raw PCM and processed PCM are empty. No published/unretired ACL
metadata remains. The decoder has waited 11,724 us for encoded input. The last
VHCI ACL callback is 28,377 us old; that identified packet reached media admission
in 1,946 us. The owner’s last pre-runtime-poll marker is 23,532 us old. That marker precedes
a renderer checkpoint, so it does not establish where the owner spent that time. These are immutable,
coherent gap records, not values inferred from the later stopped snapshot.

This particular event is encoded-input starvation. Its origin could include
credit scheduling, owner scheduling, controller delivery or the peer/radio.
The callback tracker cannot distinguish those causes alone. The next required
observation is the credit and owner state at that same gap. Earlier readiness
measurements have zero held-fence observations; they do not justify changing
buffer ownership or removing the readiness fence based on this event.

The music window's first initial shortage instead has 960 raw frames, an active
DSP helper and an 862 us callback-to-admission interval. Its 320 initially short
frames are all recovered. The two events must not be combined into one cause.
No playback improvement is claimed for this observational change.

After the two candidate windows, stopped internal free/minimum/largest heap is
63,476 /28,592 /59,392 B; selected decoder residency is 60,704 B and decoder stack
low-water 5,904 B. These include this boot's complete history and are not a matched
Bluedroid RAM comparison. LHDC continuity, 1 Mbps, sustained coexistence, acoustic
parity, every feature/security/OTA gate and comparative RAM remain unqualified.
This remains development firmware; main and stable releases are not promoted.

Evidence: native_bt/_build/experiments/light-arrival-1794130641610/ and
native_bt/_build/hardware/runs/1794130940490505000-e34ab6ea606c/traces/.
Reproduction: python native_bt/tools/test_media_arrival.py --out native_bt/_build/arrival_check.
