# LHDC stereo S32 emission specialization — 2026-11-07

The complete-frame profile measured about190us for scale/safety scan and380us for output/tail at192k/S24/fixed900. The original safety scan, quality callback, concealment decision and overlap reset remain unchanged. Only nonconcealed stereo32-bit-container emission selects a compact helper in IRAM. Mono, S16 and legacy generic output retain the original loop; concealed frames retain the original zero-output path.24-bit source remains emitted in its existing S32 container.

The helper preserves scale multiplication and truncation. It replaces the redundant float upper clamp with an explicit2^31 endpoint check; values below-2^31 return INT32_MIN, unordered values return0, and positive overflow returns INT32_MAX. Every cast is in range. No gain, precision, rate, bitrate, table, buffer, task, affinity or ownership changes. The destination is the existing unpublished PCM storage; no additional copy is made.

Separate-TU host comparison covers all4294967296 binary32 conversion patterns. A second run uses ASan/UBSan/float-cast-overflow checks on1M patterns. Each run compares12000 actual extracted original/candidate frame-output blocks: PCM bytes and canaries, quality peak/clipping/conceal/nonfinite events, overlap data, mono/stereo, S16/S32, valid widths/scales and NaN/Inf/rails.48 full independent PCM encoder cases and80 real-codec deferred cases pass. Initial disk-quota rejection is preserved; hashed obsolete AMD64 host COFF objects were removed, retaining linked binaries, source, receipts and all physical evidence. Rebuild and checks pass.

Full ccache build/source/admission pass for173206502d7a72c9f879b80a715046b998d98315de3fea80535aded262d3fa64 (1340688bytes). Relative to8ecd measurement baseline: instruction RAM+140 bytes (helper code132 plus literals), data/BSS/rodata0, flash text+92. Linked object shows native LSI/MUL.S/OLT.S/TRUNC.S and word stores. No function calls occur in the emitted helper. At document creation the candidate is not flashed and no speed gain is claimed.

Experiment: native_bt/_build/experiments/lhdc-s32-output-1794065008328.

    python native_bt/tools/test_lhdc_s32_output.py --decoder "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main/decoder" --baseline <saved_prechange_generated_decoder>

The current profiling image baseline is8ecd27c50ec8: frame prepare11.02us, channel envelope1808.23us, scale/safety scan189.58us, output/tail379.61us; whole codec2403.72us per actually decoded unit. Instrumentation/preemption/wrapper costs and publication cadence apply. This is not a1Mbps or all-feature qualification.

## Verified target result

173206502d7a is flashed onCOM6 at921600; complete written-image readback at115200 matches. Original application preserved, owned VALID, no automatic Bluedroid return.37 cold cases pass (173 callbacks, tracked peak56899, minimum free after allocation102996 versus103028 baseline, cold stack low3456 versus3536). The32-byte heap difference is observed and not attributed to a new persistent allocation; data/BSS are unchanged. No task stack allocation changes.

Two native YouTube metronome runs explicitly negotiate192k/S24/fixed900 (566media bytes/unit,899200codedbps) and confirm phone150/150. Decoder means2167.96 and2168.65us/unit versus2403.72 baseline: about9.8% lower elapsed time for this workload. Output/tail means129.94 and129.72us versus379.61: about65.8% lower. Channel processing remains approximately1.81ms and the original safety scan remains approximately0.20ms. NativeMUL.S/TRUNC.S and boundedS32 stores are linked in a132-byte IRAM routine plus8 literal bytes. Host all-value conversion and complete-frame state tests preserve PCM/quality/overlap.

No decoder rejects/nonfinite/concealed frames occur in these traces. Finite sample clipping persists, and refill unfilled totals15104/13056 (startup/drain included) mean continuity is not fixed. The first shortage in one run has1280 decoded input frames, zero processed output, decoder waiting for encoded input, helper stateIDLE, main renderer busy. The repeat has1920 decoded input frames, zero processed output, decoder waiting for PCM space with5 encoded packets, helper stateRUNNING. These are distinct scheduling states, not proof that all losses are input starvation.

The current assistance predicate admits LHDC only when decoder_wait_kind==0 (PCM space), excluding encoded-input wait==3 even if decoded audio is available. A future change must retain all raw/output preparation, single-owner, generation and cancellation gates. Fresh evidence justifies investigating that opportunity; it does not establish that broadening the gate alone fixes the running-helper case.

The repeat automatic volume restoration unexpectedly read60 instead of saved90. A bounded verified-key correction read70,80,90 and then stable90. Shell stream-adjust commands did not change readback. Future harnesses must restore by per-key readback rather than assuming fixed steps. The recorder variable-name failure before that repeat is preserved and involved no phone commands. Playback is paused, volume90/150 restored, logger live.
