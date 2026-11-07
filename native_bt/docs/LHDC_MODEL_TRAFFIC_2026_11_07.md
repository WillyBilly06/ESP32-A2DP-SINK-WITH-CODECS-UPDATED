# LHDC adaptive-model write cancellation — 2026-11-07

The largest stage in the existing instrumented Xtensa fixture profile is entropy decoding. The FAC adaptive model slides a history window per symbol. When its outgoing and incoming symbols match and the frequency is at least2, the old decrement/increment pair is an identity and the history byte is already correct. The owned source-pinned port now advances the same cursor and avoids those three redundant stores.

The original threshold/rescale check still executes on the cancelled path. Frequencies0/1 retain the original increment behavior. Matching symbols do not justify skipping rescaling: low frequencies and cumulative frequencies not yet refreshed can still require it. No probability table, decoded symbol, Rice/mantissa rule, framing, rate, bit depth, gain, format, allocation, queue, ownership or CPU affinity changes. The sibling source remains unchanged.

## Evidence before deployment

Experiment: native_bt/_build/experiments/lhdc-model-traffic-1794060102541.

-10,400,000 symbol-by-symbol comparisons between the actual original and generated model functions under ASan/UBSan. Every model field, full history array/canaries and rescale-call count match.13 window lengths,8 deterministic/random/long-run patterns and2 frequency initializations include wrap and deliberately forced threshold equality.8,085,456 cancelled transitions include1,652,545 that still rescale. These are artificial test counts, not a measured occurrence rate in music.
-48 known-input encoder cases match every PCM byte from the independently normalized original decoder, including four rates,16/24-bit PCM,10%/95% peaks,400/1000kbps and forced bitrate transitions;4608 codec units total.
-80 real-codec deferred regression cases pass, including all37 existing configurations and the maximum LHDC fixture. Its final workspace quota check initially failed after compiler symbols grew; the functional pass is preserved, only rebuildable host PDBs were removed with hash receipts, and a fresh workspace check passes. No physical evidence was removed.
-Full ccache firmware build, source-content checks and image admission pass. Candidate9316b00a1045e964af17892dd28541368e4fad0ae1c680c9e061822e3eb1caad is1,338,784 bytes. Compared with389514bc, instruction RAM increases224 bytes; data/BSS/flash text/rodata deltas are zero. There is no new persistent data allocation, but the instruction-RAM cost is real.

The existing37 target cold cases are unchanged; no boot test cases were added. Target speed, live memory/stack behavior, LDAC regression and overall continuity are unproved at document creation.

Two available pre-change live389 runs at actual192k/S24/fixed900/max phone volume record2435.91 and2425.59 elapsed decoder microseconds per actually decoded unit. These use quiet before/after endpoint differences in execution time and LHDC attempt counts, include preemption/wrapper costs, and do not represent exclusive arithmetic cycles. The dedicated second baseline has2837 decoded units,6,881,397 elapsed microseconds and7424 refill-unfilled frames including startup/drain. Native YouTube metronome, phone volume restored90/150.

Reproduce:

    python native_bt/tools/test_lhdc_model_traffic.py --decoder "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main/decoder"
    python native_bt/tools/test_lhdcv5_pcm_levels.py --encoder D:/owned61-lhdc-encoder --decoder "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main/decoder" --out native_bt/_build/experiments/<run>
    python native_bt/tools/test_probe_codec_check.py --lhdc-source "D:/ESP32 Bluetooth/LHDC-V5-Decoder-main" --deferred --sanitize --lhdc-esp32-paths --lhdc-max-fixture
