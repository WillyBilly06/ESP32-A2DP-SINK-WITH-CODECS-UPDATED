# AAC parser progress and memory bounds

The active AAC wrapper and LATM transport were reread through setup, raw/PCM capacities, fragmentation, decode, reset and close. No new wrapper-capacity cause of the reported crash was established. The four SDK units below are byte-identical after newline normalization to the released Bluedroid baseline. The defects are unsafe backend handling of malformed/truncated input; they are not proven as the cause of the reported Redmagic crash or the common low-bitrate symptom.

## Confirmed unsafe paths

- noiseless.c DecodeSectionData accepts section length zero, leaving sfb unchanged. Exhausted zero-filled reads can repeat forever. An oversized run writes past the declared band count before an assertion.
- Parsed maxSFB can exceed the selected sample-rate table. Its consumers use sfbTab[sfb+1] and sfbTab[sfb], so bounds must be checked before spectral work. Common-window channel-pair masks need the same gate.
- Long TNS orders are read from five bits, while the working order limit is20 and total coefficient storage60. Pulse start-band and cumulative positions were used before bounds assertions.
- The original bit reader can carry a negative cached count after exhaustion; later shifts must not use that state. Spectral escape lengths can exceed a positive int shift range.

## Guarded port

The generator pins canonical aacdec/noiseless/decelmnt/huffman source hashes and refuses an unexpected source filename or changed input. Original SDK files and license notices are preserved. Product and existing compiler-harness routes now select all four generated units; the owned open adapter requires four separate link-proof symbols.

Each accepted section has a positive length fitting the remaining bands before any write. Group/band capacities and selected tables are checked in both individual and common-window paths. Pulse positions fit the1024 coefficients, TNS order/aggregate storage are bounded, and the optional final mask zero cannot write past its array. Rejections propagate through DecodeICS/DecodeNoiselessData/DecodeNextElement before spectral transform or invalid cursor publication.

Guarded reads/advances preserve a sticky invalid state and avoid calling SDK shifts with an invalid cache. Huffman peeks retain the original zero-padded lookahead so a short valid codeword near the end is not rejected merely because the maximum lookahead is longer. Actual bit consumption is checked; spectral escape width is bounded to the positive-int representation before shifting. This is a machine-safety bound, not a newly advertised codec mode.

Valid-path coefficient arithmetic, codec rates/channels, PCM conversion, workspace sizes and transport limits are retained. This is selected parser hardening, not a complete Helix security/algorithm audit. Malformed/fault behavior still needs separate qualification under the held campaigns.

## Verification and device scope

Strict Xtensa -O2/-fwrapv compilation of original and guarded units passes with warnings as errors. Own frames remain DecodeNoiselessData128, DecodeNextElement64, spectral long/short64 and escape64 bytes; this is not full-chain stack measurement. Clang analysis is empty for the wrapper, guarded noiseless and elements. Guarded Huffman retains three original final unused bitBuf-shift dead-store warnings. Existing harness routes were updated and syntax-compiled, not executed.

Full ccache build passes with10349 stable source hashes; rebuildable storage460.667 MiB. Linked text887579 is4072 larger than commit60aa3b49; BSS50412, IRAM122255 and DRAM data23893 are unchanged. No runtime pool/task is added.

COM6 image 975e2c84582bfc43ec85e761c414867008453945351f1339d5d3a93250626da0 was uploaded at921600 and image/selector readbacks at230400 match. Original application full-region MD5 is unchanged. Owned ota1 remains VALID/kept_active with automatic Bluedroid restoration disabled. Ordinary startup is ready, with no watchdog/backtrace. The37 existing cold codec checks pass; all four AAC44.1/48k mono/stereo cases retain zero reference error and unchanged tracked peaks26116/26884. Idle heap remains137912 free/67356 regional minimum/110592 largest. UART source131 is connected. Archive: native_bt/_build/hardware/runs/1793952476665653500-975e2c84582b/.

No separate malformed-input, AAC crash, codec switch, pairing, RF, BLE/app, sustained playback, acoustic or performance campaign was run. These cold fixtures do not prove the reported live crash is gone or all-codec throughput is sufficient.

## Shared-path trace

The compiled product uses copied PCM mode2, not asynchronous PCM loans. It copies into the input ring with a nonblocking try-write; the suspected per-chunk ticket/DSP serialization is not present in this configuration. The common adaptive FIR/render path still needs current sustained timing evidence. The older folded-zero and paired-assembly FIR candidates were rejected as slower and remain disabled. SDK i2s_channel_tune_rate takes portMAX_DELAY and can disable/reconfigure APLL; it is not introduced as a real-time shortcut. No new common low-bitrate root cause is claimed here.
