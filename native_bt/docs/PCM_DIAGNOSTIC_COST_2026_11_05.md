# PCM diagnostic cost and codec labels

## Normal playback

The shared PCM callback previously calculated a rolling FNV-1a hash over every accepted PCM byte, even in the normal stay-native product image. This is diagnostic work, not decoding, DSP, transport validation or audio delivery. At 192000 stereo sample frames per second with four-byte sample containers, it reads and hashes 1536000 bytes per second after the PCM handoff.

Normal `-StayNative` builds now omit this byte loop by default. `-PcmHash` explicitly enables it when a PCM fingerprint is needed. Finite legacy diagnostics retain hashing by default, preserving their existing verification contract. The build script passes the CMake option explicitly on every build so a previous diagnostic cache cannot silently leave it enabled. The local build receipt records the effective `pcm_hash` setting.

PCM ownership, sample precision, decoder/DSP paths, encoded queue size and the audio output bytes are unchanged. Frame/view and byte counters, codec identity, sequence/drop/reset/fault traces, stage timings and queue diagnostics remain enabled. The normal hash field is zero and is explicitly marked `fnv1a32_enabled:false`; it cannot be mistaken for a measured fingerprint. Checksum verifiers reject an explicitly disabled hash and continue to accept historical captures that predate the flag.

## Telemetry identity

The historical `sbc_pcm` event covers every codec. Its existing `codec_name` and `diagnostic_scope:all_codecs` fields identify that scope. The legacy desktop UART monitor still hardcoded "Decoder: SBC" for this event; it now uses the reported name, with numeric-ID fallback for older captures and "unknown" when neither is available.

The historical `frames` field counts PCM callback views, not stereo sample frames or RTP packets. New records state `frames_unit:pcm_views`, and the legacy viewer labels that count as PCM views. This keeps historical readers compatible without treating the count as a codec or wire-format tag.

## Compiler evidence

The full ESP-IDF 6.1 stay-native product build with ccache passes with `pcm_hash:false`, source stability and workspace integrity checks. Compiler-only alternate translation units cover explicit checksum opt-in and the default legacy finite mode. No decoder or peer test program was executed for these checks.

The selected Xtensa compiler emits a 254-byte `pcm` function for the previous image and both checksum-enabled variants, and 220 bytes for normal checksum-disabled firmware. Before/after disassembly confirms removal of the five-instruction per-byte hash-loop body (`add`, byte load, increment, XOR and multiply), along with its setup and hash publication. This is instruction/code-size evidence, not a cycle benchmark or a measured playback speedup. There is no claim that this change alone fixes reported crashes, underruns or maximum-rate headroom.

Python bytecode compilation covers the modified monitor, checksum verifiers and existing diagnostic source extractor; PowerShell AST parsing covers the build script. The extractor was updated to retain the actual compile-time hash selection when it extracts the callback. No unit, simulated peer, playback, pairing or app campaign was invoked. Built-in firmware codec self-checks may still run during ordinary startup and are recorded separately from playback qualification.

Artifacts are retained under `native_bt/_build/experiments/pcm-diagnostics/`, with deployment/startup evidence recorded separately. The entire host goal, maximum LHDC headroom and full feature/security/lifecycle qualification remain incomplete.

## COM6 deployment

Image `42c1722ecad48a9db94b8763fff74865441cf30647a2edbc16b767ff3696d6c5` was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 match; the original application remains unchanged. The owned host booted ready, stayed selected VALID and did not automatically restore Bluedroid. Its retained startup window contains no fatal marker, watchdog or backtrace. Built-in codec startup self-checks passed; no phone playback or other separately invoked functional campaign ran.

The captured PCM records report `fnv1a32_enabled:false`, `fnv1a32:0` and `frames_unit:pcm_views`. UART collection resumed on an identity-verified live COM6 source. The idle heap snapshot remains 154596 bytes free, 130580 minimum free and a 110592-byte largest block; these startup values do not establish a streaming RAM advantage. The deployment archive is `native_bt/_build/hardware/runs/1793905821932849700-42c1722ecad4/`. The legacy GUI label change has syntax validation but has not been exercised in a GUI session.
