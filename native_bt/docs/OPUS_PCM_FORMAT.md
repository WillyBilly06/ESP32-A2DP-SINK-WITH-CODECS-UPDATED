# Baseline Android Opus PCM format and decoder stack

## Trigger and change

The user reported a live A2DP_DECODER stack-overflow panic and identified the
preceding codec as vendor `0x000000e0`, codec `0x0001`, 48 kHz stereo; the
baseline logged the generic vendor fallback as 32-bit PCM. That is the user's
observation, not an independently reproduced test in this work.

The SDK's Opus constants identify `e0/0001` as Android Opus and `05f1/1005`
as Opus 05. Its selected Opus wrapper uses `opus_decode()` into `opus_int16`
and calls the PCM callback with the decoded sample count times channels times
two bytes. The old application recognized only Opus 05, so Android Opus took
the unknown-vendor path and reported 32-bit samples. The stereo ring producer
therefore used eight bytes per frame instead of four for this 16-bit stereo
PCM; it did not interpret the callback's interleaved samples in their actual
format. This mismatch alone does not establish the cause of the stack panic.

The application wrapper now identifies both variants and forwards 48000 Hz,
16-bit PCM and the decoded mono/stereo channel count. Android's channel mode is
in the low three bits of the combined CIE byte; Opus 05 uses channel and
coupled-stream counts in separate bytes. Recognized but unsupported Opus CIEs
are not guessed: after marking the format invalid, the wrapper suppresses newly
entered PCM callbacks until a subsequent valid codec configuration is forwarded.
This atomic gate is not a quiescence barrier for a callback already in flight. The existing three-argument
codec callback remains unchanged. The SDK's media codec type remains generic
`ESP_A2D_MCT_NON_A2DP`; `get_opus_variant()` exposes the variant without changing
that signature. Logs identify the variant. Unknown non-Opus vendor behavior is
unchanged.

The helper reads the SDK event's copied CIE, not the three-byte AVDTP header.
`btc_av.c` zeroes and copies the vendor-specific bytes after that header, and
`esp_a2d_mcc_t` provides a 26-byte Opus member. The checks match the current
SDK wrapper's supported Android channel/mode/rate/frame combinations and its
Opus 05 channel/coupling and frame-duration checks. No extra codec allocation,
NVS change or codec switch behavior is introduced.

## Stack evidence and limits

The previously active ELF checked in this audit has SHA-256
`65677514ed93a761ed843b7a0e4ae34ad456a825d34eadff467a1ee729a2446b`.
That ELF predates this source change. The current SDK source defines the
`A2DP_DECODER` stack as 8192 bytes and the current application source creates
`audio_render` separately with 5120 bytes. The PCM callback only enqueues to the
ring on the decoder task; integer DSP and I2S output run on `audio_render`.
The callback executes *after* `opus_decode()` returns, so its frames must not
be added to the peak inside nested Opus decoder calls.

Read-only inspection of the supplied ELF's Xtensa `entry a1` prologues gives
fixed entry allocations, not dynamic peak stack measurements: `onStreamData`
32 bytes, `AudioPipeline::enqueueFromBt` 48 bytes, the native trampoline and
`data_cb` 32 bytes each, `opus_decode_frame` 224 bytes,
`celt_decode_with_ec_dred` 352 bytes, `celt_synthesis` 112 bytes,
`silk_Decode` 784 bytes, `celt_decode_lost` 2416 bytes, and the linked LDAC
`proc_imdct_core_ldac` 1120 bytes. Some ELF sections did not disassemble
normally with the local objdump; these sizes were extracted from the Xtensa
entry instruction bytes and are not whole-call-chain stack maxima. No panic PC
or stack high-water measurement was collected here.

The compiled Opus source configuration enables `FIXED_POINT` and `VAR_ARRAYS`;
`ALLOC` therefore declares variable-sized automatic arrays. For the conditional
48 kHz, 20 ms CELT case with two coded channels, `N=960`: the parent CELT
function retains `X[2*N]` of 16-bit elements (3840 bytes) while calling
`celt_synthesis`, which allocates `freq[N]` of 32-bit elements (3840 bytes).
Those arrays alone total 7680 bytes while nested. Fixed entry frames, other
variable arrays, ancestors and deeper calls add to this. This is a strong
source-backed overflow mechanism for that codec/mode and 8192-byte stack, not a
measured peak or proof of the exact reported failure. Packet-loss concealment
and mode transitions can also invoke bounded nested `opus_decode_frame` calls;
those are not general unbounded recursion. The exact peak depends on the
encoded mode, channel count, error path and compiler.

The Opus PCM output buffer is heap-owned and sized for the negotiated frame;
fragment accumulation is separately lazy and bounded to 6144 bytes. The audio
ring is heap-owned (configured 16384 bytes, 2048 stereo frames with one slot
reserved); the configured 256-frame DSP output has two spare frames and is also
heap-owned. These are not decoder-task automatic arrays. Moving the real decoder
scratch off its task stack requires a bounded, aligned, reentrancy-safe workspace
with correct nested lifetimes and allocation-failure/codec-switch cleanup. A
single unprotected global pseudo-stack is not a safe substitute. Increasing the
task stack alone is a RAM tradeoff and needs measured stack headroom, not a proof
of correctness. Immediate NVS commits for EQ changes remain required.

## Offline verification

`python native_bt/tools/test_opus_pcm_format.py` exercises the production inline
CIE-to-PCM helper with Android and Opus 05 mono/stereo vectors, supported frame
durations, invalid and truncated fields, unchanged output for unknown vendors,
16-bit sample stride and all 256 Android configuration bytes. The Windows host
run passed six test groups with source hashes in
`native_bt/_build/opus_pcm/results.json`. The full ESP32 application wrapper also
passed a syntax-only compile using its existing SDK command; pre-existing
unused-variable and deprecated-API warnings remain. These tests do not execute
the full SDK event dispatcher, compile/link a new firmware image, decode Opus,
measure stack, establish Bluetooth or verify audio.
