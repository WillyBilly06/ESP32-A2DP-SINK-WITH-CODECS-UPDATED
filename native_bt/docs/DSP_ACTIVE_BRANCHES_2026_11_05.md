# Process only audible DSP branches

## Source of unnecessary work

The prior integer DSP always ran both full-range and split-channel branches. After common EQ and volume compensation, it computed LP/HP crossover, split LP boost, and full-range left/right boost even when the bypass mix gave one branch exactly zero weight. The working Bluedroid DSP selects the needed branch instead.

The owned scalar and block APIs now use the same selection:

| Bypass state | Channel filter paths including common EQ/compensation |
|---|---:|
| Prior implementation, every state | 13 |
| Stable split output | 11 |
| Stable full-range output | 10 |
| Crossfade between branches | 13 |

These are structural path counts, not measured timing percentages. Identity filters already had their existing fast path. The most relevant avoided work is nonidentity boost filtering on inaudible full-range channels, and crossover/LP boost when full-range bypass is stable.

Common EQ, volume compensation, analysis, volume, channel flip, makeup gain, clipping, coefficient precision, saturation and every feature remain available. Decoder framing, codec configurations, resampler coefficients and 1800-byte/eight-slot input storage are unchanged. CPU0 remains the codec worker; Bluetooth/DSP/background placement from the preceding core-isolation change is retained.

## State and transitions

At a stable bypass endpoint, the fully inaudible branch pauses. Controls clear only an incoming branch that was previously fully inactive. The currently audible branch and common EQ histories are preserved. A reversed crossfade keeps both running histories. The incoming branch contributes through the existing 20 ms crossfade rather than replacing the audible path abruptly. Coefficient ramps remain unchanged.

This intentionally changes incoming-branch state from the old always-warm policy. It is not claimed bit-identical to the prior implementation across bypass toggles. Acoustic, rapid-update and full lifecycle qualification remain outstanding.

Both APIs decide branch participation before advancing the bypass ramp, so the final transition sample still advances both branches. A stable-coefficient block is split at the exact fade endpoint into at most two positive-length segments. This keeps scalar/block inactive-history retirement aligned without forcing an otherwise stable bypass fade through the slower coefficient-ramping kernel. There are no additional ramp samples or fixed delays.

The existing fixed block scratch is reused; a split-only block filters the common PCM view directly and avoids the unused full-range copy. No heap allocation, task, queue or persistent data field is added.

## Evidence and limits

Strict Clang C11 compilation passes for scalar, word-block and forced split-sum configurations. Clang static analysis of the active word-block mode reports no diagnostics. Full ESP-IDF 6.1 ccache compilation, source stability and workspace checks pass. No function, unit, playback, pairing or app campaign was executed during the user's test hold. Built-in firmware startup codec self-checks are recorded separately on deployment.

Target object inspection reports scalar IRAM text increasing from 683 to 770 bytes (+87), unchanged 84-byte IRAM literals, and zero data/BSS in both objects. Existing state layouts are unchanged. Thus this is a small IRAM code cost for avoiding inactive computations, not a zero-total-SRAM claim. Compiler artifacts and the exact source/build receipt are retained under native_bt/_build/experiments/dsp-active-branches/.

No elapsed-time speedup, acoustic result, zero-underrun playback or maximum-LHDC headroom is established by compilation. FIR/render cost and decoder PCM backpressure remain active investigation items. The full stack goal is incomplete.

## COM6 deployment

Image 444c690ecdccfddcad72271f6458d106ec1e0cb3d5b5c1bc9a6d354f5b1a1579 was flashed at 921600 baud to ota1 at 0x390000. Full image and selector readbacks at 230400 matched; the original application remained unchanged. The owned host booted ready, stayed VALID and retained decoder CPU0 priority21, host CPU1 priority22 and renderer CPU1 priority20. The startup capture contains no watchdog/backtrace. Built-in codec startup self-checks passed; they do not test the new DSP transitions.

Live UART resumed on verified source 55. Idle internal heap is 154536 bytes free, 130508 minimum, and 110592 largest block. These idle figures do not establish streaming headroom. Archive: native_bt/_build/hardware/runs/1793907958524580100-444c690ecdcc/. No separately invoked functional campaign ran.
