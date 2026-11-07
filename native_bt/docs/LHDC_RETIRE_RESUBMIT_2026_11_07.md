# Immediate DSP resubmission experiment — rejected

The immediate-resubmission prototype is removed. COM6 is restored to exact
image `ec89367eeef0acaaba286802a53d7aaced2e5b3b9991fa6d49fa9152f219c79f`,
with the S32 decoder improvement and idle-input DSP admission retained.
LHDC continuity remains unqualified.

## Hypothesis and ownership test

The existing renderer retires a completed Core0 DSP job, services controls and
refill, then processes one local128-frame block before lending again. Earlier
captures showed millisecond-scale local spans while decoded input remained
available. The prototype removed only that unconditional local-block interval:
an eligible idle helper could receive another existing bounded lease immediately.
If unavailable, local progress before the fair yield remained required.

The actual two-core output/DSP/sound fixture was extended with Core1 suspended
immediately after the second dispatch. Core0 had to process512 then256 frames
without an owner acknowledgement; a queued stop retained all borrowed storage
until retirement. The corrected negative control fails against the old path's
helper/frame accounting, and the prototype passes ASan/UBSan with the selected
six-block and default four-block caps. The unavailable-helper fallback still
publishes640 frames before its first fair yield. Existing cancellation, early
publication, producer-preemption and bounded-cap checks also pass.

An initial fixture assertion incorrectly required a zero decoder media budget
after DONE. The production contract intentionally allows decoding to resume at
DONE; that assertion was corrected before evaluating the intended negative
control. Both failed receipts are retained. This change does not relax DSP/PCM
lifetime or cancellation checks.

No DSP math, queue geometry, priority, affinity, security or application features
changed. Prototype image `54ea970d475c90e5a7125de27b48c097ed23ec1bb35d194137449165443d0c16`
passes ccache build, source/image admission, guarded921600 upload/full115200
readback and all37 cold codec cases. It removes20 bytes of flash text and adds
no data/BSS or IRAM. These checks do not prove real-time performance.

## Physical comparison

Redmagic native YouTube120 BPM metronome, LHDC192kHz/S24/stereo, fixed900
setting, phone volume90/150 and EQ11/8/12 throughout. The sampled payload
configuration remains the existing566-byte/one960-sample-unit path. Each trace
has an11-second command interval and an independent14-second pause guard.

| Run | DMA source frames | Unfilled frames | Unfilled per million source frames |
| --- | ---: | ---: | ---: |
| Baseline ec89367e | 2738880 | 9984 | 3645 |
| Immediate resubmission | 2724736 | 15232 | 5590 |
| Immediate resubmission repeat | 2733120 | 14464 | 5292 |
| Restored exact ec89367e | 2708160 | 18048 | 6664 |

The prototype does not establish a repeatable benefit. The reverse baseline
also exceeds both prototype runs, demonstrating substantial variation; the
data do **not** prove that immediate resubmission itself causes worse timing.
Counters include startup and drain. Fresh first-starvation records still show
1920–2048 raw frames with zero processed output, and local render spans remain
3.4–3.9ms. No queue-full, decode-reject, descriptor or refill-contract errors
were observed in these traces.

Helper share changes from53.56% to56.04/56.42%; helper elapsed math cost stays
approximately714 cycles per PCM frame. Increasing use of Core0 alone does not
demonstrate improved continuity. The prototype and its modified host fixture
are therefore removed from production source rather than retained on the
strength of a passing ownership model.

## Restoration and evidence

Source and the existing test are restored byte-for-byte. The rebuilt image is
also byte-identical to the prior ec89367e image. Guarded written-region
readbacks match, owned boot remains VALID, the original application is
preserved, and there is no automatic Bluedroid fallback. All37 restored cold
cases pass:173 callbacks, tracked peak56899 bytes, minimum free after allocation
102996 bytes and stack low-water3584/8192 bytes. The existing restored
six-block output/DSP/sound lifecycle fixture also passes ASan/UBSan again.

The local experiment directory is
`native_bt/_build/experiments/lhdc-retire-assist-1794069715082/`.
It retains baseline/prototype source, both negative controls, positive test
receipts, raw captures, all build/deploy results and the reverse comparison.
The failed postbuild storage gate is also retained. Hash-identified completed
AMD64 host compiler objects were reclaimed; source, executables/results,
target objects and all hardware evidence remain preserved.

The first new trace reused the previous JavaScript closure's old capture path.
The earlier full record was still retained in memory: its source12/2623680-frame
capture was restored, and the new source14 trace plus mixed append were saved
separately before restoration. The replacement helper accepts its directory
explicitly and rejects existing output paths before any phone action.

Restored physical archive:
`native_bt/_build/hardware/runs/1794070649262749600-ec89367eeef0/`.
[Compact results](LHDC_RETIRE_RESUBMIT_RESULTS_2026_11_07.json) preserve the
comparison and restoration identifiers. Phone playback is paused at90/150;
UART logging is listening.

## Next measured change

The actual decoder separately scans scaled PCM for quality and then traverses
it again for stereo S32 emission. Prior measurements put these scopes near200
and130us per unit. Investigate sharing that work, requiring unchanged peak and
clipping counts, NaN/concealment behavior, overlap clearing and complete PCM.
No such decoder modification or speed gain is claimed here. Full1Mbps LHDC,
fixed990 LDAC, coexistence, feature/security/OTA/lifecycle/stress and acoustic
qualification remain open.
