# Optional input loan: memory-layer checkpoint

The renderer currently allocates a second input PCM ring after the codec has
already retained its decoded buffer. At 192 kHz/stereo/S32 that ring alone is
65536 bytes. This checkpoint adds an optional render-owner input loan without
changing the product's current copied-input path.

`dbt_audio_open_borrowed` allocates output pages only. The existing config's
input frame count bounds one immutable view. `dbt_audio_input_borrow` accepts
one view or returns BUSY without accepting another. The render owner publishes
that view's logical endpoint; counters remain monotonic across uint32 wrap.
The view must remain live until `dbt_audio_input_release` succeeds, explicit
input cancellation returns, or the quiescent render owner closes the object.
These calls alone do not acknowledge a cross-task decoder loan.

A fractional step can need two source samples while only one remains in a
block. Waiting for that sample's owner to recycle the block would deadlock.
Release therefore retains at most one complete raw stereo frame (eight bytes)
in owned carry storage, then clears the external pointer. The next view extends
the logical input after that carry. Format conversion, all FIR taps, phase,
history, integer DSP and output precision are retained. No input-frame copying
occurs except the one boundary frame; no allocation occurs during borrow,
release, cancel or render. Input capacity is one bounded view plus one carry.

Cancellation explicitly discards unrendered input and returns that count. It is
an abort operation, not a successful playback outcome. Close never releases the
external PCM pointer through the audio allocator. The existing copied-input API
rejects use on a borrowed-input object, and vice versa. The existing linear
path's manually configured fractional EOF retains its last lookahead sample;
the new path matches that state instead of claiming a new drain guarantee.

Host ASan/UBSan compares 108 complete copied/borrowed streams across
48/96/192 kHz, mono/stereo, 16/packed24/S32 inputs, both adaptive and linear
paths, +/-500 ppm, one-frame/256-frame/random view sizes, alignment, counter
wrap and interrupted EQ changes. Every output sample and full DSP/resampler
state is checked. Released source views are poisoned before further rendering
to detect stale reads, including the retained boundary sample. Allocation
failures, partial-page cleanup, cancellation and ordinary/borrowed reopen also
pass. All 180 existing portable test groups pass.

```powershell
python native_bt/tools/test_audio_borrowed.py
python native_bt/tools/test_host.py
```

The optional worker hook now has explicit PCM begin/poll/cancel operations.
PENDING plus a nonzero ticket accepts a view; BUSY accepts nothing. Only an
OK/CANCELLED completion acknowledges retirement. A stale/invalid response
closes admission and reports a fault while retaining the decoder and pinned
packet. Timeout similarly requests cancellation without freeing a live loan.
One accepted-loan receipt is serviced before control jobs, so reset, close,
abort, suspend, drop and reconfiguration cannot overtake renderer ownership.
The worker sleeps on notifications/deadlines while completion is pending;
requesting cancellation is not an acknowledgement and does not trigger a spin.

Ten worker groups pass ASan/UBSan, including accepted-output failure, a busy
begin, stale acknowledgement, wrap-safe timeout and each codec lifecycle fence.
All 184 portable groups pass. Eighty existing real-codec deferred-copy cases
retain every PCM byte and timestamp across 37 configurations and the maximum
LHDC fixture, with codec ASan and owned-adapter/worker UBSan coverage. This is a
regression check of the existing codec path, not real-codec async-loan coverage.
All three test receipts were rechecked against current source hashes. The
ccache full-product build/source/image/layout admission passes.

```powershell
python native_bt/tools/test_codec_pcm_loan.py
python native_bt/tools/test_probe_codec_check.py --deferred --sanitize --lhdc-source '<LHDC checkout>'
```

The [source-bound foundation results](PCM_LOAN_FOUNDATIONS_RESULTS_2026_11_04.json)
keep these scopes separate. Renderer/provider integration remains open and
neither loan mode nor the PCM provider hook is selected by the current product.
Sound overlays, stop/drain/restart/fault transitions, deadlines and teardown
require integration tests before selection. No product SRAM saving,
decoder-to-render speedup or hardware playback pass is claimed by this checkpoint.
