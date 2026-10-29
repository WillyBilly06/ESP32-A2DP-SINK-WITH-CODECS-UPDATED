# Opus temporary scratch candidate

This is isolated source and host-test work. It is not attached to the firmware,
target-linked, flashed, or proven by Bluetooth playback. The SDK's original
Opus implementation is used in place; no SDK source is copied here.

## Why

The SDK config selects fixed point and `VAR_ARRAYS`. For 48 kHz, 20 ms,
two coded channels, the CELT decoder's function-scope allocations include
`X[2*960]` (3,840 bytes), six `int[21]` arrays (504 bytes), and
`collapse_masks[42]`. Its nested synthesis call allocates `freq[960]`
(3,840 bytes). Those array payloads total 8,226 bytes before alignment,
function frames, its callees, or its callers. The audio decoder task is
configured for 8,192 bytes. The existing Xtensa target object independently
shows a 352-byte fixed CELT decoder frame and a dynamically allocated
synthesis frequency buffer, but this audit does not identify the particular
packet or instruction that caused the observed overflow. Coded channel count
can differ from negotiated output channel count.

## Contract and integration requirements

Allocate one zero-initialized `dbt_opus_scratch` and its bounded arena only
when Opus becomes the active codec. Choose and record a capacity as part of
the active configuration. Release it after the decoder is quiescent and on
codec switch. Initialization OOM and insufficient capacity must prevent PCM
publication, not continue with an incomplete buffer. The owner must serialize
arena initialization/destruction against use. Configure switch ordering so
old and new large decoder allocations are not live simultaneously unless a
measured budget explicitly allows it.

Compile all Opus translation units that can be reached in this build with the
same compiler force-include of `codecs/opus/scratch/dbt_opus_scratch_override.h`.
The force-include replaces only temporary-allocation macros and suppresses
the SDK `stack_alloc.h` body. Do not apply it to SDK code unrelated to Opus.
Opus common code is shared by encoder and decoder; therefore **every entry**
to any reachable Opus operation that may allocate temporary scratch must be
inside `dbt_opus_scratch_run`, including any encoder or other profile use.
An unguarded allocation or one on the wrong task aborts as an integration
error; it cannot silently use another task's memory. A process-wide atomic
owner prevents concurrent or reentrant guarded operations and returns BUSY;
no spinning, waiting or per-frame heap allocation occurs. A task-local
pointer validates the caller, and small save/restore marks handle nesting.
The target build must verify C11 TLS and atomics in its actual FreeRTOS setup.

`dbt_opus_scratch_run` only reports its callback result when it returns OK.
On arena exhaustion or invalid mark it unwinds with `longjmp`, clears the
arena, and invokes the required reset callback while retaining exclusive
ownership. The callback must discard/clear all partially produced PCM and
reset or invalidate the decoder, and must not allocate Opus scratch itself.
If reset fails the caller must destroy/retire the decoder; never reuse its
partially modified state. Treat reset as a discontinuity and never retry PLC
against the dirty state. Keep the entire guarded callback C-only and audit
that it does not hold locks, own foreign resources requiring cleanup, or
cross C++ frames before allowing a nonlocal exit. Do not hold host/controller
locks across the call. A caller can itself fail after `work` returns; handle
normal negative Opus results independently. This API does not prevent callers
from publishing PCM too early; only publish after successful guarded return.

The SDK's `NONTHREADSAFE_PSEUDOSTACK` is not a safe substitute: it is an
unchecked shared global arena with no capacity failure recovery. This adapter
does not fix fixed-size local arrays or all ordinary call frames; the target
stack still needs measurement with decode, PLC, transitions, malformed input,
and the rest of the A2DP worker. No source-proven maximum arena capacity has
been established. Fallback on exhaustion is mandatory even if a chosen size
has passed a corpus. Do not claim a finite test's peak as a universal bound.

## Evidence

`python native_bt/tools/test_opus_scratch.py` builds the SDK Opus source twice
with fixed-point settings: once with this macro override, once with original
VLA macros. It compares every PCM sample and result line for 128 decode/PLC
cases including 2.5/5/10/20/40 ms, one/two coded and output channels, and
10/20/40 ms SILK/hybrid/CELT mode changes. It tests deliberate allocation
failure, capacity exhaustion at 32 and 5,000 bytes, invalid allocation size,
reset failure, nested ownership rejection, and compares subsequent decoding
after reset against a fresh decoder. It performs workspace checks before and after. The Windows Clang host
run passed; this does not establish DSP quality, target memory capacity,
thread scheduling, timing, or RF behavior. These are generated encoder
inputs, not an exhaustive malformed-packet corpus.

An earlier run recorded a high-water mark of 8,236 arena bytes for 20 ms
stereo CELT and 9,196 on one hybrid-to-CELT transition; these values are
observations for that host configuration, not selected capacity. A separate
Xtensa compile of the adapter object succeeded and showed only `abort`,
`setjmp`, and `longjmp` as unresolved library symbols, with a 4-byte TLS
symbol. This is compile-only, not ESP-IDF linking or runtime validation.

The test driver supports local Linux ASan/UBSan via
`DBT_OPUS_SCRATCH_ASAN=1`; sanitizer results have not been obtained here.
The attempted Windows ASan build failed to link because the sanitizer runtime
and Windows CRT both defined `malloc` and `free`; it did not run tests.
Do not use sanitizer failure as target evidence without examining whether an
upstream fixed-point operation, integration code, or the adapter caused it.
