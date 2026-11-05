# Exact FIR interpolation on Xtensa LX6

This change preserves the existing Q30 coefficient tables, 96/48/24 tap counts, phase resolution, sample formats, rates and filter history. It adds no allocation or state. It applies only under the generated adjacent-row difference bound; the general wide-difference implementation remains available.

## Arithmetic equivalence

Let D = next - first, 0 <= R < 2^25. The generated absolute adjacent-row bound is 10,373,296, below INT32_MAX. Therefore D, abs(D), and negation of D are representable in int32_t. The unsigned product abs(D) * R is below 2^56 and fits uint64_t. Its quotient Q = floor(abs(D) * R / 2^25) is no greater than abs(D), so Q and -Q are representable.

C signed division truncates toward zero: trunc(D * R / 2^25) = sign(D) * Q, including D = 0 and R = 0. Adding this step to first stays between first and next, so the final signed addition cannot overflow. The new expression therefore returns the same integer for every caller-valid input; no coefficient bit or rounding behavior is discarded.

## Compiler evidence

Compared the complete resampler object with the active ESP32 GCC 15.2 compile command (-O3), not an unrelated desktop approximation. The old per-coefficient signed division uses high-word sign extraction, a 25-bit bias, a two-word addition with conditional carry, and a funnel shift. The replacement uses ABS, unsigned wide multiply, funnel shift, NEG and MOVLTZ. It removes the conditional carry branch from interpolation. The arithmetic sequence after the adjacent-row subtraction is 8 instructions instead of 10 or 11 depending on carry. Scheduling and register allocation elsewhere can also change; this is not a measured cycle reduction.

The filter text section in the comparison objects decreases from 991 to 957 bytes. Both objects have zero data/bss and unchanged coefficient table sizes. Comparison disassemblies are retained locally under native_bt/_build/experiments/fir-word-interpolation/.

## Validation boundary

Full firmware compilation and deployment integrity are recorded separately. No playback, app, encoder/decoder regression or timing campaign is run in this pass, following the user's instruction to stop tests. This change is not proof of real-time headroom at 192 kHz or stable playback. Previous measured DSP/resampler pressure remains an open qualification item.
