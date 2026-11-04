# Opus exact PCM oracle

The pre-controller codec check uses the same encoded Opus packets for both
oracles and still requires an exact full-PCM hash. The SDK chooses
`OPUS_FAST_INT64` in `celt/arch.h` from compiler architecture macros. Its
32-bit branch uses a different fixed-point Q31 multiplication expression, so
the decoded PCM can differ by small integer amounts from its 64-bit branch.
The fixture header selects the corresponding exact hash using the same macro
rule. The standalone Opus component rejects configuration if the SDK rule no
longer matches that rule. The test-only slow-profile override is forbidden in
ESP32 firmware.

Run `python native_bt/tools/test_opus_char_variants.py` to rebuild guarded,
fixed-point host variants and compare all 16 configurations. It verifies the
selected profile, signed and unsigned `char`, hardening on/off, and each wrong
profile's failure. It also checks that replacing only the slow Q31 expression
in a temporary host build restores the fast hashes. No SDK source is modified;
derived test headers and results remain under `_build/native_codecs`.
`generate_opus_profile_oracle.py` reads the resulting receipt, verifies its
source hashes, and updates only the hash expressions in the existing packet
header. Its output depends on those exact SDK sources and packet fixtures.

In the preserved first ESP32 qualification capture, Opus configurations 0 and
1 passed and configuration 2 failed its original fast-profile hash. That is
consistent with the host profile comparison, but the target's actual hash and
PCM were not recorded. The new slow-profile oracles have **not** been confirmed
on physical hardware, and matching the first observed pass/fail pattern does
not establish that all 16 target outputs match. Local compilation and host
arithmetic-profile testing do not establish target execution or playback.
