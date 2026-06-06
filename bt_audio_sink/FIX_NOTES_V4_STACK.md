# v4 stack fix

This package keeps the v3 low-RAM mixer changes, but restores safe task stack sizes for the foreground control tasks.

Reason:
- v3 reduced the `buttons` task stack too aggressively to save RAM.
- Runtime log showed `***ERROR*** A stack overflow in task buttons has been detected.`

Changes:
- `main/main.cpp`: `buttons` task stack 1536 -> 3072 bytes.
- `main/main.cpp`: `beat` task stack 1536 -> 2048 bytes.
- `main/core/main.cpp`: mirrored the same safer stack sizing for the alternate/source-copy main file.

This costs about 2 KB more internal RAM than v3, but your runtime log showed around 60 KB free internal RAM at LDAC 96 kHz after skipping the connected sound, so this is a much safer tradeoff.
