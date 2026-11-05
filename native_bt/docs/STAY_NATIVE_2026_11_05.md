# Explicit persistent owned development firmware

The user requested that COM6 run our own stack and stop automatically returning
to Bluedroid after tests. `-StayNative` is the explicit development selection for
that request. It requires the complete asynchronous product entry and enables
`KEEP_OWNED_ACTIVE`; isolated codec/pipeline/checksum/sampler entries cannot use it.
Finite experiments keep their previous guards when this explicit mode is absent.

Deployment requires both `--keep-owned` and `--stay-native`, matched to the actual
build receipt/cache/main definition. The existing persistent-selector path writes
a VALID development selection for the owned image; the finite-trial reset/return
path is not entered. This does not assert production readiness or disable normal
rollback behavior for future OTA images. The old flash backup and released image
remain manual recovery artifacts, not an automatic return after this run.

Bootstrap `boot_memory` records identify the host as owned and report internal
8-bit free/largest/minimum/total bytes and the owner stack at four stages:
before controller setup, controller initialized, controller enabled and services
ready. These are separate stage observations, not one coherent resource snapshot.
The ordinary audio logger continues to report active-stream heap/stack counters.

Twelve actual CMake cases, thirteen actual image-admission cases and six actual
deployment cases pass. Existing decoder/order/compact/block/word/PCM-target guards
also pass unchanged for ordinary finite modes. Target build, persistent selection
and observed initialization memory remain the next verification steps. Full
maximum-LHDC playback and feature/stress qualification remain incomplete.
