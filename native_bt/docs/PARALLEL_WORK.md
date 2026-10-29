# Active parallel work

The user requested internal subagents in this chat, all Terra at xhigh. The
temporary separate chats were stopped and archived; partial files were preserved.

- Parent: ERTM, shared runtime/channel/CMake/probe changes, global verification,
  resource accounting, integration review and publication.
- ble_security: SMP sources/bearer, dedicated tests and SMP.md; continuing SC
  state machines and bond/storage integration. Primitives alone are not pairing.
- codec_workers: common/AAC/Opus/SBC descriptors, dedicated tests and codec docs;
  AAC/SBC have actual Windows PCM evidence. AAC needs its guarded Helix core.
- avrcp_profiles: AVCTP/AVRCP/bearers and dedicated tests; continuing reject paths,
  controller recovery and browsing semantics. Browsing bearer still needs ERTM.

The parent reviews dedicated evidence before shared integration. Workers do not
edit shared runners, CMake/probe, progress/manifest or Git state. No combined
pass is claimed for the changing worktree. Published checkpoint 3364b90 has 166
groups; newer module passes have separate scopes. The updated global runner's
pre-build hash/mtime guard still needs a combined run after sources settle.
tests/test_memory.c is Windows-only harness support, not firmware code.

The offline phase continues: no flash, serial reset, Bluetooth connection/playback
or Linux Bluetooth/Firefox changes. Preserve the audio baseline, immediate EQ NVS,
active-codec allocation and reference-only BTstack boundary.
