# UART diagnostic cadence follow-up

The owned COM6 application and live UART monitor run at **921600 baud**.
The two cumulative `media_arrival_fifo` and `vhci_ready_probe` records now share
the existing one-second background reporting gate instead of printing after
each 100 ms trace-drain interval. This removes nine of every ten scheduled
reports for those records. Their counters remain cumulative. Protocol sends,
codec processing, core placement, queue sizes and the first-gap capture are
unchanged by this edit. Records remain separate, with their original fields
and CRC; the dashboard continues to show full raw UART.

Trace-ring and first-gap draining still run before the slower reporting gate.
The logger is priority 1 on Core 1. UART transmission uses the existing IRQ
driver, whose background writer can sleep for TX space. This is not a claim
that every firmware logging path is nonblocking: formatting, CRC computation,
locks and interrupt handling have a cost. No new TX buffer or task is added.

## Verification

- ccache target build passed, with compiler-input rechecks and source-content
  stability for 10,413 manifest entries; actual Ninja compilation rules use
  the ccache launcher. Rebuildable output: 508.520 MiB under the 512 MiB gate.
- Console admission: 4 CMake, 14 image, 9 SDK and 1 CLI cases passed.
  Dashboard: all 9 tests passed.
- Image `e792418bf692dcaa351799969bc1f46e755d0d58b00fd7f9130bcc8e85ec517b` was written at
  921600 and verified by complete readback at 115200. Original application
  preserved; owned selection VALID; no automatic Bluedroid restore.
- Existing 37-configuration / 173-callback cold codec check passed. Tracked
  allocation peak 56,899 B; minimum free after allocation 101,924 B; observed
  stack low-water 3,472 / 8,192 B. This is not a live playback test.
- After resume, logger PID 34852 / source 77 was connected with verified USB
  identity at 921600. Fresh 6.220-second passive capture: 245 records, all CRC
  Verified, including 5 of each reduced diagnostic. The earlier 6.226-second
  capture had 48 arrival and 49 readiness records. These idle windows have
  different codec-lifecycle state, so total-byte and CPU deltas are not a
  matched performance comparison.
- Linked executable SRAM, IRAM, flash text and rodata sizes are unchanged
  versus the preceding `6e1acb5b` build.
- Independent phone-side pause guard completed; native YouTube was verified
  PAUSED afterward. I2S stopped. No audio fixture was started for this check.

Evidence: `native_bt/_build/experiments/log-cadence-1794119049011/` contains
full before/after NDJSON, passive receipts, section comparisons and build/deploy
logs. The first PowerShell 5 build attempt stopped on IDF activation stderr
before compilation; its failure log is retained. PowerShell 7 build passed.

LHDC continuity remains unresolved. The preceding 50 ms high-rate startup
trial had 1,280 / 896 unfilled frames in two short 192 kHz/S24/900 kbps YouTube
windows; those results precede this logging change and do not qualify continuous
playback, 1 Mbps, maximum-load coexistence or the complete feature inventory.
