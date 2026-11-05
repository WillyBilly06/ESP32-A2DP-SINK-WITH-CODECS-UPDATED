# BDK diagnostics viewer

Run `native_bt/tools/start_serial_dashboard.ps1` to reuse or start the background
service. It prints the local URL (normally `http://127.0.0.1:8765/`). Opening or
closing its browser tab does not open or close the serial port. A reopened tab
loads the retained history.

The viewer keeps up to 10,000 individual records in RAM, renders at most 400
matching rows, and fetches incremental batches asynchronously. UART reading,
HTTP requests, and capture following run independently. Pausing the display
continues collection. There are stage, severity, source, text and CRC filters,
an original-line/structured-JSON inspector, filtered NDJSON export, full-buffer
download, and replay of admitted test captures. Periodic snapshots are retained
as separate records. Source labels and capture-write times distinguish recorded
data from live UART. Invalid CRC records remain visible but do not replace
latest metrics. Older `LIVE1` firmware telemetry is retained and parsed too;
JSON PCM container width is not presented as negotiated source bit depth.

**Release COM6 for flash** closes the actual serial reader. The page remains
open and follows `native_bt/_build/hardware/COM6-probe-trial-live.txt` while the
test tool owns the port. **Resume UART** switches back to the verified CP210x
speaker. No serial command, DTR/RTS reset, flash or firmware control is sent.
Only the released COM6 fixture is admitted; other ports remain untouched.

Before flashing from a script, release and require the controller's real
reader acknowledgement:

```powershell
python native_bt/tools/serial_monitor_ctl.py release
# Run the normal verified finite deployment only after acknowledgement.
python native_bt/tools/serial_monitor_ctl.py resume
```

Resume after the deployment tool has closed COM6. The viewer intentionally
stays released until explicitly resumed, avoiding competition with the flasher.
Replaying a capture also releases the reader. Controls use loopback-only HTTP,
an origin check and a per-service control token; the existing local command-file
controller remains available. No external service or CDN is used.

Validation: six host tests cover full-record retention, legacy format, invalid
CRC isolation, concurrent bounded history, replay-path rejection and close-before-
release acknowledgement. Real UI checks cover search, stage/integrity filters,
the inspector, resume/release and display pause while background collection
continues. An independent exclusive COM6 open succeeds after release while the
HTTP viewer remains responsive; UART-write and deliberate-reset counters stay
zero. Unauthorized HTTP controls and an untrusted Host are rejected. Export is
valid NDJSON. A local checkpoint preserves the buffer through the service update.

This is host-side logging/UI decoupling. It does not certify firmware logging
latency, audio continuity or the custom Bluetooth stack's remaining goal gates.
