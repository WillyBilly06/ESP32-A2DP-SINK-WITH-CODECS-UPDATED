# Startup heap evidence

Updated 2026-10-29. These are historical saved measurements, checked against the
boot ELF markers and verified flash receipts. No board reset or connection was
performed for this audit. Reproduce with `python native_bt/tools/audit_heap_snapshots.py`;
the bounded output is `_build/host/heap_audit.json`.

The saved 92 KiB ready reading belongs to the **Bluedroid audio application**,
firmware SHA 81efbf030dd335de92473facec2735874a2b62cefdefeb1613cc420bc04e4ebc.
Its boot explicitly reports `host=bluedroid`. It is not the independent host image.
Logs label KiB as KB and truncate to integer KiB.

| Saved audio-app checkpoint | Free internal heap (KiB, truncated) |
| --- | ---: |
| After NVS | 224 |
| After SPIFFS | 222 |
| After sound player | 215 |
| After DSP setup | 215 |
| After I2S setup | 211 |
| After audio pipeline and overlay | 185 |
| After render task / LED setup | 175 |
| After controller + Bluedroid initialization | 110 |
| After BLE init call | 110 |
| After A2DP start call | 106 |
| After Bluetooth ready wait | 92 |

The first later idle diagnostic reports 86,020 free internal bytes (~84 KiB).
Therefore ready-stage and later-idle readings are different checkpoints even in
one image. Deltas include asynchronous startup activity; the 65 KiB change from
175 to 110 cannot be assigned to the host alone because controller and host are
initialized between those checkpoints.

Source-backed eager reservations in this baseline include a 16,384-byte PCM
ring, 2,064-byte DSP output buffer (256+2 stereo S32 frames), 8,192-byte overlay
ring, DMA storage and several task stacks. The 26 KiB pipeline delta agrees in
scale with those first three buffers, before allocator overhead. Sound player,
render and LED tasks, Bluetooth tasks and profile queues add their own costs.
Static BSS/DRAM reservations reduce the initial heap before any of these logs.
This is a resource inventory, not proof that these buffers can be removed while
retaining the existing playback behavior.

The archived owned probe d85bf6be79440780ac52564c8048f03c72e33a4448e2a4f278f20dc9caa529f9
reports **191,100 free internal bytes (186.6 KiB)** and a largest block of
110,592 bytes (108 KiB) at `controller_identified`. Its host object is 6,680
bytes, included inside the 10,648-byte runtime; these must not be added together.
This is an identification checkpoint with `profiles_ready=false`, inactive
decoder/output buffers and incomplete product/profile coverage. It is not an
equivalent full speaker comparison. The user-reported BTstack 120–130 KiB has
not been tied to a matching build/configuration/checkpoint in this audit.

Qualification still requires the same controller configuration, enabled services,
maximum links/MTUs, security state and audio resources, measured at startup,
ready-idle, active codec/configuration, codec switching and sustained Classic/BLE
traffic. Track free bytes, largest block, static RAM, stack reservations and peak
allocation together. No lower-RAM or faster-than-BTstack claim is established.
