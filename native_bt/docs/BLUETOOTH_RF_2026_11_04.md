# Current Bluetooth radio checks

The independently authored host is still experimental. The released Bluedroid
application remains preserved in ota0. Only COM6 was accessed; the attached
peripheral is the DAC. These checks do not establish feature parity.

## Selected-peer Classic pairing

On 2026-11-04 the Redmagic discovered `NATIVE_BT_PROBE`, presented its ordinary
pairing dialog, and was paired without contact/call-history access. Android's
device-details address matched the actual HCI controller address on COM6.

- Source: `c9d080baf03b4902cafa4af6a890bf5b2b4da440`.
- Image SHA-256: `48b052be75e9118590e1174c4d818429f27a1190ef630cbe17fa165a4aa0c6db`.
- ESP-IDF 6.1, controller-only dual mode; Classic security and diagnostic GATT,
  without audio/product profiles. Built with ccache.
- Admission accepted the explicitly configured phone address. No other peer was
  authorized by this diagnostic policy.
- The native security owner reported a committed bond (event 1, status 0), then
  encryption ready (event 0, status 0, key size 16, bonded true).
- Key type 4 and `authenticated=false` describe an unauthenticated Just Works SSP
  key. This is encryption/bonding evidence, not MITM or Secure Connections proof.
- A later link-removed event retained bonded state. A tap on Connect produced no
  additional admission event; reconnection was not verified by this image.
- The unconfirmed finite trial reset and booted the preserved `1.0.4-idf6.1`
  application from ota0, reached `System ready`, and showed no fatal boot marker.

The private local archive is
`native_bt/_build/hardware/runs/1793812485016473800-48b052be75e9`.
It contains the exact image/build inputs, UART capture, final return receipt,
phone action timestamps and address-confirming device-details XML. Full flash,
NVS backups and keys are not published.

The callback's product-consent and selected-peer decisions also passed 32 host
cases with an optional/full nonblocking log queue. Those cases prove policy
preservation in stubs, not RF authentication.

## Outstanding Bluetooth gates

The full profile build still needs current-image bond reload/reconnection,
A2DP and AVRCP exchanges, live codec negotiation/switching, BLE security and all
BDK command flows, simultaneous streaming/control, and lifecycle/stress tests.
Diagnostic GATT readiness alone does not prove an external BLE connection.
Production pairing consent without physical controls remains a separate gap.
