# ERTM implementation and verification

The independently authored enhanced-control/FCS wire codec handles I/S control,
RR/REJ/RNR/SREJ, poll/final, SAR, first-frame SDU length and optional FCS. Its
32-byte constant nibble table is generated from the polynomial. No reference
implementation or table was copied.

The owned ERTM engine now has an opt-in Classic channel/runtime binding. The
default probe still does not register AVRCP browsing. See
[ERTM channel integration](ERTM_FEATURE_DISCOVERY.md) for negotiation policy,
memory lifetime, paired-runtime tests, and the remaining limitations.

The dedicated test runner verifies official Core I-frame/RR FCS vectors, all
65,536 I-frame field combinations, 1,536 supervisory combinations, a separate
bit-serial CRC oracle, corruption/CID binding, truncation and capacity limits.
Windows and Linux pass; Linux ASan/UBSan pass. Source hashes are captured before
compilation and checked afterward. The standalone ESP32 wire/engine object was
also compiled without mutable static symbols. That standalone result does not
establish a complete firmware link or hardware behavior.

The engine covers modulo-64 windows, ACK/SREJ distinctions, lazy retransmission
metadata, busy/poll/final handling, retry limits, admission-based timers, and
channel-level failure isolation. The binding currently requires the default
16-bit FCS and disconnects on unsupported ERTM continuation/reconfiguration.
For non-flushable ACL-U, sender timer minima are 2 seconds and 12 seconds;
requests carry zero timers. Peer MaxTransmit constrains local attempts, with
zero meaning infinite for ERTM. Information-payload MPS excludes the START SDU
length field. External-peer and sustained hardware qualification remain required.

Protocol: [Bluetooth SIG L2CAP](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/logical-link-control-and-adaptation-protocol-specification.html), sections 3.3, 5.4 and 8.6.
