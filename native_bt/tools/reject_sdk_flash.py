"""Reject generic SDK flashing without opening or enumerating a serial port."""
raise SystemExit("Generic SDK flash targets are disabled for this owned project: their default boot-slot/partition writes are not the admitted OTA operation. Use the identity-checked deployment helper only after the user releases HARDWARE_HOLD. No device was accessed.")
