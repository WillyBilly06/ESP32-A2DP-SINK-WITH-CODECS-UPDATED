# Building this directory as a second IDF component duplicates global codec
# ownership, even if one copy appears unused. Use the one canonical component.
message(FATAL_ERROR "Use native_bt/esp32_probe/components/lhdcv5_decoder as the sole LHDC V5 component")
