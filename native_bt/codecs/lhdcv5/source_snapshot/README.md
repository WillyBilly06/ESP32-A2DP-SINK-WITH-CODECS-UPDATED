# Pinned portable LHDC V5 source

This directory contains the 16 portable C sources/headers used by the custom-host build. Source notices are preserved. The source comes from the user-provided [LHDC V5 decoder](https://github.com/WillyBilly06/LHDC-V5-Decoder) working snapshot. Fourteen files match upstream revision `cf857ce1a1429965e161e54adbc3f293fd9ce273`; `lhdc_imdct.c` and `lhdc_imdct.h` contain the tested local changes. [SOURCE.json](SOURCE.json) records every file digest.

The component builds a generated, audited port through [prepare_port.py](../prepare_port.py). Its source pins must match; it does not fall back to a different decoder. This snapshot contains decoder C code, not a Bluetooth host or proprietary encoder binary.
