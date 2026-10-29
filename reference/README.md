# BTstack reference checkout

Upstream: https://github.com/bluekitchen/btstack

Pinned commit: `51e11ce3692abfe31b4ee3d66cffdeb1ae1626f2`

Upstream commit date: 2026-10-23. Inspected on 2026-10-28.

The shallow checkout uses Git sparse checkout for `src`, `platform`, `port/esp32`,
`doc`, `tool`, and `chipset`, retaining upstream root files and its license.
The complete Git tree remains addressable with `git show`; missing working-tree
paths can be restored through sparse checkout if needed. No submodules were initialized.

This is reference material, not a component or library dependency. Owned code and
the ESP32 probe build without including or linking this checkout. Do not run its
example generator against the application or copy its port into owned sources.

The upstream license is retained in `btstack/LICENSE` and applies to the reference
material. Our implementation is written separately against standard protocol
requirements. Source-path checks establish the build boundary, not mathematical
proof of independent authorship.

Measured retained size, including `.git`: about 70.6 MiB. Do not fetch/pull or expand
all ports automatically; record any deliberate reference revision change.
