# Static IRAM placement comparison

The combined owned product uses the existing SDK option
`CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH=y`. The SDK linker fragment moves
selected non-ISR task APIs to flash; interrupt entry points and dependencies
retain the SDK's IRAM placement. `CONFIG_FREERTOS_PLACE_ISR_FUNCTIONS_INTO_FLASH`
remains disabled and `CONFIG_I2S_ISR_IRAM_SAFE=y` remains enabled. The relevant
primary sources are the existing SDK's `components/freertos/Kconfig` and
`components/freertos/linker.lf`; no SDK clone or replacement was introduced.

With all six codec families, SMP, AVRCP, GATT and product IO enabled, two
sequential builds used identical owned/SDK C sources and the same feature
flags. The source manifests differ only in sdkconfig, its defaults and the
generated sdkconfig header. Only the task-placement option changed.

| Static link measurement | Task APIs in IRAM | Selected task APIs in flash |
| --- | ---: | ---: |
| IRAM used | 130,243 bytes | 123,179 bytes |
| IRAM remaining | 829 bytes | 7,893 bytes |
| Static DRAM used | 50,760 bytes | 50,760 bytes |

This saves **7,064 bytes of static IRAM**. It is not a measurement of runtime
free heap, task stacks, cache stalls, CPU load, latency, or an advantage over
Bluedroid/BTstack. Source manifests, image hashes and the static comparison are
recorded in `LOCAL_CANDIDATE_EVIDENCE.json` and fixed `_build/host/iram-*.json`.
No ESP32 hardware was accessed.

Run `tools/check_iram_placement.py --expect-task-flash` with the existing SDK
Python after a build. It checks six selected interrupt/context-switch symbols,
task-only API placement, IRAM I2S callbacks, and direct calls through four
I2S/owned audio functions. SDK calls are terminal nodes; four indirect calls,
ROM/vendor internals, complete execution paths and real NVS/I2S coexistence
remain unproven. This limited audit must not be presented as universal
cache-disabled execution or audible continuity proof.
