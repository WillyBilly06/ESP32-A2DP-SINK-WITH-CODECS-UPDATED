#ifndef DBT_ESP32_AFFINITY_H
#define DBT_ESP32_AFFINITY_H

/* Product layout requested by the user: isolate codec execution on CPU0.
 * CPU0 still has required FreeRTOS idle/tick/IPC and hardware interrupts. */
#define DBT_ESP32_BLUETOOTH_CORE 1
#define DBT_ESP32_DECODER_CORE 0
#define DBT_ESP32_RENDER_CORE 1
#define DBT_ESP32_BACKGROUND_CORE 1
/* Bounded, preemptible reconnect preparation uses CPU0 idle quanta.
 * Controller/protocol owners and fresh pairing keep their existing cores. */
#define DBT_ESP32_RECONNECT_WORKER_CORE 0
/* Existing EQ/control/CCCD NVS worker stays below decoder/DSP priority.
 * General provider and OTA work retain BACKGROUND_CORE. */
#define DBT_ESP32_PERSISTENCE_WORKER_CORE 0

#endif
