#ifndef DBT_ESP32_AFFINITY_H
#define DBT_ESP32_AFFINITY_H

/* Product layout requested by the user: isolate codec execution on CPU0.
 * CPU0 still has required FreeRTOS idle/tick/IPC and hardware interrupts. */
#define DBT_ESP32_BLUETOOTH_CORE 1
#define DBT_ESP32_DECODER_CORE 0
#define DBT_ESP32_RENDER_CORE 1
#define DBT_ESP32_BACKGROUND_CORE 1

#endif
