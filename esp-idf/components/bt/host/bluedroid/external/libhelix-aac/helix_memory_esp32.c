/**
 * @file helix_memory_esp32.c
 * @brief ESP32-specific memory allocation for Helix AAC decoder
 * 
 * Replaces the C++ allocator with simple C heap_caps calls.
 * Allocates from internal RAM for fastest decoding performance.
 */

#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "HELIX_MEM";

void* helix_malloc(int size) {
    /* First try internal RAM for best performance */
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    if (ptr == NULL) {
        /* Fall back to PSRAM if internal is full */
        ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
        if (ptr) {
            ESP_LOGW(TAG, "Allocated %d bytes from PSRAM (internal full)", size);
        }
    }
    
    if (ptr == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes!", size);
    }
    
    return ptr;
}

void helix_free(void *ptr) {
    if (ptr) {
        heap_caps_free(ptr);
    }
}

/* Buffer for logging - used by helix_log.h */
char log_buffer_helix[256];
