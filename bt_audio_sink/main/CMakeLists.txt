idf_component_register(
    SRCS "main.cpp" "bt/a2dp_sink_native.cpp" "ota/idf_update.cpp" "led/led_strip_encoder.c"
    INCLUDE_DIRS "." "audio" "ble" "config" "bt" "dsp" "input" "led" "ota" "storage"
    PRIV_REQUIRES bt nvs_flash app_update esp_partition esp_timer esp_wifi esp_netif
                  esp_event esp_http_client esp_http_server esp_https_ota esp-tls
                  mbedtls spiffs esp_driver_i2s esp_driver_spi esp_driver_rmt
                  esp_driver_gpio driver)

# Suppress warnings for compatibility with newer GCC
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -DESP32_CMAKE=1 
    -Wno-error 
    -Wno-format 
    -fpermissive 
    -mtext-section-literals
    -Wno-overloaded-virtual
)