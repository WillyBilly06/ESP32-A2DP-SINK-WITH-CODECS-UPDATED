# Recovery main component - WiFi OTA with encrypted firmware
idf_component_register(
    SRCS "recovery_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash 
        esp_partition 
        app_update 
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_http_client
        esp_https_ota
        esp-tls
        mbedtls
        lwip
        driver
)
