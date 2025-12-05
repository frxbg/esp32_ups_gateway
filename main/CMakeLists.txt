idf_component_register(SRCS "esp32_ups_gateway.c" "modbus_ups.c" "wifi_manager.c" "hid_worker.c" "http_server.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "../index.html"
                       REQUIRES esp_http_server esp_wifi nvs_flash esp-modbus json)
