idf_component_register(SRCS "Esp32S3Hal.cpp" "display.cpp" "hello_world_main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES jgromes__radiolib nixy4__u8g2 driver esp_timer)
