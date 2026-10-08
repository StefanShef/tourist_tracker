idf_component_register(SRCS "Esp32S3Hal.cpp" "display.cpp" "icm20948.cpp" "gps.cpp" "phone_gps.cpp" "hello_world_main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES jgromes__radiolib nixy4__u8g2 h2zero__esp-nimble-cpp driver esp_timer)
