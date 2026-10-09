idf_component_register(SRCS "Esp32S3Hal.cpp" "display.cpp" "icm20948.cpp" "gps.cpp" "phone_gps.cpp" "hello_world_main.cpp" "ICM_20948_C.c"
                       INCLUDE_DIRS "."
                       REQUIRES jgromes__radiolib nixy4__u8g2 h2zero__esp-nimble-cpp driver esp_timer)

# SparkFun ICM-20948 C core: enable the DMP (firmware image + fusion engine).
# Without this define the C core reports ICM_20948_Stat_DMPNotSupported.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ICM_20948_USE_DMP)
