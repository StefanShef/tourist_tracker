# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# nixy4/u8g2 0.1.4 uses i2c_master_bus_config_t.flags.allow_pd, which was
# introduced in ESP-IDF 5.4. On ESP-IDF 5.3 the member is absent and the
# component fails to compile. We patch the file both before and after
# project() so the fix applies whether the component is already present or
# downloaded by the component manager during configuration.
macro(_patch_u8g2_i2c_for_idf53)
    set(_U8G2_I2C_C "${CMAKE_CURRENT_SOURCE_DIR}/managed_components/nixy4__u8g2/src/port/esp32_hw_i2c.c")
    if(EXISTS "${_U8G2_I2C_C}")
        file(READ "${_U8G2_I2C_C}" _U8G2_I2C_C_CONTENT)
        string(REPLACE ".flags.allow_pd = 0," "" _U8G2_I2C_C_CONTENT "${_U8G2_I2C_C_CONTENT}")
        file(WRITE "${_U8G2_I2C_C}" "${_U8G2_I2C_C_CONTENT}")
    endif()
endmacro()

_patch_u8g2_i2c_for_idf53()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hello_world)

_patch_u8g2_i2c_for_idf53()
