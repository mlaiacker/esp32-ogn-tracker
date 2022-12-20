# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)


#set(BUILD_DIR "build-${IDF_TARGET}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set (EXTRA_COMPONENT_DIRS "${PROJECT_DIR}/components/tft")

project("${IDF_TARGET}-ogn-tracker")

#idf_build_process(esp32s3
#                  SDKCONFIG "sdkconfig.esp32s3"
#                  SDKCONFIG_DEFAULTS "sdkconfig.defaults.esp32s3"
#                  BUILD_DIR "${CMAKE_SOURCE_DIR}/build-esp32s3"
#                  )

