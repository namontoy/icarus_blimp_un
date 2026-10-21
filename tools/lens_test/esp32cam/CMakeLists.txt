# tools/lens_test/esp32cam/CMakeLists.txt - ESP-IDF v5.x project for the lens-test
# server that runs on an AI-Thinker ESP32-CAM (plain ESP32, not the S3).
#
# Bench tool: it has nothing to do with the flight firmware in firmware/, it only
# shares the ESP-IDF install. Build it with tools\lens_test\build.bat, which routes
# through tools\idf_build.bat so the Windows PATH workarounds apply here too.
cmake_minimum_required(VERSION 3.16)

# The ESP32-CAM carries the original ESP32; sdkconfig.defaults repeats this so a
# fresh checkout picks the right chip whichever path takes effect first.
set(IDF_TARGET esp32)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(lens_test)
