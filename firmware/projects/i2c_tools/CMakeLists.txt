# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# core/ (the flight controller, portable C11) is an ESP-IDF component of its
# own (core/CMakeLists.txt), so this firmware compiles the same sources that
# tools/build.py tests on the computer and the simulator runs. Since
# 2026-10-30 the 'param' command (main/cmd_param.c) uses its blimp_params_t;
# increment 3 of docs/control-loop-plan.md adds blimp_step().
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../core")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(i2c_tools)
