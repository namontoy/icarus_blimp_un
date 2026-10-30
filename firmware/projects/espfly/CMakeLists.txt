# firmware/CMakeLists.txt - top-level ESP-IDF v5.x project file.
#
# ============================================================================
# UNCOMPILED. This project has never been configured or built with a real
# ESP-IDF install or xtensa toolchain - see docs/08-firmware-setup.md, which
# also documents exactly how to bring one up on Windows. Every mechanism
# below is written to match documented ESP-IDF v5.x CMake conventions as
# closely as could be verified without a compiler in this environment;
# anywhere that could not be fully verified is marked UNVERIFIED with a
# stated fallback. Confirm each one at first `idf.py build` / `idf.py
# reconfigure` before trusting this file.
# ============================================================================
cmake_minimum_required(VERSION 3.16)

# Hard-select the target here rather than relying only on
# `idf.py set-target esp32s3` or sdkconfig.defaults, so a fresh clone
# defaults to the right chip.
#
# UNVERIFIED: setting IDF_TARGET in the top-level CMakeLists.txt before
# including project.cmake is documented ESP-IDF behaviour ("Selecting the
# Target" in the ESP-IDF build-system docs), but has not been exercised
# against a real IDF install here. firmware/sdkconfig.defaults also sets
# CONFIG_IDF_TARGET=esp32s3 as a second, independent path to the same
# result. If NEITHER takes effect on the installed IDF version, run
# `idf.py set-target esp32s3` once by hand (docs/08).
set(IDF_TARGET esp32s3)

# Makes ../core visible to ESP-IDF's component search as the "core"
# component, so the SAME control-core sources tools/build.py compiles and
# tests on the host (core/CMakeLists.txt) are compiled unchanged into this
# firmware - see core/include/blimp/blimp.h's own header comment for why
# that matters ("no reimplementation, no sim/real drift").
#
# UNVERIFIED MECHANISM (documented, but not exercised here): ESP-IDF's
# EXTRA_COMPONENT_DIRS accepts EITHER a directory that itself directly
# contains a component's CMakeLists.txt (registered as a single component
# named after that directory - our case, since core/CMakeLists.txt lives
# directly in core/, not in a subdirectory of it) OR a directory-of-
# components (each immediate subdirectory checked for its own
# CMakeLists.txt). Confirm at first `idf.py reconfigure` (or the component
# list printed during configure / in build/CMakeCache.txt) that a component
# literally named "core" appears. If it does not, the unambiguous fallback,
# supported identically by every ESP-IDF version, is the directory-of-
# components form instead:
#     set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
# (pointing at the DroneUN repo root - core/ is then found as one of its
# immediate subdirectories).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../core")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(droneun_firmware)
