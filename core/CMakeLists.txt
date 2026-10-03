# core/CMakeLists.txt - registers the portable control core as an ESP-IDF
# component, so firmware/ compiles the EXACT SAME sources that
# tools/build.py compiles and unit-tests on the host
# (core/include/blimp/blimp.h's own stated design goal: no reimplementation,
# no drift between "sim controller" and "real controller").
#
# THIS FILE HAS NO EFFECT ON THE HOST BUILD: tools/build.py invokes clang
# directly against core/src/*.c per its own DEPS table and never reads
# CMake at all - adding this component registration cannot change what
# `python tools/build.py module <name>` does.
#
# SRCS is a glob over core/src, not an explicit file list, because other
# core modules (mixer.c, estimator.c, guidance.c, blimp.c, ...) are being
# written concurrently by other agents in this tree and did not all exist
# yet when this file was written - see core/include/blimp/*.h for the full
# module list tools/build.py's own DEPS table names. CONFIGURE_DEPENDS asks
# CMake to re-glob on every configure (a real but small cmake-time cost) so
# a newly-added core/src/*.c is picked up without hand-editing this file.
# NOTE: no CONFIGURE_DEPENDS. VERIFIED BY A REAL BUILD (ESP-IDF v5.3.2):
# ESP-IDF evaluates every component CMakeLists.txt in CMake SCRIPT MODE during
# its component-requirements pass (component_get_requirements.cmake), and
# CONFIGURE_DEPENDS is illegal there:
#     CMake Error at core/CMakeLists.txt:19 (file):
#       CONFIGURE_DEPENDS is invalid for script and find package modes.
# The trade-off is that ADDING A NEW core/src/*.c REQUIRES `idf.py reconfigure`
# (or a clean build) before the firmware picks it up. The host test build
# (tools/build.py) is unaffected - it never reads CMake.
file(GLOB BLIMP_CORE_SRCS "${CMAKE_CURRENT_LIST_DIR}/src/*.c")

# blimp_ffi.c is EXCLUDED deliberately. Per tools/build.py's build_dll(), it
# exists only to __declspec(dllexport) the API for the host-side MuJoCo
# ctypes DLL (build/libblimp.dll) - that is MSVC/Windows-DLL-export glue,
# not portable C11, and must never be compiled by xtensa-gcc.
list(FILTER BLIMP_CORE_SRCS EXCLUDE REGEX ".*_ffi\\.c$")

if(NOT BLIMP_CORE_SRCS)
    # No core/src/*.c exists yet (or only blimp_ffi.c does) at configure
    # time - fall back to an interface-only (headers-only) component so the
    # project can still configure while core/src/ is still being populated
    # by other agents. Not expected to be exercised today: core/src/params.c
    # and core/src/pid.c already exist as of this file being written.
    idf_component_register(INCLUDE_DIRS "include")
else()
    idf_component_register(
        SRCS ${BLIMP_CORE_SRCS}
        INCLUDE_DIRS "include"
    )

    # core/ is portable C11 with no ESP-IDF/FreeRTOS/stdio/malloc dependency
    # (blimp.h's own contract). Match tools/build.py's CFLAGS as closely as
    # ESP-IDF's build system allows, so a warning caught on the host is
    # caught here too and vice versa. -Wconversion is deliberately strict
    # (see tools/build.py's own comment) - core/src is written to satisfy it
    # cleanly; do not weaken it here to silence a real bug.
    #
    # Deliberately NOT -Werror, unlike tools/build.py: xtensa-gcc may surface
    # a warning clang did not (or vice versa) on the very first real compile
    # of this code, and this project has never been built with the real
    # toolchain. Do not let an unverified warning hard-fail that first
    # build. Promote to -Werror once a real `idf.py build` has gone through
    # clean at least once (docs/08-firmware-setup.md).
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -std=c11
        -Wall
        -Wextra
        -Wconversion
        -Wshadow
    )
endif()
