# ===========================================================================
# MyFOCLib — CMakeLists.txt
#
# Supported targets
#   Linux / macOS / Windows (MinGW or MSVC)
#
# Quick start
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build
#   ctest --test-dir build --output-on-failure
# ===========================================================================

cmake_minimum_required(VERSION 3.15)

project(MyFOCLib
    VERSION     1.0.0
    DESCRIPTION "Reusable FOC static library for brushless DC motor control"
    LANGUAGES   C
)

# ---------------------------------------------------------------------------
# C standard
# ---------------------------------------------------------------------------
set(CMAKE_C_STANDARD          11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS        OFF)

# ---------------------------------------------------------------------------
# Default build type
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Build type: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()

# ---------------------------------------------------------------------------
# Compiler warnings (applied to all targets via an interface library)
# ---------------------------------------------------------------------------
add_library(foc_compile_options INTERFACE)

if(MSVC)
    target_compile_options(foc_compile_options INTERFACE
        /W4
        /WX
    )
else()
    target_compile_options(foc_compile_options INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wdouble-promotion
        -Wno-unused-parameter   # common in embedded stubs
        $<$<CONFIG:Debug>:-g3 -fsanitize=address,undefined>
        $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
    )
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_options(foc_compile_options INTERFACE
            -fsanitize=address,undefined
        )
    endif()
endif()

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(FOC_SOURCES
    src/foc.c
    src/BLDCMotor.c
    src/MagneticSensor.c
    src/delay.c
    src/pid.c
    src/my_filter.c
)

# ---------------------------------------------------------------------------
# Static library: libfoc
# ---------------------------------------------------------------------------
add_library(foc STATIC ${FOC_SOURCES})

target_include_directories(foc
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
)

target_link_libraries(foc
    PUBLIC  foc_compile_options
    PRIVATE m   # math library (sin, cos, …)
)

# MSVC does not have a separate math library
if(MSVC)
    target_link_libraries(foc PRIVATE)
endif()

# ---------------------------------------------------------------------------
# Example executable
# ---------------------------------------------------------------------------
add_executable(bldc_foc_demo examples/bldc_foc_demo.c)

target_link_libraries(bldc_foc_demo
    PRIVATE foc foc_compile_options m
)

# ---------------------------------------------------------------------------
# Tests (CTest)
# ---------------------------------------------------------------------------
enable_testing()

add_executable(unit_test_foc tests/unit_test_foc.c)

target_link_libraries(unit_test_foc
    PRIVATE foc foc_compile_options m
)

add_test(NAME unit_test_foc
         COMMAND unit_test_foc
)

# ---------------------------------------------------------------------------
# Install rules
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS foc
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/myfoclib
    FILES_MATCHING PATTERN "*.h"
)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
message(STATUS "-----------------------------------------------")
message(STATUS " MyFOCLib v${PROJECT_VERSION}")
message(STATUS " Build type  : ${CMAKE_BUILD_TYPE}")
message(STATUS " C standard  : C${CMAKE_C_STANDARD}")
message(STATUS " Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "-----------------------------------------------")
