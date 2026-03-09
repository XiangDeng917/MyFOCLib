cmake_minimum_required(VERSION 3.16)
project(foc_reusable_lib C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(FOC_BUILD_TESTS "Build host-side smoke test" ON)

add_library(foc STATIC
    foc.c
    src/BLDCMotor.c
    src/MagneticSensor.c
    src/delay.c
    src/pid.c
    src/my_filter.c
)

target_include_directories(foc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(FOC_BUILD_TESTS)
    include(CTest)
    enable_testing()

    add_executable(foc_smoke_test
        tests/smoke_test.c
    )

    target_link_libraries(foc_smoke_test PRIVATE foc)

    if(UNIX AND NOT APPLE)
        target_link_libraries(foc_smoke_test PRIVATE m)
    endif()

    add_test(NAME foc_smoke_test COMMAND foc_smoke_test)
endif()
