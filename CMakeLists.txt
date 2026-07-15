cmake_minimum_required(VERSION 3.20)

project(74000m_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_core STATIC
    src/robot.cpp
)
target_include_directories(robot_core PUBLIC include)

if(MSVC)
    target_compile_options(robot_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(robot_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(robot_tests
        tests/test_main.cpp
        tests/test_build_info.cpp
        tests/test_units.cpp
        tests/test_pose2d.cpp
        tests/test_config.cpp
        tests/test_contracts.cpp
        tests/test_hal_contract.cpp
        tests/test_runtime.cpp
        tests/test_telemetry.cpp
        tests/test_drive_math.cpp
        tests/test_sensors.cpp
        tests/test_odometry.cpp
        tests/test_manual_drive.cpp
        tests/test_scheduler.cpp
        tests/test_health.cpp
        tests/test_calibration.cpp
    )
    target_link_libraries(robot_tests PRIVATE robot_core)
    target_include_directories(robot_tests PRIVATE tests)
    if(MSVC)
        target_compile_options(robot_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(robot_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME robot_tests COMMAND robot_tests)
endif()
