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

add_executable(v5l_inspect tools/v5l_inspect.cpp)
target_link_libraries(v5l_inspect PRIVATE robot_core)
if(MSVC)
    target_compile_options(v5l_inspect PRIVATE /W4 /permissive-)
else()
    target_compile_options(v5l_inspect PRIVATE -Wall -Wextra -Wpedantic)
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
        tests/test_commissioning_curvature.cpp
        tests/test_scheduler.cpp
        tests/test_health.cpp
        tests/test_calibration.cpp
        tests/test_hmi.cpp
        tests/test_freeze_gate.cpp
        tests/test_control_foundations.cpp
        tests/test_chassis_velocity_controller.cpp
        tests/test_motion_commands.cpp
        tests/test_trajectory.cpp
        tests/test_trajectory_tracker.cpp
        tests/test_command_groups_routes.cpp
        tests/test_autonomy_safety.cpp
        tests/test_offline_closed_loop.cpp
    )
    target_link_libraries(robot_tests PRIVATE robot_core)
    target_include_directories(robot_tests PRIVATE tests)
    add_executable(v5l_fixture tests/v5l_fixture.cpp)
    target_link_libraries(v5l_fixture PRIVATE robot_core)
    if(MSVC)
        target_compile_options(robot_tests PRIVATE /W4 /permissive-)
        target_compile_options(v5l_fixture PRIVATE /W4 /permissive-)
    else()
        target_compile_options(robot_tests PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(v5l_fixture PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME robot_tests COMMAND robot_tests)
    set(V5L_FIXTURE_PATH "${CMAKE_CURRENT_BINARY_DIR}/v5l_fixture.v5l")
    set(V5L_EMPTY_FIXTURE_PATH
        "${CMAKE_CURRENT_BINARY_DIR}/v5l_empty_fixture.v5l")
    add_test(NAME v5l_fixture_generate
             COMMAND v5l_fixture "${V5L_FIXTURE_PATH}")
    add_test(NAME v5l_empty_fixture_generate
             COMMAND v5l_fixture --empty "${V5L_EMPTY_FIXTURE_PATH}")
    add_test(NAME v5l_inspect_valid
             COMMAND v5l_inspect --json "${V5L_FIXTURE_PATH}")
    add_test(NAME v5l_inspect_rejects_empty
             COMMAND ${CMAKE_COMMAND}
                     "-DINSPECTOR=$<TARGET_FILE:v5l_inspect>"
                     "-DRECORDING=${V5L_EMPTY_FIXTURE_PATH}"
                     -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/expect_v5l_reject.cmake")
    set_tests_properties(v5l_inspect_valid PROPERTIES
        DEPENDS v5l_fixture_generate
        PASS_REGULAR_EXPRESSION "\"status\":\"PASS\"")
    set_tests_properties(v5l_inspect_rejects_empty PROPERTIES
        DEPENDS v5l_empty_fixture_generate)
endif()
