cmake_minimum_required(VERSION 3.18)
project(trickfire_sim CXX)

set(CHRONO_DIR "/home/trickfire/chrono/build/cmake"
    CACHE PATH "Path to Chrono's cmake/ directory inside the build tree")

option(TRICKFIRE_BUILD_GUI "Build the VSG/ImGui viewer. Off builds a headless-only binary." ON)
option(TRICKFIRE_BUILD_TESTS "Build the physics and configuration unit tests" ON)

if(TRICKFIRE_BUILD_GUI)
    find_package(Chrono COMPONENTS Vehicle VSG HINTS "${CHRONO_DIR}" REQUIRED)
else()
    find_package(Chrono COMPONENTS Vehicle HINTS "${CHRONO_DIR}" REQUIRED)
endif()

# Everything except the entry points and the viewer. Both the simulator and the test
# binaries link this, so a physics unit test exercises exactly the shipped code.
add_library(sim_core STATIC
    src/config/load.cpp
    src/experiments/driver.cpp
    src/experiments/metrics.cpp
    src/experiments/termination.cpp
    src/powertrain/actuator.cpp
    src/powertrain/battery.cpp
    src/powertrain/drivetrain.cpp
    src/reporting/environment.cpp
    src/reporting/telemetry.cpp
    src/rover/geometry.cpp
    src/rover/rover.cpp
    src/simulation.cpp
    src/terrain/terrain.cpp
)

target_include_directories(sim_core PUBLIC src)

target_compile_definitions(sim_core PUBLIC
    CHRONO_DATA_PATH="${CMAKE_SOURCE_DIR}/data/"
    CHRONO_VEHICLE_DATA_PATH="${CHRONO_VEHICLE_DATA_DIR}"
    TRICKFIRE_CHRONO_SOURCE_DIR="${CHRONO_PACKAGE_PREFIX_DIR}"
)

target_link_libraries(sim_core PUBLIC
    Chrono::Chrono_core
    Chrono::Chrono_vehicle
)

set_target_properties(sim_core PROPERTIES CXX_STANDARD 17)

add_executable(sim src/app/main.cpp)
target_link_libraries(sim PRIVATE sim_core)
set_target_properties(sim PROPERTIES CXX_STANDARD 17)

if(TRICKFIRE_BUILD_GUI)
    # Chrono::Chrono_vsg carries vsg, vsgXchange and vsgImGui as interface dependencies,
    # so the ImGui headers used by the GUI panel come along with it.
    target_sources(sim PRIVATE
        src/gui/orbit_camera.cpp
        src/gui/sim_gui.cpp
        src/gui/viewer.cpp
        src/gui/vis_system.cpp
    )
    target_compile_definitions(sim PRIVATE
        TRICKFIRE_HAS_GUI=1
        TRICKFIRE_LOGO_PATH="${CMAKE_SOURCE_DIR}/../assets/trickfire-logo.png"
    )
    target_link_libraries(sim PRIVATE
        Chrono::Chrono_vsg
        Chrono::Chrono_vehicle_vsg
    )
endif()

if(TRICKFIRE_BUILD_TESTS)
    enable_testing()

    # Hand-rolled assertions rather than GoogleTest: Chrono bundles gtest but does not
    # export it, and these tests only need "check a number against a tolerance".
    foreach(test_name config physics powertrain metrics)
        add_executable(test_${test_name} tests/test_${test_name}.cpp)
        target_include_directories(test_${test_name} PRIVATE tests)
        target_link_libraries(test_${test_name} PRIVATE sim_core)
        set_target_properties(test_${test_name} PROPERTIES CXX_STANDARD 17)
        add_test(NAME ${test_name} COMMAND test_${test_name})
        set_tests_properties(${test_name} PROPERTIES
            ENVIRONMENT "TRICKFIRE_TEST_DATA=${CMAKE_SOURCE_DIR}/tests/data")
    endforeach()
endif()
