cmake_minimum_required(VERSION 3.18)
project(trickfire_sim CXX)

set(CHRONO_DIR "/home/trickfire/chrono/build/cmake"
    CACHE PATH "Path to Chrono's cmake/ directory inside the build tree")

find_package(Chrono
    COMPONENTS Vehicle VSG
    HINTS "${CHRONO_DIR}"
    REQUIRED
)

add_executable(sim
    src/main.cpp
    src/orbit_camera.cpp
    src/scm_scene.cpp
    src/sim_config.cpp
    src/sim_gui.cpp
    src/vis_system.cpp
)

target_include_directories(sim PRIVATE src)

target_compile_definitions(sim PRIVATE
    CHRONO_DATA_PATH="${CMAKE_SOURCE_DIR}/data/"
    CHRONO_VEHICLE_DATA_PATH="${CHRONO_VEHICLE_DATA_DIR}"
    TRICKFIRE_LOGO_PATH="${CMAKE_SOURCE_DIR}/../assets/trickfire-logo.png"
)

# Chrono::Chrono_vsg carries vsg, vsgXchange and vsgImGui as interface dependencies,
# so the ImGui headers used by the GUI panel come along with it.
target_link_libraries(sim
    Chrono::Chrono_core
    Chrono::Chrono_vehicle
    Chrono::Chrono_vsg
    Chrono::Chrono_vehicle_vsg
)

set_target_properties(sim PROPERTIES CXX_STANDARD 17)
