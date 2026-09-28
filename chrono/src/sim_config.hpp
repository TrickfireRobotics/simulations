#pragma once

#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace trickfire {

enum class TireType { CYLINDRICAL, LUGGED };

/// Startup parameters for the SCM wheel-soil simulation.
/// Anything that can also be changed at run time from the GUI lives in SimState instead.
struct SimConfig {
    // --- wheel ---
    TireType tire_type = TireType::LUGGED;
    double tire_radius = 0.8;
    double wheel_mass = 500;
    double wheel_inertia = 20;
    double cylinder_radius = 0.5;
    double cylinder_width = 0.4;

    // --- terrain ---
    double terrain_length = 6;
    double terrain_width = 2;
    double mesh_resolution = 0.04;
    bool enable_active_domains = true;

    // When true, soil properties vary by position via the soil parameters callback.
    // When false, the uniform `soft_soil` values below are used everywhere.
    bool variable_soil = true;

    // --- solver ---
    int threads = 4;

    // --- camera ---
    // Also the home view that the GUI's "reset view" button returns to
    chrono::ChVector3d camera_eye{3.0, 2.0, 0.0};
    chrono::ChVector3d camera_target{0.0, 0.8, 0.0};

    // --- window ---
    int window_width = 1280;
    int window_height = 800;

    // Multiplier for every GUI dimension. VSG renders into the framebuffer's physical
    // pixels and neither it nor Chrono looks up the display scale, so on a HiDPI screen
    // the panels come out tiny. Overridden by TRICKFIRE_UI_SCALE.
    float ui_scale = 1.0f;
};

/// Read TRICKFIRE_UI_SCALE, falling back to the given default if it is unset or invalid.
float ReadUiScale(float fallback = 1.0f);

/// Soil parameters for one Bekker/Janosi-Hanamoto soil definition.
struct SoilParams {
    double bekker_Kphi;
    double bekker_Kc;
    double bekker_n;
    double mohr_cohesion;  // Pa
    double mohr_friction;  // degrees
    double janosi_shear;   // m
    double elastic_K;      // Pa/m
    double damping_R;      // Pa*s/m
};

inline constexpr SoilParams kSoftSoil{0.2e6, 0, 1.1, 0, 30, 0.01, 4e7, 3e4};
inline constexpr SoilParams kLeteSand{5301e3, 102e3, 0.793, 1.3e3, 31.1, 1.2e-2, 4e8, 3e4};

/// Mutable state shared between the run loop and the GUI panel.
struct SimState {
    bool paused = false;
    bool step_once = false;
    bool reset_requested = false;

    double step_size = 2e-3;
    double target_render_fps = 0;  // 0 = render every simulation step

    double wheel_speed = chrono::CH_PI / 4;  // rad/s, driven by the motor
    bool follow_wheel = true;

    bool bulldozing = true;
    bool wireframe = true;
    bool active_domains_visible = false;
};

}  // namespace trickfire
