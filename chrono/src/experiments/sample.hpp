#pragma once

#include <string>
#include <vector>

#include "chrono/core/ChVector3.h"

#include "powertrain/actuator.hpp"
#include "powertrain/battery.hpp"

namespace trickfire {

/// Per-wheel channels for one instant.
struct WheelSample {
    std::string id;

    double angle = 0;      // rad, accumulated
    double speed = 0;      // rad/s about the spin axis
    double commanded = 0;  // driver request, in the actuator's command units

    double longitudinal_slip = 0;  // 1 - v_x / (omega * r) when driving, clamped to [-1, 1]
    double lateral_slip = 0;       // atan(v_y / |v_x|), radians
    double sinkage = 0;            // m, positive down

    chrono::ChVector3d contact_force{0, 0, 0};   // world frame, N
    chrono::ChVector3d contact_torque{0, 0, 0};  // world frame, Nm
    double normal_load = 0;                      // component along the terrain normal
    double drawbar_pull = 0;                     // component along the travel direction

    ActuatorState drive;
};

/// Suspension channels for one instant.
struct PivotSample {
    std::string name;
    double angle = 0;            // rad
    double velocity = 0;         // rad/s
    double reaction_torque = 0;  // Nm
};

/// Everything recorded at one instant. Metrics, termination rules and the telemetry
/// writer all read this, so a new channel is added in exactly one place.
struct Sample {
    double time = 0;

    // Mobility
    chrono::ChVector3d position{0, 0, 0};
    chrono::ChVector3d velocity{0, 0, 0};
    double speed = 0;              // magnitude of the horizontal velocity
    double distance = 0;           // path length travelled since the warm-up ended
    double straight_distance = 0;  // distance from the start position
    double path_error = 0;         // lateral deviation from the commanded heading
    double grade = 0;              // local terrain slope under the rover, radians

    // Suspension and stability
    double chassis_roll = 0;
    double chassis_pitch = 0;
    double roll_rate = 0;
    double pitch_rate = 0;
    double ground_clearance = 0;
    int wheels_in_contact = 0;
    double stability_margin = 0;  // smallest normal load as a fraction of the mean

    std::vector<WheelSample> wheels;
    std::vector<PivotSample> pivots;
    BatteryState battery;

    // Numerics
    double step_size = 0;
    double real_time_factor = 0;
    int solver_iterations = 0;
    int contact_count = 0;
    int scm_ray_casts = 0;
    int scm_ray_hits = 0;
};

}  // namespace trickfire
