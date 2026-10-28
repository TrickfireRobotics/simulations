#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"

namespace trickfire {

/// Everything in this header describes a *resolved* run: the Python CLI has already
/// merged scenario, rover, wheel, actuator, battery, soil and terrain files into one
/// manifest and written it next to the results. The simulator reads that manifest and
/// nothing else, so a run can always be reproduced from its own output directory.
///
/// Values the authored configs may leave as REPLACE_WITH_* placeholders are rejected by
/// config/load.cpp before the scene is built. See docs/chrono/development-plan.mdx.

// ---------------------------------------------------------------------------- //
//                                   geometry                                   //
// ---------------------------------------------------------------------------- //

/// A rigid-body collision primitive, expressed in its parent body's reference frame.
struct CollisionShape {
    enum class Type { BOX, SPHERE, CYLINDER, MESH, CONVEX_HULL };

    Type type = Type::BOX;
    chrono::ChVector3d position{0, 0, 0};
    chrono::ChQuaternion<> rotation = chrono::QUNIT;

    chrono::ChVector3d dimensions{0, 0, 0};  // box: full extents
    double radius = 0;                       // sphere, cylinder
    double length = 0;                       // cylinder
    std::string mesh_file;                   // mesh, convex_hull
    double mesh_margin = 0.01;               // collision envelope for mesh shapes
};

/// A visual mesh attached to a body, expressed in that body's reference frame.
struct VisualMesh {
    std::string file;
    chrono::ChVector3d position{0, 0, 0};
    chrono::ChQuaternion<> rotation = chrono::QUNIT;
    double scale = 1.0;
    std::array<float, 3> color{0.6f, 0.6f, 0.6f};
};

/// Mass, centre of mass and the full inertia tensor about the centre of mass.
struct MassProperties {
    double mass = 0;
    chrono::ChVector3d com{0, 0, 0};
    chrono::ChVector3d inertia_xx{0, 0, 0};  // Ixx, Iyy, Izz
    chrono::ChVector3d inertia_xy{0, 0, 0};  // Ixy, Ixz, Iyz

    /// Where the numbers came from: "measured", "cad", "estimated" or "placeholder".
    /// Scenarios can require a minimum provenance (see ScenarioSpec::require_provenance).
    std::string provenance = "estimated";
};

// ---------------------------------------------------------------------------- //
//                                     wheel                                    //
// ---------------------------------------------------------------------------- //

/// One immutable wheel package, as resolved from data/wheels/<name>/wheel.yaml.
struct WheelSpec {
    std::string name;
    std::string revision;
    std::string content_hash;

    double radius = 0;  // m, to the outer tread surface
    double width = 0;   // m

    MassProperties mass_properties;

    VisualMesh visual;
    /// Terrain-contact geometry. Normally one MESH shape preserving grousers, but a
    /// cylinder is allowed for smooth-wheel baselines and regression tests.
    std::vector<CollisionShape> collision;

    /// Roll axis in the package's own frame, after import normalisation. Always one of
    /// the cardinal axes - the importer refuses anything else rather than guessing.
    chrono::ChVector3d roll_axis{0, 1, 0};
};

// ---------------------------------------------------------------------------- //
//                                  powertrain                                  //
// ---------------------------------------------------------------------------- //

/// Command shaping between the driver's request and the controller.
struct CommandPathSpec {
    enum class Mode { VELOCITY, TORQUE };

    Mode mode = Mode::VELOCITY;
    double rate_limit = 0;      // request units/s, 0 = unlimited
    double delay_s = 0;         // transport delay on the request
    double update_rate_hz = 0;  // 0 = continuous; otherwise the command is sampled
};

/// Controller saturation and PID gains. In velocity mode the controller closes the loop
/// on output shaft speed and produces a torque request.
struct ControllerSpec {
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double integrator_limit = 0;  // anti-windup clamp on the integral term, 0 = off
    double torque_limit_nm = 0;   // output-side request clamp, 0 = motor limits only
};

/// Electromechanical motor and gearbox parameters, all on the motor side except where
/// the name says otherwise.
struct MotorSpec {
    double torque_constant = 0;      // Nm/A, motor side
    double back_emf_constant = 0;    // V/(rad/s), motor side; 0 means "same as kt"
    double resistance_ohm = 0;       // phase resistance
    double rotor_inertia = 0;        // kg m^2, motor side
    double gear_ratio = 1;           // motor revs per output rev
    double gear_efficiency = 1;      // 0..1, forward power flow
    double backlash_rad = 0;         // output-side lost motion
    double speed_limit_rad_s = 0;    // output side, 0 = unlimited
    double viscous_damping = 0;      // Nm/(rad/s), motor side
    double coulomb_friction_nm = 0;  // motor side
};

/// Current, voltage and temperature protection, as configured on the real controller.
struct ProtectionSpec {
    double continuous_current_a = 0;
    double peak_current_a = 0;
    double peak_duration_s = 0;
    double bus_voltage_limit_v = 0;

    enum class Cutoff { DERATE, FOLD_BACK, DISABLE };
    Cutoff cutoff = Cutoff::DERATE;
};

/// Lumped single-node motor/controller thermal model.
struct ThermalSpec {
    double ambient_c = 25;
    double thermal_resistance_c_per_w = 0;  // junction to ambient
    double thermal_capacitance_j_per_c = 0;
    double derate_start_c = 0;
    double cutoff_c = 0;
};

struct ActuatorSpec {
    std::string name;
    std::string manufacturer;
    std::string model;
    std::string hardware_revision;
    std::string firmware;

    CommandPathSpec command;
    ControllerSpec controller;
    MotorSpec motor;
    ProtectionSpec protection;
    ThermalSpec thermal;

    std::string provenance = "placeholder";
};

/// Pack-level electrical model shared by every drive.
struct BatterySpec {
    std::string name;

    /// Open-circuit voltage against state of charge, as (soc, volts) pairs sorted by soc.
    /// A single pair means a constant-voltage source.
    std::vector<std::pair<double, double>> ocv_curve;

    double capacity_ah = 0;
    double internal_resistance_ohm = 0;
    double wiring_resistance_ohm = 0;
    double current_limit_a = 0;
    double auxiliary_load_w = 0;
    double initial_soc = 1.0;

    /// Whether current pushed back by a regenerating drive is accepted by the pack.
    bool accept_regen = false;
    double regen_current_limit_a = 0;

    std::string provenance = "placeholder";
};

// ---------------------------------------------------------------------------- //
//                                     rover                                    //
// ---------------------------------------------------------------------------- //

/// A passive revolute suspension pivot: a rocker on the chassis, or a bogie on a rocker.
struct PivotSpec {
    std::string name;
    std::string parent;  // body name this pivot attaches to

    chrono::ChVector3d position{0, 0, 0};  // in the chassis reference frame
    chrono::ChVector3d axis{0, 1, 0};      // in the chassis reference frame

    /// Mechanical travel stops. Left unset when the hardware has none.
    std::optional<std::pair<double, double>> limits_rad;

    double spring_stiffness = 0;  // Nm/rad about the pivot, 0 = free
    double spring_damping = 0;    // Nm/(rad/s)
    double rest_angle_rad = 0;

    MassProperties mass_properties;
    std::vector<VisualMesh> visual;
    std::vector<CollisionShape> collision;
};

/// How the two rockers are coupled, if at all.
struct DifferentialSpec {
    enum class Type {
        /// Rockers move independently. Use this when the hardware has no differential.
        NONE,
        /// Kinematic constraint forcing equal and opposite rocker rotation. Only valid if
        /// the hardware actually behaves this way.
        KINEMATIC,
        /// Explicit bar-and-links mechanism, as on Curiosity.
        LINKAGE,
    };

    Type type = Type::NONE;

    // LINKAGE only
    chrono::ChVector3d bar_position{0, 0, 0};
    chrono::ChVector3d left_link_position{0, 0, 0};
    chrono::ChVector3d right_link_position{0, 0, 0};
    MassProperties bar_mass_properties;
    MassProperties link_mass_properties;
};

/// One driven wheel station.
struct WheelStationSpec {
    std::string id;      // "fl", "ml", "bl", "fr", "mr", "br"
    std::string parent;  // suspension body the wheel hangs off

    chrono::ChVector3d position{0, 0, 0};  // hub centre, chassis reference frame
    chrono::ChVector3d axis{0, 1, 0};      // spin axis, chassis reference frame

    std::string wheel_package;  // WheelSpec name
    std::string actuator;       // ActuatorSpec name

    /// Extra rotation of the wheel package about its own roll axis, for packages whose
    /// tread is handed.
    double mount_roll_rad = 0;
    /// True on stations where the package must be mirrored to face outward.
    bool mirror = false;
};

struct RoverSpec {
    std::string name;
    std::string source_urdf;  // provenance: what this was generated from
    std::string source_urdf_hash;

    MassProperties chassis;
    std::vector<VisualMesh> chassis_visual;
    std::vector<CollisionShape> chassis_collision;

    std::vector<PivotSpec> pivots;
    DifferentialSpec differential;
    std::vector<WheelStationSpec> wheels;

    /// Sliding friction, restitution and SMC stiffness for everything but the wheels,
    /// which take their material from the terrain spec.
    double body_friction = 0.8;
    double body_restitution = 0.0;
};

// ---------------------------------------------------------------------------- //
//                                    terrain                                   //
// ---------------------------------------------------------------------------- //

/// One Bekker / Mohr-Coulomb / Janosi-Hanamoto soil definition.
struct SoilSpec {
    std::string name;
    double bekker_kphi = 0;
    double bekker_kc = 0;
    double bekker_n = 1;
    double mohr_cohesion = 0;  // Pa
    double mohr_friction = 0;  // degrees
    double janosi_shear = 0;   // m
    double elastic_k = 0;      // Pa/m
    double damping_r = 0;      // Pa s/m

    std::string provenance = "placeholder";
};

/// Rectangular region of the SCM patch that uses a different soil. Regions are tested in
/// order and the first match wins, so later entries act as the background.
struct SoilRegion {
    double x_min = 0, x_max = 0;
    double y_min = 0, y_max = 0;
    std::string soil;  // SoilSpec name
};

/// Rigid obstacle placed on the terrain. Rock fields are expanded by the CLI into
/// explicit entries so that a manifest fully determines the scene.
struct ObstacleSpec {
    CollisionShape shape;
    chrono::ChVector3d position{0, 0, 0};
    chrono::ChQuaternion<> rotation = chrono::QUNIT;
    bool fixed = true;
    double mass = 0;  // only used when !fixed
    double friction = 0.9;
};

struct TerrainSpec {
    /// The fidelity ladder from the development plan. CRM/DEM is deliberately absent:
    /// those studies run through separate Chrono demos until tier 4 is scoped.
    enum class Type {
        RIGID_FLAT,     // tier 0
        RIGID_MESH,     // tier 1, procedural or surveyed geometry
        SCM_FLAT,       // tier 2
        SCM_HEIGHTMAP,  // tier 2
        SCM_MESH,       // tier 2
        MIXED,          // tier 3, SCM patch plus rigid regions
    };

    Type type = Type::SCM_FLAT;
    std::string name;
    int version = 1;

    double size_x = 10;
    double size_y = 4;
    double grid_delta = 0.04;

    /// SCM_HEIGHTMAP only: greyscale image plus the height range it spans.
    std::string heightmap_file;
    double height_min = 0;
    double height_max = 1;

    /// RIGID_MESH / SCM_MESH only.
    std::string mesh_file;

    /// Slope of the whole patch, applied by tilting the terrain reference frame.
    double pitch_rad = 0;  // nose-up positive
    double roll_rad = 0;   // side slope

    std::vector<SoilSpec> soils;
    std::vector<SoilRegion> regions;
    std::string default_soil;

    bool bulldozing = true;
    chrono::ChVector3d bulldozing_params{55, 1, 5};  // erosion angle, flow factor, iterations
    int bulldozing_erosion_iters = 6;

    /// Restrict SCM ray casting to boxes that follow each wheel.
    bool active_domains = true;
    double active_domain_margin = 0.25;  // m added around each wheel's bounding box

    double rigid_friction = 0.9;
    double rigid_restitution = 0.0;

    std::vector<ObstacleSpec> obstacles;

    /// Provenance for terrain built from survey data.
    std::string source_file;
    std::string source_hash;
    std::string source_crs;
};

// ---------------------------------------------------------------------------- //
//                                   scenario                                   //
// ---------------------------------------------------------------------------- //

/// Driver command profile. All of these produce a per-wheel request in the units the
/// actuator's command path expects.
struct DriverSpec {
    enum class Type {
        CONSTANT,    // hold `value` after the ramp
        RAMP,        // linear from 0 to `value` over `ramp_time`
        SLIP_SWEEP,  // step through a list of commanded speeds, holding each
        REPLAY,      // commands recorded from hardware
        STOPPED,     // zero command, for drop and settling tests
    };

    Type type = Type::CONSTANT;
    double value = 0;  // rad/s or Nm depending on the actuator command mode
    double ramp_time = 1.0;
    double steering = 0;  // skid-steer bias: +1 turns right, -1 left, 0 straight

    std::vector<double> sweep_values;
    double sweep_hold_s = 5.0;

    std::string replay_file;  // CSV: time, then one column per wheel id
};

/// One reason a run may stop early. Every run also stops at ScenarioSpec::end_time.
struct TerminationSpec {
    enum class Type { DISTANCE, ROLLOVER, IMMOBILITY, SINKAGE, MOTOR_TEMPERATURE, BATTERY_EMPTY };

    Type type = Type::DISTANCE;
    double threshold = 0;
    double window_s = 0;  // for IMMOBILITY: how long progress must stay below threshold
    bool pass = true;     // whether hitting this condition counts as a pass
};

/// A pass/fail assertion evaluated against the summary metrics once the run ends.
struct CheckSpec {
    std::string metric;
    enum class Op { LT, LE, GT, GE } op = Op::LT;
    double value = 0;
    std::string description;
};

struct ScenarioSpec {
    std::string name;
    int version = 1;

    chrono::ChVector3d initial_position{0, 0, 0};
    double initial_yaw_rad = 0;
    /// Drop height above the terrain surface at the initial position.
    double drop_height = 0.05;

    double end_time = 20.0;
    double warmup_time = 0.5;  // excluded from summary metrics
    double sample_rate_hz = 200;

    DriverSpec driver;
    std::vector<TerminationSpec> termination;
    std::vector<CheckSpec> checks;

    /// Metric groups to record. Empty means all of them.
    std::vector<std::string> metric_groups;

    /// Refuse to run if any mass property, soil or actuator in the manifest has a weaker
    /// provenance than this. "placeholder" disables the check.
    std::string require_provenance = "placeholder";
};

// ---------------------------------------------------------------------------- //
//                                   manifest                                   //
// ---------------------------------------------------------------------------- //

struct SimulationSpec {
    double step_size = 2e-3;
    int threads = 4;

    enum class ContactMethod { SMC, NSC };
    ContactMethod contact_method = ContactMethod::SMC;

    /// Vertical axis of the world frame. Z matches URDF, SCM's native frame and every
    /// Chrono vehicle demo; Y is kept only for the original single-wheel prototype.
    enum class UpAxis { Y, Z };
    UpAxis up_axis = UpAxis::Z;

    double gravity = 9.81;

    /// Iterations the VI solver is allowed per step. The default is well above Chrono's
    /// own because the rocker-bogie differential is a constraint the solver has to
    /// actually converge on, not merely approach.
    int solver_iterations = 200;
};

struct RunSpec {
    std::string id;
    std::string study;
    std::uint64_t seed = 0;
    std::string manifest_path;
    std::string output_dir;
    bool headless = true;
};

/// The whole resolved run. Produced by `sim chrono` and consumed by the simulator.
struct RunManifest {
    int schema_version = 0;

    RunSpec run;
    SimulationSpec simulation;
    RoverSpec rover;
    TerrainSpec terrain;
    ScenarioSpec scenario;

    std::map<std::string, WheelSpec> wheels;
    std::map<std::string, ActuatorSpec> actuators;
    BatterySpec battery;

    /// Resolve helpers. These throw std::runtime_error naming the missing key, which is
    /// what load.cpp's validation pass relies on to report every dangling reference.
    const WheelSpec& Wheel(const std::string& name) const;
    const ActuatorSpec& Actuator(const std::string& name) const;
    const SoilSpec& Soil(const std::string& name) const;
};

/// Schema version this build understands. A manifest written by a newer CLI is rejected
/// rather than partially read.
inline constexpr int kManifestSchemaVersion = 1;

}  // namespace trickfire
