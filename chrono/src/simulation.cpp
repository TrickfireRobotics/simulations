#include "simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/core/ChRandom.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChIterativeSolverVI.h"

#include "reporting/environment.hpp"

using namespace chrono;

namespace trickfire {

namespace fs = std::filesystem;

namespace {

double WallClockSeconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

}  // namespace

Simulation::Simulation(RunManifest manifest) : m_manifest(std::move(manifest)) {
    m_up = m_manifest.simulation.up_axis == SimulationSpec::UpAxis::Z ? ChVector3d(0, 0, 1)
                                                                      : ChVector3d(0, 1, 0);
    BuildSystem();

    m_terrain = std::make_unique<Terrain>(*m_sys, m_manifest);
    m_rover = std::make_unique<Rover>(*m_sys, m_manifest);

    std::vector<std::pair<std::string, std::string>> wheel_actuators;
    std::vector<std::string> wheel_ids;
    for (const auto& station : m_manifest.rover.wheels) {
        wheel_actuators.emplace_back(station.id, station.actuator);
        wheel_ids.push_back(station.id);
    }
    m_drivetrain = std::make_unique<Drivetrain>(
        m_manifest.actuators, wheel_actuators, m_manifest.battery, m_manifest.simulation.step_size);
    m_driver = std::make_unique<Driver>(m_manifest.scenario.driver, wheel_ids);

    std::vector<std::string> pivot_names;
    for (const auto& pivot : m_manifest.rover.pivots) pivot_names.push_back(pivot.name);

    m_metrics = std::make_unique<Metrics>(m_manifest.scenario, wheel_ids, pivot_names);
    m_monitor = std::make_unique<TerminationMonitor>(m_manifest.scenario.termination,
                                                     m_manifest.scenario.end_time);

    m_sample_period = m_manifest.scenario.sample_rate_hz > 0
                          ? 1.0 / m_manifest.scenario.sample_rate_hz
                          : m_manifest.simulation.step_size;
}

Simulation::~Simulation() {
    try {
        Finish();
    } catch (const std::exception&) {
        // A destructor cannot usefully report a failure to flush, and throwing here would
        // mask whatever exception was already unwinding.
    }
}

void Simulation::BuildSystem() {
    // Seeding Chrono's generator makes a run with a rock field or any sampled soil
    // reproducible. Scenarios without randomness are unaffected.
    ChRandom::SetSeed(static_cast<double>(m_manifest.run.seed));

    if (m_manifest.simulation.contact_method == SimulationSpec::ContactMethod::NSC) {
        m_sys = std::make_unique<ChSystemNSC>();
    } else {
        m_sys = std::make_unique<ChSystemSMC>();
    }

    const double g = m_manifest.simulation.gravity;
    m_sys->SetGravitationalAcceleration(-m_up * g);

    // SCM's ray casting is Bullet-based, and the development plan's terrain tiers all
    // assume it, so the collision system is pinned rather than left to the default.
    m_sys->SetCollisionSystem(chrono_types::make_shared<ChCollisionSystemBullet>());

    const int threads = m_manifest.simulation.threads;
    m_sys->SetNumThreads(threads, 2 * threads, 1);

    // Pin the solver iteration count rather than taking Chrono's default. The rover's
    // suspension carries a differential constraint that an under-iterated VI solver
    // simply does not enforce, and silently so: the rockers drift to their travel stops
    // and every load and attitude number becomes wrong. This also makes the solver
    // effort part of the run's recorded environment rather than a property of whichever
    // Chrono version is installed.
    if (auto iterative = std::dynamic_pointer_cast<ChIterativeSolverVI>(m_sys->GetSolver())) {
        iterative->SetMaxIterations(m_manifest.simulation.solver_iterations);
        iterative->EnableWarmStart(true);
    }
}

void Simulation::PlaceRover() {
    const auto& scenario = m_manifest.scenario;

    // Start height comes from the terrain, not the scenario, so the same scenario works
    // on a flat patch and on a height map without being re-authored.
    ChVector3d start = scenario.initial_position;
    const double surface = m_terrain->InitialHeight(start);

    double lowest_hub = std::numeric_limits<double>::infinity();
    double largest_radius = 0;
    for (const auto& station : m_manifest.rover.wheels) {
        const WheelSpec& wheel = m_manifest.Wheel(station.wheel_package);
        lowest_hub = std::min(lowest_hub, Vdot(station.position, m_up));
        largest_radius = std::max(largest_radius, wheel.radius);
    }
    if (!std::isfinite(lowest_hub)) lowest_hub = 0;

    // Lift the chassis so the lowest wheel's tread starts `drop_height` above the surface.
    const double lift = surface + largest_radius + scenario.drop_height - lowest_hub;
    start += m_up * (lift - Vdot(start, m_up));

    const ChQuaternion<> yaw = QuatFromAngleAxis(scenario.initial_yaw_rad, m_up);

    // Square the rover to the slope, so a side-slope scenario does not spend its first
    // second rolling into contact.
    ChQuaternion<> level = QUNIT;
    const ChVector3d normal = m_terrain->SurfaceNormal();
    const double alignment = Vdot(normal, m_up);
    if (alignment < 1 - 1e-9) {
        const ChVector3d axis = Vcross(m_up, normal);
        if (axis.Length() > 1e-9) {
            level = QuatFromAngleAxis(std::acos(std::clamp(alignment, -1.0, 1.0)),
                                      axis.GetNormalized());
        }
    }

    m_initial_pose = ChFrame<>(start, level * yaw);
    m_rover->Initialize(m_initial_pose);

    m_heading = m_initial_pose.GetRotMat().GetAxisX();
    m_last_position = m_rover->Chassis()->GetFrameRefToAbs().GetPos();

    std::vector<std::shared_ptr<ChBody>> wheel_bodies;
    std::vector<double> radii;
    std::vector<double> widths;
    for (const auto& station : m_manifest.rover.wheels) {
        const WheelSpec& wheel = m_manifest.Wheel(station.wheel_package);
        radii.push_back(wheel.radius);
        widths.push_back(wheel.width);
    }
    for (const auto& wheel : m_rover->Wheels()) wheel_bodies.push_back(wheel.body);
    m_terrain->TrackWheels(wheel_bodies, radii, widths);
}

void Simulation::Initialize() {
    if (m_initialized) return;

    PlaceRover();

    WriteEnvironment(m_manifest.run.output_dir, m_manifest);

    m_sample = BuildSample();
    m_telemetry =
        std::make_unique<TelemetryWriter>(m_manifest.run.output_dir, m_sample, *m_metrics);

    m_wall_clock_start = WallClockSeconds();
    m_last_wall_clock = m_wall_clock_start;
    m_last_sim_time = 0;
    m_next_sample_time = 0;
    m_initialized = true;
}

void Simulation::Reset() {
    m_terrain->Reset();
    m_drivetrain->Reset();
    m_driver->Reset();
    m_monitor->Reset();

    m_rover->Chassis()->SetFrameRefToAbs(m_initial_pose);
    for (const auto& body : m_rover->Bodies()) {
        body->SetLinVel(VNULL);
        body->SetAngVelLocal(VNULL);
    }

    m_sys->SetChTime(0);
    m_distance = 0;
    m_last_position = m_initial_pose.GetPos();
    m_next_sample_time = 0;
    m_termination = Termination{};

    if (m_telemetry) {
        m_telemetry->WriteEvent({0, "reset", "", "state restored from the viewer"});
    }
}

bool Simulation::Step() {
    if (!m_initialized) Initialize();
    if (Finished()) return false;

    const double step = m_manifest.simulation.step_size;
    const double time = m_sys->GetChTime();

    // Command, then powertrain, then physics. The drivetrain sees the wheel speeds from
    // the end of the previous step, which is the same one-step lag a real controller has.
    m_last_requests = m_driver->Advance(time);
    const auto& requests = m_last_requests;
    const auto& speeds = m_rover->WheelSpeeds();
    const auto& torques = m_drivetrain->Advance(time, requests, speeds);
    m_rover->ApplyWheelTorques(torques);

    m_sys->DoStepDynamics(step);
    m_terrain->Advance(step);

    const double now = m_sys->GetChTime();
    if (now + 1e-12 >= m_next_sample_time) {
        m_sample = BuildSample();
        m_metrics->Add(m_sample);
        m_telemetry->WriteSample(m_sample);
        m_telemetry->DetectEvents(m_sample);
        m_next_sample_time = now + m_sample_period;

        m_termination = m_monitor->Check(m_sample);
        if (Finished()) {
            m_telemetry->WriteEvent(
                {m_sample.time, "termination", m_termination.rule, m_termination.detail});
            Finish();
            return false;
        }
    }
    return true;
}

void Simulation::Run() {
    Initialize();
    while (Step()) {
    }
}

void Simulation::Finish() {
    if (m_finished || !m_telemetry) return;
    m_finished = true;

    const auto summary = m_metrics->Summary();
    const auto checks = m_metrics->EvaluateChecks(m_manifest.scenario.checks);
    m_telemetry->WriteSummary(summary, checks, m_termination,
                              WallClockSeconds() - m_wall_clock_start);
}

bool Simulation::Passed() const {
    if (!m_metrics) return false;
    return TelemetryWriter::Passed(m_metrics->EvaluateChecks(m_manifest.scenario.checks),
                                   m_termination);
}

// ---------------------------------------------------------------------------- //
//                                   sampling                                   //
// ---------------------------------------------------------------------------- //

Sample Simulation::BuildSample() {
    Sample sample;
    sample.time = m_sys->GetChTime();
    sample.step_size = m_manifest.simulation.step_size;

    const auto& chassis = m_rover->Chassis();
    sample.position = chassis->GetFrameRefToAbs().GetPos();
    sample.velocity = chassis->GetPosDt();

    // Path length in the ground plane, accumulated once per sample rather than once per
    // step. Each delta is unsigned, so accumulating them at the step rate adds the
    // magnitude of the vertical settling and of per-step solver jitter - enough to
    // overstate a short run by several per cent. Sampling also means a reader can
    // recompute this column from the positions in telemetry.csv and get the same number.
    const ChVector3d step = sample.position - m_last_position;
    m_distance += (step - m_up * Vdot(step, m_up)).Length();
    m_last_position = sample.position;

    // Horizontal speed: the component in the ground plane, so a rover dropping into a
    // trench does not report that as progress.
    const ChVector3d horizontal = sample.velocity - m_up * Vdot(sample.velocity, m_up);
    sample.speed = horizontal.Length();
    sample.distance = m_distance;

    const ChVector3d travel = sample.position - m_initial_pose.GetPos();
    sample.straight_distance = (travel - m_up * Vdot(travel, m_up)).Length();

    // Path error is the lateral offset from the line the rover was pointed down at the
    // start, which is the only reference a skid-steer scenario has.
    const ChVector3d lateral = Vcross(m_up, m_heading).GetNormalized();
    sample.path_error = Vdot(travel, lateral);

    m_rover->ChassisAttitude(sample.chassis_roll, sample.chassis_pitch);

    const double dt = sample.time - m_last_sim_time;
    if (dt > 0) {
        sample.roll_rate = (sample.chassis_roll - m_previous_roll) / dt;
        sample.pitch_rate = (sample.chassis_pitch - m_previous_pitch) / dt;

        const double wall_now = WallClockSeconds();
        const double wall_dt = wall_now - m_last_wall_clock;
        sample.real_time_factor = wall_dt > 0 ? dt / wall_dt : 0;
        m_last_wall_clock = wall_now;
    }
    m_previous_roll = sample.chassis_roll;
    m_previous_pitch = sample.chassis_pitch;
    m_last_sim_time = sample.time;

    UpdateSampleGeometry(sample);

    sample.battery = m_drivetrain->Pack().State();
    sample.contact_count = static_cast<int>(m_sys->GetNumContacts());

    if (auto iterative = std::dynamic_pointer_cast<ChIterativeSolver>(m_sys->GetSolver())) {
        sample.solver_iterations = iterative->GetIterations();
    }

    const auto counters = m_terrain->Counters();
    sample.scm_ray_casts = counters.ray_casts;
    sample.scm_ray_hits = counters.ray_hits;

    return sample;
}

void Simulation::UpdateSampleGeometry(Sample& sample) {
    const ChVector3d normal = m_terrain->SurfaceNormal();
    const ChVector3d forward = m_rover->Chassis()->GetFrameRefToAbs().GetRotMat().GetAxisX();

    // Local grade: the slope of the surface along the direction of travel, measured over
    // one wheelbase so a single rock does not read as a hill.
    {
        const ChVector3d flat_forward = (forward - m_up * Vdot(forward, m_up)).GetNormalized();
        const double span = 1.0;
        const ChVector3d ahead = sample.position + flat_forward * span;
        const ChVector3d behind = sample.position - flat_forward * span;
        const double rise = m_terrain->Height(ahead) - m_terrain->Height(behind);
        sample.grade = std::atan2(rise, 2 * span);
    }

    sample.ground_clearance = m_rover->GroundClearance(m_terrain->Height(sample.position));

    const auto& wheels = m_rover->Wheels();
    const auto& requests = m_last_requests;

    double load_sum = 0;
    double min_load = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < wheels.size(); i++) {
        const auto& wheel = wheels[i];

        WheelSample out;
        out.id = wheel.id;
        out.angle = wheel.Angle();
        out.speed = wheel.Speed();
        out.commanded = i < requests.size() ? requests[i] : 0.0;
        out.drive = m_drivetrain->Drive(i).State();

        const ChVector3d hub = wheel.body->GetFrameRefToAbs().GetPos();
        const ChVector3d hub_velocity = wheel.body->GetPosDt();

        // Slip is measured along the wheel's own travel direction, taken from the hub's
        // velocity projected into the ground plane, so it stays meaningful in a turn.
        const ChVector3d travel_dir = (forward - m_up * Vdot(forward, m_up)).GetNormalized();
        const double v_long = Vdot(hub_velocity, travel_dir);
        const double v_lat = Vdot(hub_velocity, Vcross(m_up, travel_dir).GetNormalized());
        const double rolling = out.speed * wheel.radius;

        if (std::abs(rolling) > 1e-3 || std::abs(v_long) > 1e-3) {
            // Driving uses the wheel's own speed as the reference, braking the ground's,
            // which is the usual terramechanics convention and keeps slip in [-1, 1].
            const double reference = std::max(std::abs(rolling), std::abs(v_long));
            out.longitudinal_slip = std::clamp((rolling - v_long) / reference, -1.0, 1.0);
        }
        if (std::abs(v_long) > 1e-3) out.lateral_slip = std::atan2(v_lat, std::abs(v_long));

        out.sinkage = m_terrain->Sinkage(hub);
        m_terrain->ContactForce(wheel.body, out.contact_force, out.contact_torque);
        out.normal_load = Vdot(out.contact_force, normal);
        out.drawbar_pull = Vdot(out.contact_force, travel_dir);

        if (out.normal_load > 1e-3) sample.wheels_in_contact++;
        load_sum += std::max(0.0, out.normal_load);
        min_load = std::min(min_load, std::max(0.0, out.normal_load));

        sample.wheels.push_back(std::move(out));
    }

    // A tip indicator rather than a true support-polygon margin: the ratio of the least
    // loaded wheel to the average. 1 is perfectly even, 0 means a wheel carries nothing.
    const double mean_load = wheels.empty() ? 0 : load_sum / static_cast<double>(wheels.size());
    sample.stability_margin = mean_load > 1e-3 ? min_load / mean_load : 0.0;

    for (const auto& pivot : m_rover->Pivots()) {
        PivotSample out;
        out.name = pivot.name;
        out.angle = pivot.Angle();
        out.velocity = pivot.Velocity();
        out.reaction_torque = pivot.ReactionTorque();
        sample.pivots.push_back(std::move(out));
    }
}

}  // namespace trickfire
