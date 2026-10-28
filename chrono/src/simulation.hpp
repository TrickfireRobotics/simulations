#pragma once

#include <memory>
#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"

#include "config/spec.hpp"
#include "experiments/driver.hpp"
#include "experiments/metrics.hpp"
#include "experiments/sample.hpp"
#include "experiments/termination.hpp"
#include "powertrain/drivetrain.hpp"
#include "reporting/telemetry.hpp"
#include "rover/rover.hpp"
#include "terrain/terrain.hpp"

namespace trickfire {

/// One run, with no reference to any visualisation.
///
/// This is the split the development plan's Phase 0 asks for: the viewer attaches to a
/// Simulation and drives the same Step() the headless runner does, so the two cannot
/// drift apart and GUI state cannot change test conditions. Everything the viewer wants
/// to show is read back through the accessors rather than computed in the render loop.
class Simulation {
  public:
    explicit Simulation(RunManifest manifest);
    ~Simulation();

    /// Place the rover, settle the scene and open the output files.
    void Initialize();

    /// Advance one step. Returns false once the run has terminated.
    bool Step();

    /// Finish the run: write the summary and close the telemetry files. Safe to call
    /// more than once, and called by the destructor so an interrupted run still leaves
    /// a usable summary behind.
    void Finish();

    /// Run to termination. The headless entry point is this one line.
    void Run();

    bool Finished() const { return m_termination.reason != Termination::Reason::RUNNING; }
    const Termination& Result() const { return m_termination; }
    bool Passed() const;

    /// Restore the initial state without rebuilding the scene, for the viewer's reset.
    /// Telemetry already written is kept, and a reset event is logged, so a reset run is
    /// never mistaken for a clean one.
    void Reset();

    const RunManifest& Manifest() const { return m_manifest; }
    chrono::ChSystem& System() { return *m_sys; }
    Rover& RoverModel() { return *m_rover; }
    Terrain& TerrainModel() { return *m_terrain; }
    Drivetrain& Powertrain() { return *m_drivetrain; }
    const Sample& LatestSample() const { return m_sample; }
    const Metrics& MetricsAccumulator() const { return *m_metrics; }

    double Time() const { return m_sys->GetChTime(); }
    double RealTimeFactor() const { return m_sample.real_time_factor; }

    /// Step size in use. Fixed for now; kept as an accessor so an adaptive scheme can be
    /// added without touching the viewer.
    double StepSize() const { return m_manifest.simulation.step_size; }

  private:
    void BuildSystem();
    void PlaceRover();
    Sample BuildSample();
    void UpdateSampleGeometry(Sample& sample);

    RunManifest m_manifest;

    std::unique_ptr<chrono::ChSystem> m_sys;
    std::unique_ptr<Terrain> m_terrain;
    std::unique_ptr<Rover> m_rover;
    std::unique_ptr<Drivetrain> m_drivetrain;
    std::unique_ptr<Driver> m_driver;
    std::unique_ptr<Metrics> m_metrics;
    std::unique_ptr<TerminationMonitor> m_monitor;
    std::unique_ptr<TelemetryWriter> m_telemetry;

    Termination m_termination;
    Sample m_sample;

    chrono::ChFrame<> m_initial_pose;
    chrono::ChVector3d m_up{0, 0, 1};
    chrono::ChVector3d m_heading{1, 0, 0};
    chrono::ChVector3d m_last_position{0, 0, 0};

    /// The requests the driver issued this step, kept so sampling can report them without
    /// advancing the driver a second time.
    std::vector<double> m_last_requests;

    double m_distance = 0;
    double m_next_sample_time = 0;
    double m_sample_period = 0;
    double m_wall_clock_start = 0;
    double m_last_wall_clock = 0;
    double m_last_sim_time = 0;
    double m_previous_roll = 0;
    double m_previous_pitch = 0;

    bool m_initialized = false;
    bool m_finished = false;
};

}  // namespace trickfire
