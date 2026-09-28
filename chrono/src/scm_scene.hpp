#pragma once

#include <memory>

#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "sim_config.hpp"

namespace trickfire {

/// The wheel-on-soil scene: a driven wheel above an SCM deformable terrain patch.
/// Owns the Chrono system and everything in it.
class ScmScene {
  public:
    explicit ScmScene(const SimConfig& config);

    chrono::ChSystemSMC& System() { return m_sys; }
    chrono::vehicle::SCMTerrain& Terrain() { return *m_terrain; }
    std::shared_ptr<chrono::ChBody> Wheel() const { return m_wheel; }

    /// Commanded wheel angular velocity (rad/s). Safe to call every step.
    void SetWheelSpeed(double omega) { m_speed_fun->SetConstant(omega); }

    /// SCMTerrain has no getter for the current false-colour quantity, so track it here.
    chrono::vehicle::SCMTerrain::DataPlotType PlotType() const { return m_plot_type; }
    void SetPlotType(chrono::vehicle::SCMTerrain::DataPlotType type, double min, double max);

    /// Measured motor angle, angular velocity and reaction torque.
    double MotorAngle() const { return m_motor->GetMotorAngle(); }
    double MotorSpeed() const { return m_motor->GetMotorAngleDt(); }
    double MotorTorque() const { return m_motor->GetMotorTorque(); }

    /// Net terrain contact force on the wheel, in world coordinates.
    chrono::ChVector3d ContactForce() const;

    /// Sinkage of the terrain directly below the wheel centre (m, positive down).
    double WheelSinkage() const;

    /// Restore the wheel pose and flatten the ruts. Note that this only restores a
    /// flat patch correctly - a terrain built from a height map would need its
    /// per-node initial levels recorded instead.
    void Reset();

    void AdvanceTo(double step_size) { m_sys.DoStepDynamics(step_size); }

  private:
    void BuildWheel();
    void BuildTerrain();

    SimConfig m_config;
    chrono::ChSystemSMC m_sys;
    std::unique_ptr<chrono::vehicle::SCMTerrain> m_terrain;

    std::shared_ptr<chrono::ChBody> m_ground;
    std::shared_ptr<chrono::ChBody> m_wheel;
    std::shared_ptr<chrono::ChLinkMotorRotationSpeed> m_motor;
    std::shared_ptr<chrono::ChFunctionConst> m_speed_fun;

    chrono::vehicle::SCMTerrain::DataPlotType m_plot_type =
        chrono::vehicle::SCMTerrain::PLOT_PRESSURE_YIELD;

    chrono::ChVector3d m_wheel_start;
};

}  // namespace trickfire
