#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChSystem.h"

#include "config/spec.hpp"

namespace trickfire {

/// One driven wheel station, after assembly.
struct Wheel {
    std::string id;
    std::shared_ptr<chrono::ChBodyAuxRef> body;
    /// Bearing and drive in one link: a torque motor with a REVOLUTE spindle constraint
    /// is a revolute joint that also takes an applied torque, so no separate bearing is
    /// needed and the station cannot be over-constrained.
    std::shared_ptr<chrono::ChLinkMotorRotationTorque> motor;
    std::shared_ptr<chrono::ChFunctionConst> torque_function;

    /// Spin axis in world coordinates at assembly time, and in the parent body's frame.
    chrono::ChVector3d axis_local;
    double radius = 0;
    double reflected_inertia = 0;

    /// Measured wheel speed about its own spin axis, signed so positive drives forward.
    double Speed() const;
    /// Wheel angle about the spin axis, accumulated across revolutions.
    double Angle() const;
};

/// One suspension pivot, after assembly.
struct Pivot {
    std::string name;
    std::shared_ptr<chrono::ChBodyAuxRef> body;
    std::shared_ptr<chrono::ChLinkLockRevolute> joint;
    std::shared_ptr<chrono::ChLinkRSDA> spring;

    double Angle() const;
    double Velocity() const;
    /// Reaction torque magnitude at the joint, in the joint frame.
    double ReactionTorque() const;
};

/// The assembled rover: chassis, rocker-bogie suspension, optional differential and six
/// torque-driven wheels.
///
/// Built entirely from a RoverSpec, which the Python CLI generates from the chassis URDF.
/// Nothing is hard-coded here, so a geometry change is a config change.
class Rover {
  public:
    Rover(chrono::ChSystem& system, const RunManifest& manifest);

    /// Place the rover with its chassis reference frame at the given pose.
    void Initialize(const chrono::ChFrame<>& pose);

    /// Apply this step's drive torques, indexed the same way as Wheels().
    void ApplyWheelTorques(const std::vector<double>& torques);

    /// Measured wheel speeds about their spin axes, in the same order as Wheels().
    const std::vector<double>& WheelSpeeds();

    const std::vector<Wheel>& Wheels() const { return m_wheels; }
    const std::vector<Pivot>& Pivots() const { return m_pivots; }
    const std::shared_ptr<chrono::ChBodyAuxRef>& Chassis() const { return m_chassis; }

    /// Every body belonging to the rover, for mass accounting and contact queries.
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> Bodies() const;

    /// Sum of every body's mass. Compared against the measured rover mass by the
    /// static-load test, which is the cheapest check that the model is the right rover.
    double TotalMass() const;

    /// Chassis roll and pitch about the world up axis, in radians.
    void ChassisAttitude(double& roll, double& pitch) const;

    /// Lowest point of the chassis collision geometry above the terrain, in metres.
    double GroundClearance(double terrain_height) const;

    /// Lock or free the suspension pivots, used by the articulation fixtures.
    void FixSuspension(bool fixed);

  private:
    void BuildChassis();
    void BuildPivots();
    void BuildDifferential();
    /// Inertia to give the kinematic differential's shafts. See BuildDifferential.
    double DifferentialShaftInertia() const;
    void BuildWheels();

    std::shared_ptr<chrono::ChBodyAuxRef> BodyByName(const std::string& name) const;

    chrono::ChSystem& m_sys;
    const RunManifest& m_manifest;
    chrono::ChVector3d m_up;

    std::shared_ptr<chrono::ChBodyAuxRef> m_chassis;
    std::vector<Pivot> m_pivots;
    std::vector<Wheel> m_wheels;

    // Kinematic differential, built from a planetary gear across the two rocker shafts.
    std::shared_ptr<chrono::ChShaftsPlanetary> m_differential;
    std::vector<std::shared_ptr<chrono::ChShaft>> m_diff_shafts;
    std::vector<std::shared_ptr<chrono::ChShaftBodyRotation>> m_diff_couplings;

    // Explicit differential linkage bodies, when the hardware has one.
    std::shared_ptr<chrono::ChBodyAuxRef> m_diff_bar;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_diff_links;

    std::vector<double> m_wheel_speeds;
    bool m_initialized = false;
};

}  // namespace trickfire
