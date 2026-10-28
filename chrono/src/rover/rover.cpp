#include "rover/rover.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChConstants.h"

#include "powertrain/actuator.hpp"
#include "rover/geometry.hpp"

using namespace chrono;

namespace trickfire {

namespace {

/// Rotation taking the joint frame's Z axis onto `axis`. Revolute joints and rotation
/// motors both actuate about their frame's Z, so every joint in the rover is built this
/// way rather than by hand-writing quaternions per station.
ChQuaternion<> AlignZTo(const ChVector3d& axis) {
    const ChVector3d target = axis.GetNormalized();
    const ChVector3d z(0, 0, 1);

    const double dot = Vdot(z, target);
    if (dot > 1 - 1e-12) return QUNIT;
    if (dot < -1 + 1e-12) return QuatFromAngleX(CH_PI);  // any axis perpendicular to Z

    const ChVector3d cross = Vcross(z, target);
    ChQuaternion<> q(1 + dot, cross.x(), cross.y(), cross.z());
    q.Normalize();
    return q;
}

/// Collision family for every rover body.
///
/// The rover's own parts cannot physically touch each other, but the chassis collision
/// volume is a deliberately crude set of primitives that *does* overlap the wheels and
/// the suspension arms. Leaving self-collision on would fill the contact solver with
/// spurious contacts at t = 0 and, under explicit SMC stepping, blow the model apart
/// before it ever reaches the ground.
///
/// Terrain patches and obstacles stay in the default family, so wheel-ground and
/// chassis-rock contact are unaffected.
constexpr int kRoverCollisionFamily = 2;

void JoinRoverFamily(const std::shared_ptr<ChBodyAuxRef>& body) {
    if (!body->GetCollisionModel()) return;
    body->GetCollisionModel()->SetFamily(kRoverCollisionFamily);
    body->GetCollisionModel()->DisallowCollisionsWith(kRoverCollisionFamily);
}

}  // namespace

// ---------------------------------------------------------------------------- //
//                              measurement helpers                             //
// ---------------------------------------------------------------------------- //

double Wheel::Speed() const { return motor ? motor->GetMotorAngleDt() : 0.0; }

double Wheel::Angle() const { return motor ? motor->GetMotorAngle() : 0.0; }

double Pivot::Angle() const {
    if (!joint) return 0.0;

    // ChLinkMarkers keeps the relative rotation as an angle-axis pair in the joint
    // frame, and a revolute joint's free axis is that frame's Z - so the signed angle is
    // the magnitude with the axis's Z sign. GetFrame2Rel() looks like the obvious source
    // but is frame 2 relative to its own body, which is constant, so reading it reports
    // a fixed angle no matter how far the joint has actually travelled.
    const double axis_sign = joint->GetRelAngleAxis().z() < 0 ? -1.0 : 1.0;
    return joint->GetRelAngle() * axis_sign;
}

double Pivot::Velocity() const {
    if (!joint) return 0.0;
    return joint->GetRelativeAngVel().z();
}

double Pivot::ReactionTorque() const {
    if (!joint) return 0.0;
    return joint->GetReaction2().torque.Length();
}

// ---------------------------------------------------------------------------- //
//                                   assembly                                   //
// ---------------------------------------------------------------------------- //

Rover::Rover(ChSystem& system, const RunManifest& manifest) : m_sys(system), m_manifest(manifest) {
    m_up = manifest.simulation.up_axis == SimulationSpec::UpAxis::Z ? ChVector3d(0, 0, 1)
                                                                    : ChVector3d(0, 1, 0);
    BuildChassis();
    BuildPivots();
    BuildWheels();
    BuildDifferential();
}

void Rover::BuildChassis() {
    m_chassis = chrono_types::make_shared<ChBodyAuxRef>();
    m_chassis->SetName("chassis");
    ApplyMassProperties(m_chassis, m_manifest.rover.chassis);
    AddVisualMeshes(m_chassis, m_manifest.rover.chassis_visual);

    auto material =
        MakeContactMaterial(m_manifest.simulation.contact_method, m_manifest.rover.body_friction,
                            m_manifest.rover.body_restitution);
    AddCollisionShapes(m_chassis, m_manifest.rover.chassis_collision, material);
    m_chassis->EnableCollision(true);
    JoinRoverFamily(m_chassis);
    m_sys.AddBody(m_chassis);
}

void Rover::BuildPivots() {
    auto material =
        MakeContactMaterial(m_manifest.simulation.contact_method, m_manifest.rover.body_friction,
                            m_manifest.rover.body_restitution);

    for (const auto& spec : m_manifest.rover.pivots) {
        Pivot pivot;
        pivot.name = spec.name;
        pivot.body = chrono_types::make_shared<ChBodyAuxRef>();
        pivot.body->SetName(spec.name);
        ApplyMassProperties(pivot.body, spec.mass_properties);
        AddVisualMeshes(pivot.body, spec.visual);
        AddCollisionShapes(pivot.body, spec.collision, material);
        pivot.body->EnableCollision(!spec.collision.empty());
        JoinRoverFamily(pivot.body);
        m_sys.AddBody(pivot.body);
        m_pivots.push_back(std::move(pivot));
    }
}

void Rover::BuildWheels() {
    for (const auto& station : m_manifest.rover.wheels) {
        const WheelSpec& package = m_manifest.Wheel(station.wheel_package);
        const ActuatorSpec& actuator = m_manifest.Actuator(station.actuator);

        Wheel wheel;
        wheel.id = station.id;
        wheel.radius = package.radius;
        wheel.axis_local = station.axis;

        wheel.body = chrono_types::make_shared<ChBodyAuxRef>();
        wheel.body->SetName("wheel_" + station.id);

        MassProperties properties = package.mass_properties;

        // Fold the drive's reflected rotor inertia into the wheel about its spin axis.
        // For a rigid gearbox this is exactly equivalent to a separate rotor state, and
        // it avoids a stiff two-inertia pair that a 2 ms step cannot integrate well.
        Actuator probe(actuator, m_manifest.simulation.step_size);
        wheel.reflected_inertia = probe.ReflectedInertia();
        for (int i = 0; i < 3; i++) {
            if (std::abs(package.roll_axis[i]) > 0.5) {
                properties.inertia_xx[i] += wheel.reflected_inertia;
            }
        }
        ApplyMassProperties(wheel.body, properties);

        // Orient the package so its own roll axis lines up with this station's spin axis,
        // then apply the station's handedness and mount roll.
        ChQuaternion<> mount = AlignZTo(station.axis) * AlignZTo(package.roll_axis).GetConjugate();
        if (station.mirror) mount = mount * QuatFromAngleZ(CH_PI);
        if (station.mount_roll_rad != 0) {
            mount = mount * QuatFromAngleAxis(station.mount_roll_rad, package.roll_axis);
        }

        VisualMesh visual = package.visual;
        visual.rotation = mount * visual.rotation;
        visual.position = mount.Rotate(visual.position);
        AddVisualMeshes(wheel.body, {visual});

        std::vector<CollisionShape> collision = package.collision;
        for (auto& shape : collision) {
            shape.rotation = mount * shape.rotation;
            shape.position = mount.Rotate(shape.position);
        }

        // The wheel's contact material comes from the terrain, so a soil change does not
        // need every wheel package edited.
        auto material = MakeContactMaterial(m_manifest.simulation.contact_method,
                                            m_manifest.terrain.rigid_friction,
                                            m_manifest.terrain.rigid_restitution);
        AddCollisionShapes(wheel.body, collision, material);
        wheel.body->EnableCollision(true);
        JoinRoverFamily(wheel.body);
        m_sys.AddBody(wheel.body);

        wheel.torque_function = chrono_types::make_shared<ChFunctionConst>(0.0);
        wheel.motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
        wheel.motor->SetName("drive_" + station.id);
        wheel.motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::REVOLUTE);
        wheel.motor->SetTorqueFunction(wheel.torque_function);

        m_wheels.push_back(std::move(wheel));
    }
    m_wheel_speeds.assign(m_wheels.size(), 0.0);
}

double Rover::DifferentialShaftInertia() const {
    // Scale from the rockers' own inertia about the pivot axis, so the conditioning holds
    // for a rover of any size rather than only this one.
    double largest = 0;
    for (const auto& pivot : m_manifest.rover.pivots) {
        if (pivot.parent != "chassis") continue;
        for (int i = 0; i < 3; i++) {
            if (std::abs(pivot.axis[i]) > 0.5) {
                largest = std::max(largest, pivot.mass_properties.inertia_xx[i]);
            }
        }
    }
    return 0.5 * largest;
}

void Rover::BuildDifferential() {
    const auto& spec = m_manifest.rover.differential;
    if (spec.type == DifferentialSpec::Type::NONE) return;

    std::vector<const PivotSpec*> rockers;
    for (const auto& pivot : m_manifest.rover.pivots) {
        if (pivot.parent == "chassis") rockers.push_back(&pivot);
    }
    if (rockers.size() != 2) {
        throw std::runtime_error("a differential needs exactly two chassis pivots, found " +
                                 std::to_string(rockers.size()));
    }

    if (spec.type == DifferentialSpec::Type::KINEMATIC) {
        // A planetary gear across (chassis, left rocker, right rocker) with ratios
        // (-2, 1, 1) enforces -2*w_chassis + w_left + w_right = 0, which is exactly
        // "the rockers rotate equally and oppositely relative to the chassis". Writing it
        // this way rather than against absolute rotations keeps it correct when the whole
        // rover pitches on a slope.
        // The shafts need an inertia comparable to the bodies they are tied to. They are
        // a modelling device, but giving them a token 1e-4 makes the shaft subsystem
        // three orders of magnitude softer than the rockers, and the iterative VI solver
        // then fails to enforce the planetary constraint at all - the rockers drift
        // straight to their travel stops as if the differential were not there. A real
        // differential bar has inertia of this order anyway.
        const double shaft_inertia = std::max(1e-3, DifferentialShaftInertia());
        for (int i = 0; i < 3; i++) {
            auto shaft = chrono_types::make_shared<ChShaft>();
            shaft->SetInertia(shaft_inertia);
            m_sys.AddShaft(shaft);
            m_diff_shafts.push_back(shaft);
        }

        m_differential = chrono_types::make_shared<ChShaftsPlanetary>();
        m_differential->SetName("rocker_differential");
        if (!m_differential->Initialize(m_diff_shafts[0], m_diff_shafts[1], m_diff_shafts[2])) {
            throw std::runtime_error("the differential's planetary gear failed to initialize");
        }
        m_differential->SetTransmissionRatios(-2, 1, 1);
        m_sys.Add(m_differential);
        return;
    }

    // LINKAGE: a bar pivoted on the chassis with a link down to each rocker, as on
    // Curiosity. The joints are added in Initialize once the bodies are placed.
    m_diff_bar = chrono_types::make_shared<ChBodyAuxRef>();
    m_diff_bar->SetName("differential_bar");
    ApplyMassProperties(m_diff_bar, spec.bar_mass_properties);
    m_sys.AddBody(m_diff_bar);

    for (int side = 0; side < 2; side++) {
        auto link = chrono_types::make_shared<ChBodyAuxRef>();
        link->SetName(side == 0 ? "differential_link_left" : "differential_link_right");
        ApplyMassProperties(link, spec.link_mass_properties);
        m_sys.AddBody(link);
        m_diff_links.push_back(link);
    }
}

// ---------------------------------------------------------------------------- //
//                                 initialisation                               //
// ---------------------------------------------------------------------------- //

void Rover::Initialize(const ChFrame<>& pose) {
    if (m_initialized) throw std::runtime_error("Rover::Initialize called twice");

    m_chassis->SetFrameRefToAbs(pose);

    // Bodies first, so every joint can be built directly in world coordinates. Pivot
    // parents are validated to come earlier in the list, so one pass is enough.
    for (size_t i = 0; i < m_pivots.size(); i++) {
        const auto& spec = m_manifest.rover.pivots[i];
        m_pivots[i].body->SetFrameRefToAbs(ChFrame<>(pose * spec.position, pose.GetRot()));
    }
    for (size_t i = 0; i < m_wheels.size(); i++) {
        const auto& station = m_manifest.rover.wheels[i];
        m_wheels[i].body->SetFrameRefToAbs(ChFrame<>(pose * station.position, pose.GetRot()));
    }

    for (size_t i = 0; i < m_pivots.size(); i++) {
        const auto& spec = m_manifest.rover.pivots[i];
        auto parent = BodyByName(spec.parent);

        const ChFrame<> joint_frame(pose * spec.position, pose.GetRot() * AlignZTo(spec.axis));

        auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
        joint->SetName(spec.name + "_pivot");
        joint->Initialize(m_pivots[i].body, parent, joint_frame);

        if (spec.limits_rad) {
            // Penalty limits: a revolute joint's travel stop is a hard contact on the real
            // rover, and Chrono's penalty formulation is far better behaved under the
            // explicit SMC stepping used here than a unilateral constraint would be.
            auto& limit = joint->LimitRz();
            limit.SetActive(true);
            limit.SetMin(spec.limits_rad->first);
            limit.SetMax(spec.limits_rad->second);
            limit.SetSpringCoefficientMin(1e5);
            limit.SetSpringCoefficientMax(1e5);
            limit.SetDampingCoefficientMin(1e3);
            limit.SetDampingCoefficientMax(1e3);
        }
        m_sys.AddLink(joint);
        m_pivots[i].joint = joint;

        if (spec.spring_stiffness > 0 || spec.spring_damping > 0) {
            auto spring = chrono_types::make_shared<ChLinkRSDA>();
            spring->SetName(spec.name + "_spring");
            spring->Initialize(m_pivots[i].body, parent, joint_frame);
            spring->SetSpringCoefficient(spec.spring_stiffness);
            spring->SetDampingCoefficient(spec.spring_damping);
            spring->SetRestAngle(spec.rest_angle_rad);
            m_sys.AddLink(spring);
            m_pivots[i].spring = spring;
        }
    }

    for (size_t i = 0; i < m_wheels.size(); i++) {
        const auto& station = m_manifest.rover.wheels[i];
        auto parent = BodyByName(station.parent);

        const ChFrame<> joint_frame(pose * station.position,
                                    pose.GetRot() * AlignZTo(station.axis));
        m_wheels[i].motor->Initialize(m_wheels[i].body, parent, joint_frame);
        m_sys.AddLink(m_wheels[i].motor);
    }

    const auto& diff = m_manifest.rover.differential;
    if (diff.type == DifferentialSpec::Type::KINEMATIC) {
        std::vector<std::shared_ptr<ChBodyAuxRef>> coupled{m_chassis};
        std::vector<ChVector3d> axes{VNULL};

        for (size_t i = 0; i < m_pivots.size(); i++) {
            if (m_manifest.rover.pivots[i].parent != "chassis") continue;
            coupled.push_back(m_pivots[i].body);
            axes.push_back(m_manifest.rover.pivots[i].axis);
        }
        // All three shafts measure rotation about the rocker pivot axis, expressed in each
        // body's own frame. The bodies start co-aligned, so the chassis uses the same axis.
        axes[0] = axes.size() > 1 ? axes[1] : ChVector3d(0, 1, 0);

        for (size_t i = 0; i < coupled.size(); i++) {
            auto coupling = chrono_types::make_shared<ChShaftBodyRotation>();
            if (!coupling->Initialize(m_diff_shafts[i], coupled[i], axes[i])) {
                throw std::runtime_error("a differential shaft could not be tied to body \"" +
                                         std::string(coupled[i]->GetName()) + "\"");
            }
            m_sys.Add(coupling);
            m_diff_couplings.push_back(coupling);
        }
    } else if (diff.type == DifferentialSpec::Type::LINKAGE) {
        m_diff_bar->SetFrameRefToAbs(ChFrame<>(pose * diff.bar_position, pose.GetRot()));

        const ChVector3d link_positions[] = {diff.left_link_position, diff.right_link_position};
        for (int side = 0; side < 2; side++) {
            m_diff_links[side]->SetFrameRefToAbs(
                ChFrame<>(pose * link_positions[side], pose.GetRot()));
        }

        // The bar pivots on the chassis about the rover's longitudinal axis.
        auto bar_axis = ChVector3d(0, 0, 1);
        for (const auto& pivot : m_manifest.rover.pivots) {
            if (pivot.parent == "chassis") {
                // Perpendicular to the rocker axis and to the world up direction.
                bar_axis = Vcross(pivot.axis, m_up).GetNormalized();
                break;
            }
        }

        auto bar_joint = chrono_types::make_shared<ChLinkLockRevolute>();
        bar_joint->SetName("differential_bar_pivot");
        bar_joint->Initialize(
            m_diff_bar, m_chassis,
            ChFrame<>(pose * diff.bar_position, pose.GetRot() * AlignZTo(bar_axis)));
        m_sys.AddLink(bar_joint);

        std::vector<std::shared_ptr<ChBodyAuxRef>> rockers;
        ChVector3d rocker_axis(0, 1, 0);
        for (size_t i = 0; i < m_pivots.size(); i++) {
            if (m_manifest.rover.pivots[i].parent != "chassis") continue;
            rockers.push_back(m_pivots[i].body);
            rocker_axis = m_manifest.rover.pivots[i].axis;
        }

        for (int side = 0; side < 2; side++) {
            auto to_rocker = chrono_types::make_shared<ChLinkLockRevolute>();
            to_rocker->Initialize(
                rockers[side], m_diff_links[side],
                ChFrame<>(pose * link_positions[side], pose.GetRot() * AlignZTo(rocker_axis)));
            m_sys.AddLink(to_rocker);

            // The link meets the bar directly above where it meets the rocker, which is
            // the arrangement the bar_position / link_position pair describes.
            ChVector3d bar_end = link_positions[side];
            const ChVector3d along = Vcross(rocker_axis, m_up).GetNormalized();
            bar_end += along * Vdot(diff.bar_position - link_positions[side], along);

            auto to_bar = chrono_types::make_shared<ChLinkLockRevolute>();
            to_bar->Initialize(m_diff_links[side], m_diff_bar,
                               ChFrame<>(pose * bar_end, pose.GetRot() * AlignZTo(rocker_axis)));
            m_sys.AddLink(to_bar);
        }
    }

    m_initialized = true;
}

std::shared_ptr<ChBodyAuxRef> Rover::BodyByName(const std::string& name) const {
    if (name == "chassis") return m_chassis;
    for (const auto& pivot : m_pivots) {
        if (pivot.name == name) return pivot.body;
    }
    throw std::runtime_error("no rover body named \"" + name + "\"");
}

// ---------------------------------------------------------------------------- //
//                                   stepping                                   //
// ---------------------------------------------------------------------------- //

void Rover::ApplyWheelTorques(const std::vector<double>& torques) {
    if (torques.size() != m_wheels.size()) {
        throw std::runtime_error("got " + std::to_string(torques.size()) + " torques for " +
                                 std::to_string(m_wheels.size()) + " wheels");
    }
    for (size_t i = 0; i < m_wheels.size(); i++) {
        m_wheels[i].torque_function->SetConstant(torques[i]);
    }
}

const std::vector<double>& Rover::WheelSpeeds() {
    for (size_t i = 0; i < m_wheels.size(); i++) m_wheel_speeds[i] = m_wheels[i].Speed();
    return m_wheel_speeds;
}

// ---------------------------------------------------------------------------- //
//                                  measurement                                 //
// ---------------------------------------------------------------------------- //

std::vector<std::shared_ptr<ChBodyAuxRef>> Rover::Bodies() const {
    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies{m_chassis};
    for (const auto& pivot : m_pivots) bodies.push_back(pivot.body);
    for (const auto& wheel : m_wheels) bodies.push_back(wheel.body);
    if (m_diff_bar) bodies.push_back(m_diff_bar);
    for (const auto& link : m_diff_links) bodies.push_back(link);
    return bodies;
}

double Rover::TotalMass() const {
    double total = 0;
    for (const auto& body : Bodies()) total += body->GetMass();
    return total;
}

void Rover::ChassisAttitude(double& roll, double& pitch) const {
    // Roll and pitch of the chassis relative to the world up direction, taken from the
    // chassis axes rather than a fixed Cardan order, so the result is the same whether
    // the world is Y-up or Z-up.
    const ChMatrix33<> rotation = m_chassis->GetFrameRefToAbs().GetRotMat();
    const ChVector3d forward = rotation.GetAxisX();
    const ChVector3d lateral = rotation.GetAxisY();

    pitch = std::asin(std::clamp(Vdot(forward, m_up), -1.0, 1.0));
    roll = std::asin(std::clamp(Vdot(lateral, m_up), -1.0, 1.0));
}

double Rover::GroundClearance(double terrain_height) const {
    double lowest = std::numeric_limits<double>::infinity();
    for (const auto& shape : m_manifest.rover.chassis_collision) {
        // Only the shape origin is sampled, offset by its own half-height along the world
        // up direction. Exact for the box and cylinder primitives the chassis is allowed
        // to use, and a convex hull reports its origin.
        const ChVector3d world =
            m_chassis->GetFrameRefToAbs().TransformPointLocalToParent(shape.position);
        double half = 0;
        switch (shape.type) {
            case CollisionShape::Type::BOX:
                half = 0.5 * std::abs(Vdot(shape.dimensions, m_up));
                break;
            case CollisionShape::Type::SPHERE:
                half = shape.radius;
                break;
            case CollisionShape::Type::CYLINDER:
                half = std::max(shape.radius, 0.5 * shape.length);
                break;
            default:
                break;
        }
        lowest = std::min(lowest, Vdot(world, m_up) - half);
    }
    if (!std::isfinite(lowest)) return 0;
    return lowest - terrain_height;
}

void Rover::FixSuspension(bool fixed) {
    for (auto& pivot : m_pivots) {
        if (pivot.joint) pivot.joint->Lock(fixed);
    }
}

}  // namespace trickfire
