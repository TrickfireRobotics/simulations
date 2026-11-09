// Physics unit tests: the checks from the development plan's verification layer 2.
//
// Free fall, a pendulum, a static wheel load, an ideal slope force and a constrained
// joint. These are deliberately not rover tests - they confirm that the solver settings
// the simulator pins actually integrate simple, exactly-known mechanics correctly, so
// that a later disagreement on the rover can be attributed to the model rather than to
// the integrator.

#include <cmath>
#include <memory>
#include <vector>

#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/utils/ChConstants.h"

#include "check.hpp"

using namespace chrono;

namespace {

constexpr double kGravity = 9.81;

std::unique_ptr<ChSystemSMC> MakeSystem(int iterations = 200) {
    auto sys = std::make_unique<ChSystemSMC>();
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -kGravity));
    sys->SetNumThreads(1, 1, 1);
    // The same collision system and iteration count the simulator pins, so these tests
    // characterise the configuration that actually ships. Leaving the collision system
    // to the default is not harmless: bodies silently fall through each other.
    sys->SetCollisionSystem(chrono_types::make_shared<ChCollisionSystemBullet>());
    if (auto solver = std::dynamic_pointer_cast<ChIterativeSolverVI>(sys->GetSolver())) {
        solver->SetMaxIterations(iterations);
    }
    return sys;
}

void TestFreeFall() {
    auto sys = MakeSystem();

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(2.0);
    body->SetInertiaXX(ChVector3d(1, 1, 1));
    body->SetPos(ChVector3d(0, 0, 0));
    sys->AddBody(body);

    const double step = 1e-3;
    const double duration = 1.0;
    for (int i = 0; i < static_cast<int>(duration / step); i++) sys->DoStepDynamics(step);

    // s = -g t^2 / 2, v = -g t. The tolerance is loose enough for one step of the
    // integrator's own offset and no looser.
    check::Near(body->GetPos().z(), -0.5 * kGravity * duration * duration, 2e-2,
                "free fall covers g t^2 / 2");
    check::Near(body->GetPosDt().z(), -kGravity * duration, 2e-2,
                "free fall reaches a speed of g t");
}

/// A uniform rod hanging below its pivot, started a few degrees off vertical.
///
/// Small amplitude on purpose: it is the case with a closed-form period, and it tests the
/// integrator rather than testing how well it copes with a body swinging through 180
/// degrees. A rod rather than a point mass on a massless arm, because a body with a
/// near-zero rotational inertia under a revolute constraint leaves the mass matrix nearly
/// singular and the solver returns nonsense angular velocities.
struct Pendulum {
    std::shared_ptr<ChBody> rod;
    double length = 0;
    double mass = 0;

    /// Small-angle period of a uniform rod pivoted about one end.
    double Period() const { return CH_2PI * std::sqrt(2.0 * length / (3.0 * kGravity)); }

    /// Angle from vertical, from the position of the rod's centre.
    double Angle() const {
        const ChVector3d centre = rod->GetPos();
        return std::atan2(centre.x(), -centre.z());
    }
};

Pendulum AddPendulum(ChSystem& sys, double length, double mass, double offset_deg) {
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    const double angle = offset_deg * CH_DEG_TO_RAD;
    const ChQuaternion<> rotation = QuatFromAngleY(angle);

    auto rod = chrono_types::make_shared<ChBody>();
    rod->SetMass(mass);
    // Only the moment about the swing axis affects the period, and the revolute joint
    // constrains the other two, so one well-conditioned value is used for all three.
    const double moment = mass * length * length / 12.0;
    rod->SetInertiaXX(ChVector3d(moment, moment, moment));
    rod->SetPos(rotation.Rotate(ChVector3d(0, 0, -length / 2)));
    rod->SetRot(rotation);
    sys.AddBody(rod);

    // Revolute about the world Y axis at the origin, so the rod swings in the XZ plane.
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(rod, ground, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    sys.AddLink(joint);

    return {rod, length, mass};
}

void TestPendulumPeriod() {
    auto sys = MakeSystem();
    auto pendulum = AddPendulum(*sys, 1.0, 1.0, 5.0);

    // Time a full period by watching the horizontal position return to its starting side
    // twice: the zero crossings of x are half a period apart.
    const double step = 2e-4;
    double previous = pendulum.Angle();
    std::vector<double> crossings;

    for (int i = 0; i < 40000 && crossings.size() < 3; i++) {
        sys->DoStepDynamics(step);
        const double angle = pendulum.Angle();
        if ((previous > 0) != (angle > 0)) crossings.push_back(sys->GetChTime());
        previous = angle;
    }

    check::True(crossings.size() >= 3, "the pendulum oscillated");
    if (crossings.size() >= 3) {
        // Two zero crossings span one half period, so the first to the third is a full one.
        check::Close(crossings[2] - crossings[0], pendulum.Period(), 0.02,
                     "the small-angle rod pendulum period matches 2 pi sqrt(2 L / 3 g)");
    }
}

void TestEnergyConservation() {
    // A frictionless pendulum must conserve energy. Drift here is the integrator's, and
    // bounding it is what justifies trusting a longer rover run.
    auto sys = MakeSystem();
    auto pendulum = AddPendulum(*sys, 0.8, 2.0, 10.0);

    auto energy = [&] {
        const double speed = pendulum.rod->GetPosDt().Length();
        const ChVector3d spin = pendulum.rod->GetAngVelParent();
        const double moment = pendulum.mass * pendulum.length * pendulum.length / 12.0;
        return 0.5 * pendulum.mass * speed * speed + 0.5 * moment * spin.Length2() +
               pendulum.mass * kGravity * pendulum.rod->GetPos().z();
    };

    const double initial = energy();
    const double step = 2e-4;
    double worst = 0;
    for (int i = 0; i < 25000; i++) {
        sys->DoStepDynamics(step);
        worst = std::max(worst, std::abs(energy() - initial));
    }

    // Compare against the swing's own energy range, not against m g L: at 10 degrees the
    // pendulum only exchanges m g L (1 - cos 10 deg), and measuring drift against the
    // larger number would hide a real loss.
    // The rod's centre is at L/2, so its potential swing is m g (L/2) (1 - cos theta).
    const double swing =
        pendulum.mass * kGravity * (pendulum.length / 2.0) * (1.0 - std::cos(10.0 * CH_DEG_TO_RAD));
    check::Less(worst / swing, 0.05,
                "a frictionless pendulum conserves energy to within 5% of its swing over 5 s");
}

/// A box built the way the production code builds one: explicit collision shape, then
/// EnableCollision. ChBodyEasy* exists but its overloads differ between Chrono versions
/// and silently produced a body with no collision at all here.
std::shared_ptr<ChBody> AddBox(ChSystem& sys, const ChVector3d& size, double mass,
                               const ChVector3d& position, const ChQuaternion<>& rotation,
                               const std::shared_ptr<ChContactMaterial>& material, bool fixed) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(mass);
    body->SetInertiaXX(ChVector3d(mass * (size.y() * size.y() + size.z() * size.z()) / 12.0,
                                  mass * (size.x() * size.x() + size.z() * size.z()) / 12.0,
                                  mass * (size.x() * size.x() + size.y() * size.y()) / 12.0));
    body->SetPos(position);
    body->SetRot(rotation);
    body->SetFixed(fixed);
    body->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeBox>(material, size.x(), size.y(), size.z()));
    body->EnableCollision(true);
    sys.AddBody(body);
    return body;
}

std::shared_ptr<ChContactMaterial> SmcMaterial(double friction) {
    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    material->SetFriction(static_cast<float>(friction));
    material->SetRestitution(0.0f);
    material->SetYoungModulus(2e7f);
    material->SetPoissonRatio(0.3f);
    return material;
}

void TestStaticWheelLoad() {
    auto sys = MakeSystem();
    auto material = SmcMaterial(0.9);

    AddBox(*sys, ChVector3d(4, 4, 0.4), 1000.0, ChVector3d(0, 0, -0.2), QUNIT, material, true);

    const double mass = 20.0;
    const double radius = 0.2;
    const double width = 0.1;

    auto wheel = chrono_types::make_shared<ChBody>();
    wheel->SetMass(mass);
    wheel->SetInertiaXX(ChVector3d(mass * (3 * radius * radius + width * width) / 12.0,
                                   0.5 * mass * radius * radius,
                                   mass * (3 * radius * radius + width * width) / 12.0));
    wheel->SetPos(ChVector3d(0, 0, radius + 1e-3));
    wheel->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, width),
        // The primitive is built about its local Z, so it is laid along Y to be a wheel.
        ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    wheel->EnableCollision(true);
    sys->AddBody(wheel);

    const double step = 5e-4;
    for (int i = 0; i < 4000; i++) sys->DoStepDynamics(step);

    check::Close(wheel->GetContactForce().z(), mass * kGravity, 0.05,
                 "a resting wheel's contact force equals its weight");
    check::Near(wheel->GetPos().z(), radius, 5e-3, "and it rests at one radius above ground");
    // Vertical velocity only: an unconstrained cylinder with no rolling resistance will
    // happily trundle off on the slightest contact asymmetry, which says nothing about
    // whether it has settled onto the ground.
    check::Less(std::abs(wheel->GetPosDt().z()), 0.02, "and has settled vertically");
}

void TestSlopeForce() {
    // On a frictionless slope the normal force is m g cos(theta) and the downslope
    // acceleration is g sin(theta). This is the check that catches a slope implemented by
    // tilting the wrong frame.
    auto sys = MakeSystem();

    const double angle = 15.0 * CH_DEG_TO_RAD;
    const double mass = 10.0;
    const ChQuaternion<> tilt = QuatFromAngleY(angle);

    auto material = SmcMaterial(0.0);
    AddBox(*sys, ChVector3d(6, 6, 0.4), 1000.0, tilt.Rotate(ChVector3d(0, 0, -0.2)), tilt, material,
           true);

    const double size = 0.2;
    auto block = AddBox(*sys, ChVector3d(size, size, size), mass,
                        tilt.Rotate(ChVector3d(0, 0, size / 2 + 1e-3)), tilt, material, false);

    const double step = 2e-4;
    // Let it settle onto the surface before measuring, so the normal force is the static
    // one rather than an impact transient.
    for (int i = 0; i < 1000; i++) sys->DoStepDynamics(step);

    const ChVector3d normal = tilt.Rotate(ChVector3d(0, 0, 1));
    const ChVector3d downslope = tilt.Rotate(ChVector3d(1, 0, 0));

    double worst_normal_error = 0;
    const double start_speed = Vdot(block->GetPosDt(), downslope);
    const double start_time = sys->GetChTime();

    for (int i = 0; i < 2000; i++) {
        sys->DoStepDynamics(step);
        const double normal_force = Vdot(block->GetContactForce(), normal);
        worst_normal_error = std::max(worst_normal_error,
                                      std::abs(normal_force - mass * kGravity * std::cos(angle)));
    }

    const double elapsed = sys->GetChTime() - start_time;
    const double acceleration = (Vdot(block->GetPosDt(), downslope) - start_speed) / elapsed;

    check::Close(worst_normal_error + mass * kGravity * std::cos(angle),
                 mass * kGravity * std::cos(angle), 0.1,
                 "the normal force on a slope stays at m g cos(theta)");
    check::Close(acceleration, kGravity * std::sin(angle), 0.05,
                 "and a frictionless block accelerates at g sin(theta)");

    // Without this the acceleration check passes just as well on a block in free fall,
    // which accelerates along the downslope direction at exactly the same rate.
    const double height_above_surface = Vdot(block->GetPos(), normal) - size / 2;
    check::Less(std::abs(height_above_surface), 5e-3,
                "and it stayed on the surface rather than falling past it");
}

void TestRevoluteConstraint() {
    // A revolute joint must hold its centre exactly, whatever is hung off it. This is the
    // check that catches a joint built in the wrong frame.
    auto sys = MakeSystem();

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    auto arm = chrono_types::make_shared<ChBody>();
    arm->SetMass(5.0);
    arm->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    arm->SetPos(ChVector3d(0.5, 0, 0));
    sys->AddBody(arm);

    const ChVector3d pivot(0, 0, 0);
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(arm, ground, ChFrame<>(pivot, QuatFromAngleX(CH_PI_2)));
    sys->AddLink(joint);

    const double step = 1e-3;
    double worst_drift = 0;
    double worst_radius_error = 0;
    for (int i = 0; i < 3000; i++) {
        sys->DoStepDynamics(step);
        worst_drift = std::max(worst_drift, std::abs(arm->GetPos().y()));
        worst_radius_error = std::max(worst_radius_error, std::abs(arm->GetPos().Length() - 0.5));
    }

    check::Less(worst_drift, 1e-6, "a revolute joint allows no motion out of its plane");
    check::Less(worst_radius_error, 1e-4, "and holds its arm at a constant radius");
}

}  // namespace

int main() {
    TestFreeFall();
    TestPendulumPeriod();
    TestStaticWheelLoad();
    TestSlopeForce();
    TestRevoluteConstraint();
    TestEnergyConservation();
    return check::Finish("physics");
}
