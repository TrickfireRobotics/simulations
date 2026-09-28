#include "scm_scene.hpp"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace trickfire {

namespace {

/// Location-dependent soil properties. The SCM reference frame has its own x/y in the
/// ground plane, so y > 0 is one half of the patch and y < 0 the other.
class SplitSoilParams : public SCMTerrain::SoilParametersCallback {
  public:
    void Set(const ChVector3d& loc, double& Bekker_Kphi, double& Bekker_Kc, double& Bekker_n,
             double& Mohr_cohesion, double& Mohr_friction, double& Janosi_shear, double& elastic_K,
             double& damping_R) override {
        const SoilParams& p = (loc.y() > 0) ? kSoftSoil : kLeteSand;
        Bekker_Kphi = p.bekker_Kphi;
        Bekker_Kc = p.bekker_Kc;
        Bekker_n = p.bekker_n;
        Mohr_cohesion = p.mohr_cohesion;
        Mohr_friction = p.mohr_friction;
        Janosi_shear = p.janosi_shear;
        elastic_K = p.elastic_K;
        damping_R = p.damping_R;
    }
};

}  // namespace

ScmScene::ScmScene(const SimConfig& config)
    : m_config(config), m_wheel_start(0, 0.02 + config.tire_radius, -1.5) {
    m_sys.SetGravityY();
    m_sys.SetNumThreads(m_config.threads, 2 * m_config.threads, 1);
    m_sys.SetCollisionSystem(chrono_types::make_shared<ChCollisionSystemBullet>());

    BuildWheel();
    BuildTerrain();
}

void ScmScene::BuildWheel() {
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetFixed(true);
    m_sys.Add(m_ground);

    m_wheel = chrono_types::make_shared<ChBody>();
    m_sys.Add(m_wheel);
    m_wheel->SetMass(m_config.wheel_mass);
    m_wheel->SetInertiaXX(
        ChVector3d(m_config.wheel_inertia, m_config.wheel_inertia, m_config.wheel_inertia));
    m_wheel->SetPos(m_wheel_start + ChVector3d(0, 0.3, 0));

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    switch (m_config.tire_type) {
        case TireType::LUGGED: {
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
                GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"));

            auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            vis_shape->SetMesh(trimesh);
            vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            m_wheel->AddVisualShape(vis_shape);

            auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                material, trimesh, false, false, 0.01);
            m_wheel->AddCollisionShape(ct_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
            break;
        }
        case TireType::CYLINDRICAL: {
            auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(
                material, m_config.cylinder_radius, m_config.cylinder_width);
            m_wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector3d(0), QuatFromAngleY(CH_PI_2)));

            auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(
                m_config.cylinder_radius, m_config.cylinder_width);
            vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            m_wheel->AddVisualShape(vis_shape, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
            break;
        }
    }
    m_wheel->EnableCollision(true);

    // A speed motor (rather than an angle motor driven by a ramp) so the GUI slider can
    // change the wheel speed mid-run without stepping the commanded angle discontinuously.
    m_speed_fun = chrono_types::make_shared<ChFunctionConst>(0.0);
    m_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    m_motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    m_motor->SetSpeedFunction(m_speed_fun);
    m_motor->Initialize(m_wheel, m_ground, ChFrame<>(m_wheel_start, QuatFromAngleY(CH_PI_2)));
    m_sys.Add(m_motor);
}

void ScmScene::BuildTerrain() {
    m_terrain = std::make_unique<SCMTerrain>(&m_sys);

    // SCMTerrain uses the ISO frame (Z up); rotate to match the Y-up world frame
    m_terrain->SetReferenceFrame(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));
    m_terrain->Initialize(m_config.terrain_width, m_config.terrain_length,
                          m_config.mesh_resolution);

    if (m_config.variable_soil) {
        m_terrain->RegisterSoilParametersCallback(chrono_types::make_shared<SplitSoilParams>());
    } else {
        m_terrain->SetSoilParameters(kSoftSoil.bekker_Kphi, kSoftSoil.bekker_Kc, kSoftSoil.bekker_n,
                                     kSoftSoil.mohr_cohesion, kSoftSoil.mohr_friction,
                                     kSoftSoil.janosi_shear, kSoftSoil.elastic_K,
                                     kSoftSoil.damping_R);
    }

    m_terrain->SetBulldozingParameters(55, 1, 5, 6);

    if (m_config.enable_active_domains) {
        m_terrain->AddActiveDomain(
            m_wheel, ChVector3d(0, 0, 0),
            ChVector3d(0.5, 2 * m_config.tire_radius, 2 * m_config.tire_radius));
    }

    SetPlotType(m_plot_type, 0, 30000.2);
    m_terrain->SetColormap(ChColormap::Type::COPPER);
}

void ScmScene::SetPlotType(SCMTerrain::DataPlotType type, double min, double max) {
    m_plot_type = type;
    m_terrain->SetPlotType(type, min, max);
}

ChVector3d ScmScene::ContactForce() const {
    ChVector3d force, torque;
    m_terrain->GetContactForceBody(m_wheel, force, torque);
    return force;
}

double ScmScene::WheelSinkage() const {
    // Both queries project onto the grid, so the wheel's own height is ignored
    const auto& pos = m_wheel->GetPos();
    return m_terrain->GetInitHeight(pos) - m_terrain->GetHeight(pos);
}

void ScmScene::Reset() {
    // Flatten every node the wheel has touched back to the undeformed level. The patch is
    // built flat at level 0, so that level is the same for all of them.
    auto nodes = m_terrain->GetModifiedNodes(true);
    for (auto& node : nodes) node.second = 0.0;
    m_terrain->SetModifiedNodes(nodes);

    m_wheel->SetPos(m_wheel_start + ChVector3d(0, 0.3, 0));
    m_wheel->SetRot(QUNIT);
    m_wheel->SetLinVel(VNULL);
    m_wheel->SetAngVelLocal(VNULL);

    m_sys.SetChTime(0);
}

}  // namespace trickfire
