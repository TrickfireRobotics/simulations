#include "terrain/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChConstants.h"

#include "rover/geometry.hpp"

using namespace chrono;
using namespace chrono::vehicle;

namespace trickfire {

namespace {

/// Location-dependent soil, resolved from the terrain's region list. Regions are tested
/// in order and the first containing the point wins, so a background soil is just the
/// last entry - or the default when nothing matches.
class RegionSoil : public SCMTerrain::SoilParametersCallback {
  public:
    RegionSoil(std::vector<SoilRegion> regions, std::vector<SoilSpec> soils,
               std::string default_soil)
        : m_regions(std::move(regions)),
          m_soils(std::move(soils)),
          m_default(std::move(default_soil)) {}

    void Set(const ChVector3d& loc, double& Bekker_Kphi, double& Bekker_Kc, double& Bekker_n,
             double& Mohr_cohesion, double& Mohr_friction, double& Janosi_shear, double& elastic_K,
             double& damping_R) override {
        const SoilSpec* soil = Lookup(m_default);

        // The callback receives the point in the SCM frame, whose x/y lie in the ground
        // plane, so region bounds are authored in those same coordinates.
        for (const auto& region : m_regions) {
            if (loc.x() >= region.x_min && loc.x() <= region.x_max && loc.y() >= region.y_min &&
                loc.y() <= region.y_max) {
                const SoilSpec* match = Lookup(region.soil);
                if (match) soil = match;
                break;
            }
        }
        if (!soil) return;

        Bekker_Kphi = soil->bekker_kphi;
        Bekker_Kc = soil->bekker_kc;
        Bekker_n = soil->bekker_n;
        Mohr_cohesion = soil->mohr_cohesion;
        Mohr_friction = soil->mohr_friction;
        Janosi_shear = soil->janosi_shear;
        elastic_K = soil->elastic_k;
        damping_R = soil->damping_r;
    }

  private:
    const SoilSpec* Lookup(const std::string& name) const {
        for (const auto& soil : m_soils) {
            if (soil.name == name) return &soil;
        }
        return nullptr;
    }

    std::vector<SoilRegion> m_regions;
    std::vector<SoilSpec> m_soils;
    std::string m_default;
};

}  // namespace

Terrain::Terrain(ChSystem& system, const RunManifest& manifest)
    : m_sys(system), m_manifest(manifest), m_spec(manifest.terrain) {
    m_up = manifest.simulation.up_axis == SimulationSpec::UpAxis::Z ? ChVector3d(0, 0, 1)
                                                                    : ChVector3d(0, 1, 0);
    BuildReferenceFrame();

    switch (m_spec.type) {
        case TerrainSpec::Type::RIGID_FLAT:
        case TerrainSpec::Type::RIGID_MESH:
            BuildRigid();
            break;
        case TerrainSpec::Type::SCM_FLAT:
        case TerrainSpec::Type::SCM_HEIGHTMAP:
        case TerrainSpec::Type::SCM_MESH:
            BuildScm();
            break;
        case TerrainSpec::Type::MIXED:
            // Tier 3: a deformable patch with rigid regions laid on top of it. The rigid
            // parts come in as obstacles, which is the same mechanism a rock field uses.
            BuildScm();
            BuildRigid();
            break;
    }
    BuildObstacles();
}

void Terrain::BuildReferenceFrame() {
    // Slope is applied by tilting the whole patch rather than by rotating gravity, so a
    // slope scenario and a flat one differ only in this frame and the rover's start pose.
    ChQuaternion<> tilt = QUNIT;
    if (m_manifest.simulation.up_axis == SimulationSpec::UpAxis::Z) {
        // SCM's own frame is Z-up, so the reference frame is the identity plus the slope.
        tilt = QuatFromAngleY(m_spec.pitch_rad) * QuatFromAngleX(m_spec.roll_rad);
        m_reference = ChCoordsys<>(VNULL, tilt);
    } else {
        // Y-up world: rotate SCM's Z-up frame onto it first, then tilt. Kept for the
        // original single-wheel prototype; new work should use Z-up.
        tilt = QuatFromAngleY(m_spec.pitch_rad) * QuatFromAngleX(m_spec.roll_rad);
        m_reference = ChCoordsys<>(VNULL, QuatFromAngleX(-CH_PI_2) * tilt);
    }
    m_normal = m_reference.rot.Rotate(ChVector3d(0, 0, 1));
    if (m_manifest.simulation.up_axis == SimulationSpec::UpAxis::Y) {
        m_normal = tilt.Rotate(ChVector3d(0, 1, 0));
    }
}

void Terrain::BuildRigid() {
    m_rigid = std::make_unique<RigidTerrain>(&m_sys);
    auto material = MakeContactMaterial(m_manifest.simulation.contact_method, m_spec.rigid_friction,
                                        m_spec.rigid_restitution);

    if (m_spec.type == TerrainSpec::Type::RIGID_MESH) {
        m_rigid->AddPatch(material, m_reference, m_spec.mesh_file, true);
    } else if (m_spec.type == TerrainSpec::Type::RIGID_FLAT) {
        // Tiled, so a long traverse patch does not become one enormous collision box.
        const bool tiled = std::max(m_spec.size_x, m_spec.size_y) > 20.0;
        m_rigid->AddPatch(material, m_reference, m_spec.size_x, m_spec.size_y, 1.0, tiled, 4.0);
    }
    // MIXED adds no rigid patch of its own: its rigid content is the obstacle list, which
    // keeps a single surface under the wheels and avoids two overlapping ground planes.
    if (!m_rigid->GetPatches().empty()) m_rigid->Initialize();
}

void Terrain::BuildScm() {
    m_scm = std::make_unique<SCMTerrain>(&m_sys);
    m_scm->SetReferenceFrame(m_reference);

    switch (m_spec.type) {
        case TerrainSpec::Type::SCM_HEIGHTMAP:
            m_scm->Initialize(m_spec.heightmap_file, m_spec.size_x, m_spec.size_y,
                              m_spec.height_min, m_spec.height_max, m_spec.grid_delta);
            break;
        case TerrainSpec::Type::SCM_MESH:
            m_scm->Initialize(m_spec.mesh_file, m_spec.grid_delta);
            break;
        default:
            m_scm->Initialize(m_spec.size_x, m_spec.size_y, m_spec.grid_delta);
            break;
    }

    if (m_spec.soils.size() == 1 && m_spec.regions.empty()) {
        const SoilSpec& soil = m_spec.soils.front();
        m_scm->SetSoilParameters(soil.bekker_kphi, soil.bekker_kc, soil.bekker_n,
                                 soil.mohr_cohesion, soil.mohr_friction, soil.janosi_shear,
                                 soil.elastic_k, soil.damping_r);
    } else {
        m_scm->RegisterSoilParametersCallback(chrono_types::make_shared<RegionSoil>(
            m_spec.regions, m_spec.soils, m_spec.default_soil));
    }

    m_scm->EnableBulldozing(m_spec.bulldozing);
    if (m_spec.bulldozing) {
        m_scm->SetBulldozingParameters(m_spec.bulldozing_params.x(), m_spec.bulldozing_params.y(),
                                       static_cast<int>(m_spec.bulldozing_params.z()),
                                       m_spec.bulldozing_erosion_iters);
    }

    // Snapshot every node's starting level so Reset() restores a height-map or mesh patch
    // correctly, not only a flat one.
    m_initial_levels = m_scm->GetModifiedNodes(true);
}

void Terrain::BuildObstacles() {
    for (const auto& spec : m_spec.obstacles) {
        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        body->SetFixed(spec.fixed);

        if (spec.fixed) {
            // A fixed body's mass is irrelevant, but Chrono still wants it positive.
            body->SetMass(1.0);
        } else {
            body->SetMass(spec.mass);
            // Treat a loose rock as a solid of comparable extent; the exact tensor matters
            // far less than whether it moves at all, and nobody measures rock inertia.
            const double extent = spec.shape.type == CollisionShape::Type::SPHERE
                                      ? spec.shape.radius
                                      : 0.5 * spec.shape.dimensions.Length();
            const double inertia = 0.4 * spec.mass * extent * extent;
            body->SetInertiaXX(ChVector3d(inertia, inertia, inertia));
        }

        auto material = MakeContactMaterial(m_manifest.simulation.contact_method, spec.friction,
                                            m_spec.rigid_restitution);
        AddCollisionShapes(body, {spec.shape}, material);
        body->EnableCollision(true);

        // Obstacles are placed in the tilted terrain frame, so a rock on a slope sits on
        // the slope rather than hovering over it.
        const ChVector3d world = m_reference.TransformPointLocalToParent(spec.position);
        body->SetFrameRefToAbs(ChFrame<>(world, m_reference.rot * spec.rotation));
        m_sys.AddBody(body);
        m_obstacles.push_back(body);
    }
}

void Terrain::TrackWheels(const std::vector<std::shared_ptr<ChBody>>& wheels,
                          const std::vector<double>& radii, const std::vector<double>& widths) {
    if (!m_scm || !m_spec.active_domains) return;

    for (size_t i = 0; i < wheels.size(); i++) {
        const double margin = m_spec.active_domain_margin;
        // The box is expressed in the wheel's own frame: wide across the axle, and a full
        // diameter plus margin in the other two directions so the rut ahead of and behind
        // the contact patch stays inside the domain.
        const ChVector3d dims(widths[i] + 2 * margin, 2 * radii[i] + 2 * margin,
                              2 * radii[i] + 2 * margin);
        m_scm->AddActiveDomain(wheels[i], VNULL, dims);
    }
}

void Terrain::Advance(double step_size) {
    // SCMTerrain and RigidTerrain are both ChTerrain subclasses whose Advance only
    // updates their own bookkeeping; the system step is driven by Simulation.
    if (m_scm) m_scm->Advance(step_size);
    if (m_rigid) m_rigid->Advance(step_size);
}

double Terrain::InitialHeight(const ChVector3d& point) const {
    if (m_scm) return m_scm->GetInitHeight(point);
    if (m_rigid) return m_rigid->GetHeight(point);
    return 0;
}

double Terrain::Height(const ChVector3d& point) const {
    if (m_scm) return m_scm->GetHeight(point);
    if (m_rigid) return m_rigid->GetHeight(point);
    return 0;
}

double Terrain::Sinkage(const ChVector3d& point) const {
    if (!m_scm) return 0;
    // Both queries project onto the grid, so the querying body's own height is ignored.
    return m_scm->GetInitHeight(point) - m_scm->GetHeight(point);
}

bool Terrain::ContactForce(const std::shared_ptr<ChBody>& body, ChVector3d& force,
                           ChVector3d& torque) const {
    if (m_scm && m_scm->GetContactForceBody(body, force, torque)) return true;

    // Rigid terrain contact goes through the normal collision system, so the body itself
    // carries the accumulated force.
    force = body->GetContactForce();
    torque = body->GetContactTorque();
    return force.Length() > 0 || torque.Length() > 0;
}

void Terrain::Reset() {
    if (!m_scm) return;

    auto modified = m_scm->GetModifiedNodes(true);
    if (m_initial_levels.empty()) {
        for (auto& node : modified) node.second = 0.0;
        m_scm->SetModifiedNodes(modified);
        return;
    }
    m_scm->SetModifiedNodes(m_initial_levels);
}

TerrainCounters Terrain::Counters() const {
    TerrainCounters counters;
    if (!m_scm) return counters;

    counters.ray_casts = m_scm->GetNumRayCasts();
    counters.ray_hits = m_scm->GetNumRayHits();
    counters.contact_patches = m_scm->GetNumContactPatches();
    counters.erosion_nodes = m_scm->GetNumErosionNodes();
    counters.timer_ray_casting = m_scm->GetTimerRayCasting();
    counters.timer_contact_forces = m_scm->GetTimerContactForces();
    counters.timer_bulldozing = m_scm->GetTimerBulldozing();
    return counters;
}

}  // namespace trickfire
