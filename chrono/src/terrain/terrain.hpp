#pragma once

#include <memory>
#include <string>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "config/spec.hpp"

namespace trickfire {

/// SCM counters and timers for one step, written to the telemetry's numerics group.
struct TerrainCounters {
    int ray_casts = 0;
    int ray_hits = 0;
    int contact_patches = 0;
    int erosion_nodes = 0;
    double timer_ray_casting = 0;
    double timer_contact_forces = 0;
    double timer_bulldozing = 0;
};

/// The terrain fidelity ladder behind one interface.
///
/// A scenario picks a tier and the rest of the simulator does not care which it got:
/// rigid ground for drivetrain and controls work, SCM for wheel ranking, mixed patches
/// for rock and crust transitions. Obstacles are rigid bodies in every tier, so a step
/// or rock field can be added to any of them.
class Terrain {
  public:
    Terrain(chrono::ChSystem& system, const RunManifest& manifest);

    /// Register the SCM active domains that follow each wheel. Called after the rover is
    /// assembled, because the domains are bound to the wheel bodies.
    void TrackWheels(const std::vector<std::shared_ptr<chrono::ChBody>>& wheels,
                     const std::vector<double>& radii, const std::vector<double>& widths);

    void Advance(double step_size);

    /// Undeformed surface height at a world point, used to place the rover and to
    /// measure sinkage.
    double InitialHeight(const chrono::ChVector3d& point) const;
    /// Current surface height, which differs from the above once a wheel has passed.
    double Height(const chrono::ChVector3d& point) const;

    /// Sinkage directly below a point: how far the surface has dropped since the start.
    double Sinkage(const chrono::ChVector3d& point) const;

    /// Net contact force and torque the terrain applies to a body.
    bool ContactForce(const std::shared_ptr<chrono::ChBody>& body, chrono::ChVector3d& force,
                      chrono::ChVector3d& torque) const;

    /// Flatten every node a wheel has deformed back to its initial level. Only valid for
    /// patches whose initial levels are recorded, which is every tier built here.
    void Reset();

    TerrainCounters Counters() const;

    bool IsDeformable() const { return m_scm != nullptr; }
    chrono::vehicle::SCMTerrain* Scm() const { return m_scm.get(); }
    chrono::vehicle::RigidTerrain* Rigid() const { return m_rigid.get(); }

    /// Surface normal direction of the patch, accounting for the configured slope. The
    /// scenario uses it to place the rover square to the ground.
    const chrono::ChVector3d& SurfaceNormal() const { return m_normal; }

    /// Frame the terrain was built in, which the slope is baked into.
    const chrono::ChCoordsys<>& ReferenceFrame() const { return m_reference; }

  private:
    void BuildReferenceFrame();
    void BuildRigid();
    void BuildScm();
    void BuildObstacles();

    chrono::ChSystem& m_sys;
    const RunManifest& m_manifest;
    const TerrainSpec& m_spec;

    chrono::ChVector3d m_up;
    chrono::ChVector3d m_normal;
    chrono::ChCoordsys<> m_reference;

    std::unique_ptr<chrono::vehicle::SCMTerrain> m_scm;
    std::unique_ptr<chrono::vehicle::RigidTerrain> m_rigid;
    std::vector<std::shared_ptr<chrono::ChBody>> m_obstacles;

    /// Initial node levels, recorded so Reset() works for height-map and mesh patches as
    /// well as flat ones.
    std::vector<chrono::vehicle::SCMTerrain::NodeLevel> m_initial_levels;
};

}  // namespace trickfire
