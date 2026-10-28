#pragma once

#include <memory>
#include <vector>

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactMaterial.h"

#include "config/spec.hpp"

namespace trickfire {

/// Build a contact material matching the system's contact method. SMC needs stiffness
/// and damping, NSC does not, and handing the wrong one to a system throws deep inside
/// the collision system rather than here.
std::shared_ptr<chrono::ChContactMaterial> MakeContactMaterial(SimulationSpec::ContactMethod method,
                                                               double friction, double restitution);

/// Attach visual meshes to a body. Missing files are a load-time error, so anything
/// reaching here is known to exist.
void AddVisualMeshes(const std::shared_ptr<chrono::ChBodyAuxRef>& body,
                     const std::vector<VisualMesh>& meshes);

/// Attach collision shapes to a body, all sharing one material.
void AddCollisionShapes(const std::shared_ptr<chrono::ChBodyAuxRef>& body,
                        const std::vector<CollisionShape>& shapes,
                        const std::shared_ptr<chrono::ChContactMaterial>& material);

/// Apply mass, centre of mass and the full inertia tensor to a body whose reference
/// frame is already set.
void ApplyMassProperties(const std::shared_ptr<chrono::ChBodyAuxRef>& body,
                         const MassProperties& properties);

}  // namespace trickfire
