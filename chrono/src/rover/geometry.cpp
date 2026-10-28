#include "rover/geometry.hpp"

#include <stdexcept>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeConvexHull.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/collision/ChCollisionShapeSphere.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"

using namespace chrono;

namespace trickfire {

std::shared_ptr<ChContactMaterial> MakeContactMaterial(SimulationSpec::ContactMethod method,
                                                       double friction, double restitution) {
    if (method == SimulationSpec::ContactMethod::NSC) {
        auto material = chrono_types::make_shared<ChContactMaterialNSC>();
        material->SetFriction(static_cast<float>(friction));
        material->SetRestitution(static_cast<float>(restitution));
        return material;
    }

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    material->SetFriction(static_cast<float>(friction));
    material->SetRestitution(static_cast<float>(restitution));

    // Aluminium-extrusion chassis against soil and rock. These are stiffness defaults,
    // not measurements - the terrain's SCM parameters dominate wheel-soil behaviour, and
    // anything studying rigid-body impacts should set them deliberately.
    material->SetYoungModulus(2e7f);
    material->SetPoissonRatio(0.3f);
    return material;
}

void AddVisualMeshes(const std::shared_ptr<ChBodyAuxRef>& body,
                     const std::vector<VisualMesh>& meshes) {
    for (const auto& mesh : meshes) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh.file, true, true);
        if (!trimesh || trimesh->GetNumTriangles() == 0) {
            throw std::runtime_error("visual mesh has no triangles: " + mesh.file);
        }
        if (mesh.scale != 1.0) trimesh->Transform(VNULL, ChMatrix33<>(mesh.scale));

        auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        shape->SetMesh(trimesh);
        shape->SetName(mesh.file);
        shape->SetColor(ChColor(mesh.color[0], mesh.color[1], mesh.color[2]));
        body->AddVisualShape(shape, ChFrame<>(mesh.position, mesh.rotation));
    }
}

void AddCollisionShapes(const std::shared_ptr<ChBodyAuxRef>& body,
                        const std::vector<CollisionShape>& shapes,
                        const std::shared_ptr<ChContactMaterial>& material) {
    for (const auto& shape : shapes) {
        const ChFrame<> frame(shape.position, shape.rotation);

        switch (shape.type) {
            case CollisionShape::Type::BOX: {
                body->AddCollisionShape(
                    chrono_types::make_shared<ChCollisionShapeBox>(
                        material, shape.dimensions.x(), shape.dimensions.y(), shape.dimensions.z()),
                    frame);
                body->AddVisualShape(
                    chrono_types::make_shared<ChVisualShapeBox>(
                        shape.dimensions.x(), shape.dimensions.y(), shape.dimensions.z()),
                    frame);
                break;
            }
            case CollisionShape::Type::SPHERE: {
                body->AddCollisionShape(
                    chrono_types::make_shared<ChCollisionShapeSphere>(material, shape.radius),
                    frame);
                body->AddVisualShape(chrono_types::make_shared<ChVisualShapeSphere>(shape.radius),
                                     frame);
                break;
            }
            case CollisionShape::Type::CYLINDER: {
                body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCylinder>(
                                            material, shape.radius, shape.length),
                                        frame);
                body->AddVisualShape(
                    chrono_types::make_shared<ChVisualShapeCylinder>(shape.radius, shape.length),
                    frame);
                break;
            }
            case CollisionShape::Type::MESH: {
                auto trimesh =
                    ChTriangleMeshConnected::CreateFromWavefrontFile(shape.mesh_file, false, false);
                if (!trimesh || trimesh->GetNumTriangles() == 0) {
                    throw std::runtime_error("collision mesh has no triangles: " + shape.mesh_file);
                }
                body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                                            material, trimesh, false, false, shape.mesh_margin),
                                        frame);
                break;
            }
            case CollisionShape::Type::CONVEX_HULL: {
                auto trimesh =
                    ChTriangleMeshConnected::CreateFromWavefrontFile(shape.mesh_file, false, false);
                if (!trimesh || trimesh->GetNumTriangles() == 0) {
                    throw std::runtime_error("convex hull mesh has no triangles: " +
                                             shape.mesh_file);
                }
                // The mesh's own vertices are the hull points. The importer is expected to
                // have decimated it already; Chrono does not reduce it here.
                body->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeConvexHull>(
                                            material, trimesh->GetCoordsVertices()),
                                        frame);
                break;
            }
        }
    }
}

void ApplyMassProperties(const std::shared_ptr<ChBodyAuxRef>& body,
                         const MassProperties& properties) {
    body->SetMass(properties.mass);
    body->SetFrameCOMToRef(ChFrame<>(properties.com, QUNIT));
    body->SetInertiaXX(properties.inertia_xx);

    // Chrono's SetInertiaXY takes (Ixy, Ixz, Iyz) and stores them negated internally, so
    // the manifest's convention matches the usual CAD report directly.
    body->SetInertiaXY(properties.inertia_xy);
}

}  // namespace trickfire
