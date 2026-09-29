#pragma once

#include <optional>
#include <string>
#include <vector>

namespace trickfire {

/// One selectable wheel: either the built-in procedural cylinder, or a triangle mesh
/// loaded from an OBJ file (Chrono's own tractor wheel, or a custom one). The optional
/// fields are only ever set by a custom wheel's "<name>.toml" sidecar, and override the
/// matching SimConfig default when present (see BuildWheelCatalog).
struct WheelOption {
    std::string name;
    bool is_mesh;
    std::string mesh_path;         // unused when !is_mesh
    double mesh_scale = 1.0;       // multiplies mesh vertices, to convert its units to meters
    double mesh_rotation_deg = 0;  // corrective yaw (about the vertical axis) applied after scaling

    std::optional<double> mass;     // kg
    std::optional<double> inertia;  // kg*m^2, applied equally to all three axes
    std::optional<double> radius;   // m - wheel start height and SCM active-domain box size
};

/// The procedural cylinder, Chrono's tractor wheel, and one entry per *.obj file found in
/// chrono/data/models/wheels/ - drop a mesh in there and it shows up here under its
/// filename, in both `sim chrono run <wheel>` and the GUI's wheel dropdown.
///
/// A sibling "<name>.toml" file next to "<name>.obj" can set any of:
///
///   scale = 0.0254        # mesh units -> meters (CAD exports are often cm or inches)
///   rotation_deg = 90     # corrective yaw, for meshes not authored with the roll axis
///                         # this scene expects
///   mass = 3.0            # kg, overrides SimConfig's generic default
///   inertia = 0.05        # kg*m^2, overrides SimConfig's generic default
///   radius = 0.26         # m, overrides SimConfig's generic default
///
/// All fields are optional and default to whatever SimConfig already uses.
std::vector<WheelOption> BuildWheelCatalog();

/// Index of the option with this name, or -1 if there isn't one.
int FindWheelOption(const std::vector<WheelOption>& catalog, const std::string& name);

}  // namespace trickfire
