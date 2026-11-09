#include "config/load.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>

#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChConstants.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

using namespace chrono;

namespace trickfire {

namespace fs = std::filesystem;

namespace {

/// Marker the authored configs use for values nobody has measured yet. Anything still
/// carrying it is a hard error: the development plan's whole point is that a convincing
/// number nobody measured is worse than a missing one.
constexpr const char* kPlaceholderPrefix = "REPLACE_WITH";

bool IsPlaceholder(const std::string& value) { return value.rfind(kPlaceholderPrefix, 0) == 0; }

/// A node that tests false.
///
/// `YAML::Node()` is a *Null* node, not an undefined one, so it tests true - which makes
/// "key is missing" indistinguishable from "key is present" at every call site. A
/// non-const lookup of a key that cannot exist yields a zombie, which is undefined,
/// tests false, and is safe to copy and query. Nothing is ever assigned through it, so
/// the holder is never modified.
const YAML::Node& Absent() {
    static YAML::Node holder = YAML::Load("{}");
    static YAML::Node absent = holder["\x01 absent"];
    return absent;
}

/// Collects problems with a key path attached, so an error points at
/// "rover.wheels[2].position" rather than just "bad vector".
class Reader {
  public:
    Reader(const YAML::Node& root, fs::path base_dir)
        : m_root(root), m_base_dir(std::move(base_dir)) {}

    const std::vector<std::string>& Problems() const { return m_problems; }
    bool Failed() const { return !m_problems.empty(); }

    void Error(const std::string& path, const std::string& message) const {
        m_problems.push_back(path + ": " + message);
    }

    /// Whether a dotted path exists and carries a value.
    bool Has(const YAML::Node& node, const std::string& path) const {
        return Present(At(node, path, false));
    }

    /// Resolve a path from the manifest against the manifest's own directory.
    std::string File(const YAML::Node& node, const std::string& path, bool required) {
        std::string raw = Str(node, path, required, "");
        if (raw.empty()) return raw;

        fs::path p(raw);
        if (!p.is_absolute()) p = m_base_dir / p;

        std::error_code ec;
        if (!fs::exists(p, ec)) {
            Error(path, "file not found: " + p.string());
            return {};
        }
        return fs::weakly_canonical(p, ec).string();
    }

    std::string Str(const YAML::Node& node, const std::string& path, bool required,
                    const std::string& fallback) {
        const YAML::Node child = At(node, path, required);
        if (!Present(child)) return fallback;

        std::string value;
        try {
            value = child.as<std::string>();
        } catch (const YAML::Exception&) {
            Error(path, "expected a string");
            return fallback;
        }
        if (IsPlaceholder(value)) {
            Error(path, "still set to the " + std::string(kPlaceholderPrefix) +
                            " placeholder - fill in a measured or datasheet value");
            return fallback;
        }
        return value;
    }

    double Num(const YAML::Node& node, const std::string& path, bool required, double fallback) {
        const YAML::Node child = At(node, path, required);
        if (!Present(child)) return fallback;

        if (child.IsScalar() && IsPlaceholder(child.Scalar())) {
            Error(path, "still set to the " + std::string(kPlaceholderPrefix) +
                            " placeholder - fill in a measured or datasheet value");
            return fallback;
        }

        try {
            return child.as<double>();
        } catch (const YAML::Exception&) {
            Error(path, "expected a number, got \"" + child.Scalar() + "\"");
            return fallback;
        }
    }

    /// A number that must be strictly positive. Catches the "wheel ten times too large"
    /// and "zero mass" classes of mistake at load time.
    double Positive(const YAML::Node& node, const std::string& path, bool required,
                    double fallback) {
        const bool present = Has(node, path);
        double value = Num(node, path, required, fallback);
        if (present && !(value > 0)) {
            Error(path, "must be greater than zero, got " + Format(value));
        }
        return value;
    }

    double InRange(const YAML::Node& node, const std::string& path, bool required, double fallback,
                   double lo, double hi) {
        const bool present = Has(node, path);
        double value = Num(node, path, required, fallback);
        if (present && (value < lo || value > hi)) {
            Error(path, "must be between " + Format(lo) + " and " + Format(hi) + ", got " +
                            Format(value));
        }
        return value;
    }

    int Int(const YAML::Node& node, const std::string& path, bool required, int fallback) {
        return static_cast<int>(Num(node, path, required, fallback));
    }

    bool Bool(const YAML::Node& node, const std::string& path, bool fallback) {
        const YAML::Node child = At(node, path, false);
        if (!Present(child)) return fallback;
        try {
            return child.as<bool>();
        } catch (const YAML::Exception&) {
            Error(path, "expected true or false");
            return fallback;
        }
    }

    ChVector3d Vec(const YAML::Node& node, const std::string& path, bool required,
                   const ChVector3d& fallback) {
        const YAML::Node child = At(node, path, required);
        if (!Present(child)) return fallback;

        if (!child.IsSequence() || child.size() != 3) {
            Error(path, "expected a 3-element list [x, y, z]");
            return fallback;
        }
        ChVector3d out = fallback;
        for (int i = 0; i < 3; i++) {
            try {
                out[i] = child[i].as<double>();
            } catch (const YAML::Exception&) {
                Error(path, "element " + std::to_string(i) + " is not a number");
                return fallback;
            }
        }
        return out;
    }

    /// A direction that must be a cardinal unit axis. Suspension and spin axes are read
    /// this way because an axis that is "nearly" +Y silently produces a rover that drives
    /// in a slow curve, which is far harder to spot than a load error.
    ChVector3d Axis(const YAML::Node& node, const std::string& path, bool required,
                    const ChVector3d& fallback) {
        if (!Has(node, path)) {
            if (required) At(node, path, true);  // records the missing-key error
            return fallback;
        }
        ChVector3d axis = Vec(node, path, required, fallback);

        double length = axis.Length();
        if (length < 1e-9) {
            Error(path, "axis has zero length");
            return fallback;
        }
        axis /= length;

        int nonzero = 0;
        for (int i = 0; i < 3; i++) {
            if (std::abs(axis[i]) > 1e-6) nonzero++;
        }
        if (nonzero != 1) {
            Error(path, "must be a cardinal axis such as [0, 1, 0]; got a direction " +
                            std::to_string(nonzero) +
                            " components wide. Rotate the model "
                            "instead of tilting the axis.");
        }
        return axis;
    }

    /// Reads either a quaternion [w, x, y, z] or Cardan XYZ angles in degrees.
    ChQuaternion<> Rotation(const YAML::Node& node, const std::string& path) {
        const YAML::Node child = At(node, path, false);
        if (!Present(child)) return QUNIT;

        if (child.IsSequence() && child.size() == 4) {
            ChQuaternion<> q(child[0].as<double>(), child[1].as<double>(), child[2].as<double>(),
                             child[3].as<double>());
            if (q.Length() < 1e-9) {
                Error(path, "quaternion has zero length");
                return QUNIT;
            }
            q.Normalize();
            return q;
        }
        if (child.IsSequence() && child.size() == 3) {
            ChVector3d rpy = Vec(node, path, true, VNULL) * CH_DEG_TO_RAD;
            return QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, rpy}).GetNormalized();
        }
        Error(path, "expected [roll, pitch, yaw] in degrees or [w, x, y, z]");
        return QUNIT;
    }

    /// Looks up a dotted path like "rover.chassis.mass" under `node`.
    ///
    /// Three yaml-cpp traps are avoided here.
    ///
    /// `current = current[key]` *writes the child into current's own position*, which
    /// rewrites the document as it is read, so the walk uses reset() to rebind instead.
    ///
    /// The const operator[] returns an *invalid* node for a missing key, and every
    /// accessor on one throws - including IsDefined(). So the lookup goes through the
    /// non-const operator[], whose result for a missing key is a harmless zombie that
    /// tests false and inserts nothing unless assigned to.
    ///
    /// And a default-constructed Node is a *Null* node, not an undefined one, so it
    /// tests true - returning one for a missing key would make absence undetectable.
    /// Absent() supplies a node that really does test false. Treating an explicit
    /// `key: null` as absent too is deliberate: in a config it means the same thing.
    YAML::Node At(const YAML::Node& node, const std::string& path, bool required) const {
        YAML::Node current;
        current.reset(node);

        std::istringstream parts(path);
        std::string key;

        while (std::getline(parts, key, '.')) {
            if (!current || !current.IsMap()) {
                if (required) Error(path, "required key is missing");
                return Absent();
            }
            YAML::Node child = current[key];
            if (!child) {
                if (required) Error(path, "required key is missing");
                return Absent();
            }
            current.reset(child);
        }
        if (current.IsNull()) {
            if (required) Error(path, "required key is null");
            return Absent();
        }
        return current;
    }

    const YAML::Node& Root() const { return m_root; }
    const fs::path& BaseDir() const { return m_base_dir; }

  private:
    /// A node that exists and carries a value. At() already maps both "missing" and
    /// "explicitly null" onto an undefined node, so this is now just its truthiness -
    /// kept as a named concept because the reason is not obvious from `!!node`.
    static bool Present(const YAML::Node& node) { return static_cast<bool>(node); }

    static std::string Format(double value) {
        std::ostringstream out;
        out << value;
        return out.str();
    }

    YAML::Node m_root;
    fs::path m_base_dir;
    // Mutable so the const path-walker above can record a missing key.
    mutable std::vector<std::string> m_problems;
};

// ---------------------------------------------------------------------------- //
//                              shared sub-readers                              //
// ---------------------------------------------------------------------------- //

MassProperties ReadMassProperties(Reader& r, const YAML::Node& node, const std::string& path,
                                  bool require_mass) {
    MassProperties mp;
    const YAML::Node child = r.At(node, path, require_mass);
    if (!child) return mp;

    mp.mass = r.Positive(child, "mass", require_mass, 0);
    mp.com = r.Vec(child, "com", false, VNULL);
    mp.inertia_xx = r.Vec(child, "inertia_xx", require_mass, ChVector3d(0, 0, 0));
    mp.inertia_xy = r.Vec(child, "inertia_xy", false, VNULL);
    mp.provenance = r.Str(child, "provenance", false, "estimated");

    // A body with mass but no inertia spins up like a point mass, which looks plausible
    // and is wrong for anything measuring articulation or rollover.
    if (mp.mass > 0 && mp.inertia_xx.Length() <= 0) {
        r.Error(path + ".inertia_xx", "a body with mass needs a non-zero inertia tensor");
    }

    // Triangle inequality on the principal moments. Catches transposed or mismatched-unit
    // CAD exports, which otherwise produce a stable but physically impossible body.
    const ChVector3d& I = mp.inertia_xx;
    if (I.x() > 0 && I.y() > 0 && I.z() > 0) {
        const double tol = 1e-6 * std::max({I.x(), I.y(), I.z()});
        if (I.x() + I.y() < I.z() - tol || I.y() + I.z() < I.x() - tol ||
            I.z() + I.x() < I.y() - tol) {
            r.Error(path + ".inertia_xx",
                    "principal moments violate the triangle inequality, so they cannot describe "
                    "a real body - check the units and axis convention of the CAD export");
        }
    }
    return mp;
}

VisualMesh ReadVisualMesh(Reader& r, const YAML::Node& node) {
    VisualMesh mesh;
    mesh.file = r.File(node, "file", true);
    mesh.position = r.Vec(node, "position", false, VNULL);
    mesh.rotation = r.Rotation(node, "rotation");
    mesh.scale = r.Positive(node, "scale", false, 1.0);

    const YAML::Node color = r.At(node, "color", false);
    if (color && color.IsSequence() && color.size() == 3) {
        for (int i = 0; i < 3; i++) mesh.color[i] = color[i].as<float>();
    }
    return mesh;
}

CollisionShape ReadCollisionShape(Reader& r, const YAML::Node& node, const std::string& path) {
    CollisionShape shape;
    const std::string type = r.Str(node, "type", true, "box");

    shape.position = r.Vec(node, "position", false, VNULL);
    shape.rotation = r.Rotation(node, "rotation");

    if (type == "box") {
        shape.type = CollisionShape::Type::BOX;
        shape.dimensions = r.Vec(node, "dimensions", true, VNULL);
        for (int i = 0; i < 3; i++) {
            if (!(shape.dimensions[i] > 0)) {
                r.Error(path + ".dimensions", "box extents must all be greater than zero");
                break;
            }
        }
    } else if (type == "sphere") {
        shape.type = CollisionShape::Type::SPHERE;
        shape.radius = r.Positive(node, "radius", true, 0);
    } else if (type == "cylinder") {
        shape.type = CollisionShape::Type::CYLINDER;
        shape.radius = r.Positive(node, "radius", true, 0);
        shape.length = r.Positive(node, "length", true, 0);
    } else if (type == "mesh" || type == "convex_hull") {
        shape.type =
            type == "mesh" ? CollisionShape::Type::MESH : CollisionShape::Type::CONVEX_HULL;
        shape.mesh_file = r.File(node, "file", true);
        shape.mesh_margin = r.Num(node, "margin", false, 0.01);
    } else if (!type.empty()) {
        r.Error(path + ".type", "unknown shape \"" + type +
                                    "\"; expected box, sphere, cylinder, mesh or "
                                    "convex_hull");
    }
    return shape;
}

std::vector<CollisionShape> ReadCollisionList(Reader& r, const YAML::Node& node,
                                              const std::string& path) {
    std::vector<CollisionShape> shapes;
    const YAML::Node list = r.At(node, path, false);
    if (!list) return shapes;

    if (!list.IsSequence()) {
        r.Error(path, "expected a list of collision shapes");
        return shapes;
    }
    for (size_t i = 0; i < list.size(); i++) {
        shapes.push_back(ReadCollisionShape(r, list[i], path + "[" + std::to_string(i) + "]"));
    }
    return shapes;
}

std::vector<VisualMesh> ReadVisualList(Reader& r, const YAML::Node& node, const std::string& path) {
    std::vector<VisualMesh> meshes;
    const YAML::Node list = r.At(node, path, false);
    if (!list) return meshes;

    if (!list.IsSequence()) {
        r.Error(path, "expected a list of visual meshes");
        return meshes;
    }
    for (size_t i = 0; i < list.size(); i++) meshes.push_back(ReadVisualMesh(r, list[i]));
    return meshes;
}

// ---------------------------------------------------------------------------- //
//                                   sections                                   //
// ---------------------------------------------------------------------------- //

RunSpec ReadRun(Reader& r, const YAML::Node& root) {
    RunSpec run;
    const YAML::Node node = r.At(root, "run", true);
    if (!node) return run;

    run.id = r.Str(node, "id", true, "");
    run.study = r.Str(node, "study", false, "");
    run.seed = static_cast<std::uint64_t>(r.Num(node, "seed", false, 0));
    run.headless = r.Bool(node, "headless", true);
    run.output_dir = r.Str(node, "output_dir", false, ".");
    return run;
}

SimulationSpec ReadSimulation(Reader& r, const YAML::Node& root) {
    SimulationSpec sim;
    const YAML::Node node = r.At(root, "simulation", true);
    if (!node) return sim;

    sim.step_size = r.InRange(node, "step_size", true, 2e-3, 1e-6, 0.1);
    sim.threads = std::max(1, r.Int(node, "threads", false, 4));
    sim.gravity = r.Positive(node, "gravity", false, 9.81);
    sim.solver_iterations = std::max(10, r.Int(node, "solver_iterations", false, 200));

    const std::string method = r.Str(node, "contact_method", false, "smc");
    if (method == "smc") {
        sim.contact_method = SimulationSpec::ContactMethod::SMC;
    } else if (method == "nsc") {
        sim.contact_method = SimulationSpec::ContactMethod::NSC;
    } else {
        r.Error("simulation.contact_method", "expected smc or nsc, got \"" + method + "\"");
    }

    const std::string up = r.Str(node, "up_axis", false, "z");
    if (up == "z") {
        sim.up_axis = SimulationSpec::UpAxis::Z;
    } else if (up == "y") {
        sim.up_axis = SimulationSpec::UpAxis::Y;
    } else {
        r.Error("simulation.up_axis", "expected y or z, got \"" + up + "\"");
    }
    return sim;
}

WheelSpec ReadWheel(Reader& r, const YAML::Node& node, const std::string& path) {
    WheelSpec wheel;
    wheel.name = r.Str(node, "name", true, "");
    wheel.revision = r.Str(node, "revision", false, "");
    wheel.content_hash = r.Str(node, "content_hash", false, "");

    // Bounds wide enough for anything from a desktop test wheel to an oversized paddle,
    // but narrow enough to catch a mesh imported in millimetres or inches.
    wheel.radius = r.InRange(node, "radius", true, 0, 0.02, 2.0);
    wheel.width = r.InRange(node, "width", true, 0, 0.01, 2.0);

    wheel.mass_properties = ReadMassProperties(r, node, "mass_properties", true);
    wheel.roll_axis = r.Axis(node, "roll_axis", true, ChVector3d(0, 1, 0));

    const YAML::Node visual = r.At(node, "visual", true);
    if (visual) wheel.visual = ReadVisualMesh(r, visual);

    wheel.collision = ReadCollisionList(r, node, "collision");
    if (wheel.collision.empty()) {
        r.Error(path + ".collision", "a wheel package needs at least one contact shape");
    }
    return wheel;
}

ActuatorSpec ReadActuator(Reader& r, const YAML::Node& node) {
    ActuatorSpec act;
    act.name = r.Str(node, "name", true, "");
    act.manufacturer = r.Str(node, "identity.manufacturer", false, "");
    act.model = r.Str(node, "identity.model", true, "");
    act.hardware_revision = r.Str(node, "identity.hardware_revision", true, "");
    act.firmware = r.Str(node, "identity.firmware", false, "");
    act.provenance = r.Str(node, "provenance.quality", false, "placeholder");

    const std::string mode = r.Str(node, "command.mode", false, "velocity");
    if (mode == "velocity") {
        act.command.mode = CommandPathSpec::Mode::VELOCITY;
    } else if (mode == "torque") {
        act.command.mode = CommandPathSpec::Mode::TORQUE;
    } else {
        r.Error("command.mode", "expected velocity or torque, got \"" + mode + "\"");
    }
    act.command.rate_limit = r.Num(node, "command.rate_limit", false, 0);
    act.command.delay_s = r.Num(node, "command.delay_s", false, 0);
    act.command.update_rate_hz = r.Num(node, "command.update_rate_hz", false, 0);

    act.controller.kp = r.Num(node, "controller.kp", false, 0);
    act.controller.ki = r.Num(node, "controller.ki", false, 0);
    act.controller.kd = r.Num(node, "controller.kd", false, 0);
    act.controller.integrator_limit = r.Num(node, "controller.integrator_limit", false, 0);
    act.controller.torque_limit_nm = r.Num(node, "controller.torque_limit_nm", false, 0);

    act.motor.torque_constant = r.Positive(node, "motor.torque_constant_nm_per_a", true, 0);
    act.motor.back_emf_constant = r.Num(node, "motor.back_emf_v_per_rad_s", false, 0);
    act.motor.resistance_ohm = r.Positive(node, "motor.resistance_ohm", true, 0);
    act.motor.rotor_inertia = r.Num(node, "motor.rotor_inertia_kg_m2", false, 0);
    act.motor.gear_ratio = r.Positive(node, "identity.reduction_ratio", true, 1);
    act.motor.gear_efficiency = r.InRange(node, "motor.gear_efficiency", false, 1.0, 0.01, 1.0);
    act.motor.backlash_rad = r.Num(node, "motor.backlash_rad", false, 0);
    act.motor.speed_limit_rad_s = r.Num(node, "limits.output_speed_rad_s", false, 0);
    act.motor.viscous_damping = r.Num(node, "motor.viscous_damping", false, 0);
    act.motor.coulomb_friction_nm = r.Num(node, "motor.coulomb_friction_nm", false, 0);

    // Back-EMF and torque constant are the same number in SI for an ideal machine, so
    // defaulting one to the other is correct rather than a guess.
    if (act.motor.back_emf_constant <= 0) act.motor.back_emf_constant = act.motor.torque_constant;

    act.protection.continuous_current_a = r.Positive(node, "limits.continuous_current_a", true, 0);
    act.protection.peak_current_a = r.Positive(node, "limits.peak_current_a", true, 0);
    act.protection.peak_duration_s = r.Num(node, "limits.peak_duration_s", false, 0);
    act.protection.bus_voltage_limit_v = r.Positive(node, "limits.bus_voltage_v", true, 0);

    if (act.protection.peak_current_a < act.protection.continuous_current_a) {
        r.Error("limits.peak_current_a", "peak current is below continuous current");
    }

    const std::string cutoff = r.Str(node, "limits.cutoff", false, "derate");
    if (cutoff == "derate") {
        act.protection.cutoff = ProtectionSpec::Cutoff::DERATE;
    } else if (cutoff == "fold_back") {
        act.protection.cutoff = ProtectionSpec::Cutoff::FOLD_BACK;
    } else if (cutoff == "disable") {
        act.protection.cutoff = ProtectionSpec::Cutoff::DISABLE;
    } else {
        r.Error("limits.cutoff", "expected derate, fold_back or disable, got \"" + cutoff + "\"");
    }

    act.thermal.ambient_c = r.Num(node, "thermal.ambient_c", false, 25);
    act.thermal.thermal_resistance_c_per_w = r.Num(node, "thermal.resistance_c_per_w", false, 0);
    act.thermal.thermal_capacitance_j_per_c = r.Num(node, "thermal.capacitance_j_per_c", false, 0);
    act.thermal.derate_start_c = r.Num(node, "thermal.derate_start_c", false, 0);
    act.thermal.cutoff_c = r.Num(node, "thermal.cutoff_c", false, 0);

    if (act.thermal.cutoff_c > 0 && act.thermal.derate_start_c > act.thermal.cutoff_c) {
        r.Error("thermal.derate_start_c", "derating starts above the cutoff temperature");
    }
    // Capacitance without resistance (or the reverse) integrates to either an infinite or
    // a frozen temperature, so a half-specified thermal model is rejected outright.
    const bool has_r = act.thermal.thermal_resistance_c_per_w > 0;
    const bool has_c = act.thermal.thermal_capacitance_j_per_c > 0;
    if (has_r != has_c) {
        r.Error("thermal", "specify both resistance_c_per_w and capacitance_j_per_c, or neither");
    }
    return act;
}

BatterySpec ReadBattery(Reader& r, const YAML::Node& root) {
    BatterySpec bat;
    const YAML::Node node = r.At(root, "battery", true);
    if (!node) return bat;

    bat.name = r.Str(node, "name", true, "");
    bat.capacity_ah = r.Positive(node, "capacity_ah", true, 0);
    bat.internal_resistance_ohm = r.Num(node, "internal_resistance_ohm", true, 0);
    bat.wiring_resistance_ohm = r.Num(node, "wiring_resistance_ohm", false, 0);
    bat.current_limit_a = r.Positive(node, "current_limit_a", true, 0);
    bat.auxiliary_load_w = r.Num(node, "auxiliary_load_w", false, 0);
    bat.initial_soc = r.InRange(node, "initial_soc", false, 1.0, 0.0, 1.0);
    bat.accept_regen = r.Bool(node, "accept_regen", false);
    bat.regen_current_limit_a = r.Num(node, "regen_current_limit_a", false, 0);
    bat.provenance = r.Str(node, "provenance.quality", false, "placeholder");

    const YAML::Node curve = r.At(node, "ocv_curve", true);
    if (curve && curve.IsSequence()) {
        for (size_t i = 0; i < curve.size(); i++) {
            const std::string path = "battery.ocv_curve[" + std::to_string(i) + "]";
            if (!curve[i].IsSequence() || curve[i].size() != 2) {
                r.Error(path, "expected [state_of_charge, volts]");
                continue;
            }
            bat.ocv_curve.emplace_back(curve[i][0].as<double>(), curve[i][1].as<double>());
        }
        std::sort(bat.ocv_curve.begin(), bat.ocv_curve.end());
    } else if (curve) {
        r.Error("battery.ocv_curve", "expected a list of [state_of_charge, volts] pairs");
    }
    if (bat.ocv_curve.empty()) {
        r.Error("battery.ocv_curve", "needs at least one [state_of_charge, volts] pair");
    }
    return bat;
}

PivotSpec ReadPivot(Reader& r, const YAML::Node& node, const std::string& path) {
    PivotSpec pivot;
    pivot.name = r.Str(node, "name", true, "");
    pivot.parent = r.Str(node, "parent", true, "");
    pivot.position = r.Vec(node, "position", true, VNULL);
    pivot.axis = r.Axis(node, "axis", true, ChVector3d(0, 1, 0));

    const YAML::Node limits = r.At(node, "limits_deg", false);
    if (limits) {
        if (!limits.IsSequence() || limits.size() != 2) {
            r.Error(path + ".limits_deg", "expected [lower, upper] in degrees");
        } else {
            double lower = limits[0].as<double>() * CH_DEG_TO_RAD;
            double upper = limits[1].as<double>() * CH_DEG_TO_RAD;
            if (lower >= upper) {
                r.Error(path + ".limits_deg", "lower limit must be below the upper limit");
            } else {
                pivot.limits_rad = {lower, upper};
            }
        }
    }

    pivot.spring_stiffness = r.Num(node, "spring_stiffness", false, 0);
    pivot.spring_damping = r.Num(node, "spring_damping", false, 0);
    pivot.rest_angle_rad = r.Num(node, "rest_angle_deg", false, 0) * CH_DEG_TO_RAD;

    pivot.mass_properties = ReadMassProperties(r, node, "mass_properties", true);
    pivot.visual = ReadVisualList(r, node, "visual");
    pivot.collision = ReadCollisionList(r, node, "collision");
    return pivot;
}

DifferentialSpec ReadDifferential(Reader& r, const YAML::Node& node) {
    DifferentialSpec diff;
    const YAML::Node child = r.At(node, "differential", false);
    if (!child) return diff;

    const std::string type = r.Str(child, "type", true, "none");
    if (type == "none") {
        diff.type = DifferentialSpec::Type::NONE;
        return diff;
    }
    if (type == "kinematic") {
        diff.type = DifferentialSpec::Type::KINEMATIC;
        return diff;
    }
    if (type != "linkage") {
        r.Error("rover.differential.type",
                "expected none, kinematic or linkage, got \"" + type + "\"");
        return diff;
    }

    diff.type = DifferentialSpec::Type::LINKAGE;
    diff.bar_position = r.Vec(child, "bar_position", true, VNULL);
    diff.left_link_position = r.Vec(child, "left_link_position", true, VNULL);
    diff.right_link_position = r.Vec(child, "right_link_position", true, VNULL);
    diff.bar_mass_properties = ReadMassProperties(r, child, "bar_mass_properties", true);
    diff.link_mass_properties = ReadMassProperties(r, child, "link_mass_properties", true);
    return diff;
}

RoverSpec ReadRover(Reader& r, const YAML::Node& root) {
    RoverSpec rover;
    const YAML::Node node = r.At(root, "rover", true);
    if (!node) return rover;

    rover.name = r.Str(node, "name", true, "");
    rover.source_urdf = r.Str(node, "source.urdf", false, "");
    rover.source_urdf_hash = r.Str(node, "source.urdf_hash", false, "");
    rover.body_friction = r.InRange(node, "body_friction", false, 0.8, 0.0, 3.0);
    rover.body_restitution = r.InRange(node, "body_restitution", false, 0.0, 0.0, 1.0);

    rover.chassis = ReadMassProperties(r, node, "chassis.mass_properties", true);
    const YAML::Node chassis = r.At(node, "chassis", true);
    if (chassis) {
        rover.chassis_visual = ReadVisualList(r, chassis, "visual");
        rover.chassis_collision = ReadCollisionList(r, chassis, "collision");
        if (rover.chassis_collision.empty()) {
            r.Error("rover.chassis.collision",
                    "the chassis needs deliberate collision geometry; an empty list lets the "
                    "rover pass through obstacles");
        }
        for (size_t i = 0; i < rover.chassis_collision.size(); i++) {
            if (rover.chassis_collision[i].type == CollisionShape::Type::MESH) {
                r.Error("rover.chassis.collision[" + std::to_string(i) + "]",
                        "triangle-mesh chassis collision is rejected: it is both slow and "
                        "unreliable against SCM. Use box, sphere, cylinder or convex_hull.");
            }
        }
    }

    const YAML::Node pivots = r.At(node, "pivots", true);
    if (pivots && pivots.IsSequence()) {
        for (size_t i = 0; i < pivots.size(); i++) {
            rover.pivots.push_back(
                ReadPivot(r, pivots[i], "rover.pivots[" + std::to_string(i) + "]"));
        }
    } else if (pivots) {
        r.Error("rover.pivots", "expected a list of suspension pivots");
    }

    rover.differential = ReadDifferential(r, node);

    const YAML::Node wheels = r.At(node, "wheels", true);
    if (wheels && wheels.IsSequence()) {
        for (size_t i = 0; i < wheels.size(); i++) {
            const std::string path = "rover.wheels[" + std::to_string(i) + "]";
            const YAML::Node& w = wheels[i];

            WheelStationSpec station;
            station.id = r.Str(w, "id", true, "");
            station.parent = r.Str(w, "parent", true, "");
            station.position = r.Vec(w, "position", true, VNULL);
            station.axis = r.Axis(w, "axis", true, ChVector3d(0, 1, 0));
            station.wheel_package = r.Str(w, "wheel_package", true, "");
            station.actuator = r.Str(w, "actuator", true, "");
            station.mount_roll_rad = r.Num(w, "mount_roll_deg", false, 0) * CH_DEG_TO_RAD;
            station.mirror = r.Bool(w, "mirror", false);
            (void)path;
            rover.wheels.push_back(station);
        }
    } else if (wheels) {
        r.Error("rover.wheels", "expected a list of wheel stations");
    }
    return rover;
}

SoilSpec ReadSoil(Reader& r, const YAML::Node& node) {
    SoilSpec soil;
    soil.name = r.Str(node, "name", true, "");
    soil.bekker_kphi = r.Positive(node, "bekker_kphi", true, 0);
    soil.bekker_kc = r.Num(node, "bekker_kc", true, 0);
    soil.bekker_n = r.InRange(node, "bekker_n", true, 1, 0.1, 3.0);
    soil.mohr_cohesion = r.Num(node, "mohr_cohesion", true, 0);
    soil.mohr_friction = r.InRange(node, "mohr_friction_deg", true, 0, 0, 70);
    soil.janosi_shear = r.Positive(node, "janosi_shear", true, 0);
    soil.elastic_k = r.Positive(node, "elastic_k", true, 0);
    soil.damping_r = r.Num(node, "damping_r", true, 0);
    soil.provenance = r.Str(node, "provenance.quality", false, "placeholder");
    return soil;
}

TerrainSpec ReadTerrain(Reader& r, const YAML::Node& root) {
    TerrainSpec terrain;
    const YAML::Node node = r.At(root, "terrain", true);
    if (!node) return terrain;

    terrain.name = r.Str(node, "name", true, "");
    terrain.version = r.Int(node, "version", false, 1);

    const std::string type = r.Str(node, "type", true, "scm_flat");
    static const std::map<std::string, TerrainSpec::Type> kTypes{
        {"rigid_flat", TerrainSpec::Type::RIGID_FLAT},
        {"rigid_mesh", TerrainSpec::Type::RIGID_MESH},
        {"scm_flat", TerrainSpec::Type::SCM_FLAT},
        {"scm_heightmap", TerrainSpec::Type::SCM_HEIGHTMAP},
        {"scm_mesh", TerrainSpec::Type::SCM_MESH},
        {"mixed", TerrainSpec::Type::MIXED},
    };
    auto found = kTypes.find(type);
    if (found == kTypes.end()) {
        r.Error("terrain.type", "unknown terrain type \"" + type + "\"");
    } else {
        terrain.type = found->second;
    }

    terrain.size_x = r.Positive(node, "size_x", true, 10);
    terrain.size_y = r.Positive(node, "size_y", true, 4);
    terrain.pitch_rad = r.InRange(node, "pitch_deg", false, 0, -45, 45) * CH_DEG_TO_RAD;
    terrain.roll_rad = r.InRange(node, "roll_deg", false, 0, -45, 45) * CH_DEG_TO_RAD;
    terrain.rigid_friction = r.InRange(node, "rigid_friction", false, 0.9, 0.0, 3.0);
    terrain.rigid_restitution = r.InRange(node, "rigid_restitution", false, 0.0, 0.0, 1.0);

    const bool deformable = terrain.type != TerrainSpec::Type::RIGID_FLAT &&
                            terrain.type != TerrainSpec::Type::RIGID_MESH;

    if (deformable) {
        terrain.grid_delta = r.InRange(node, "grid_delta", true, 0.04, 0.002, 0.5);
        terrain.bulldozing = r.Bool(node, "bulldozing", true);
        terrain.bulldozing_params = r.Vec(node, "bulldozing_params", false, ChVector3d(55, 1, 5));
        terrain.bulldozing_erosion_iters = r.Int(node, "bulldozing_erosion_iters", false, 6);
        terrain.active_domains = r.Bool(node, "active_domains", true);
        terrain.active_domain_margin = r.Num(node, "active_domain_margin", false, 0.25);

        const YAML::Node soils = r.At(node, "soils", true);
        if (soils && soils.IsSequence()) {
            for (size_t i = 0; i < soils.size(); i++)
                terrain.soils.push_back(ReadSoil(r, soils[i]));
        } else if (soils) {
            r.Error("terrain.soils", "expected a list of soil definitions");
        }
        if (terrain.soils.empty()) {
            r.Error("terrain.soils", "a deformable terrain needs at least one soil definition");
        }

        terrain.default_soil = r.Str(node, "default_soil", true, "");

        const YAML::Node regions = r.At(node, "regions", false);
        if (regions && regions.IsSequence()) {
            for (size_t i = 0; i < regions.size(); i++) {
                const YAML::Node& reg = regions[i];
                SoilRegion region;
                region.x_min = r.Num(reg, "x_min", true, 0);
                region.x_max = r.Num(reg, "x_max", true, 0);
                region.y_min = r.Num(reg, "y_min", true, 0);
                region.y_max = r.Num(reg, "y_max", true, 0);
                region.soil = r.Str(reg, "soil", true, "");
                if (region.x_min >= region.x_max || region.y_min >= region.y_max) {
                    r.Error("terrain.regions[" + std::to_string(i) + "]",
                            "region bounds are empty or inverted");
                }
                terrain.regions.push_back(region);
            }
        }
    }

    if (terrain.type == TerrainSpec::Type::SCM_HEIGHTMAP) {
        terrain.heightmap_file = r.File(node, "heightmap_file", true);
        terrain.height_min = r.Num(node, "height_min", true, 0);
        terrain.height_max = r.Num(node, "height_max", true, 1);
        if (terrain.height_min >= terrain.height_max) {
            r.Error("terrain.height_min", "height_min must be below height_max");
        }
    }
    if (terrain.type == TerrainSpec::Type::SCM_MESH ||
        terrain.type == TerrainSpec::Type::RIGID_MESH) {
        terrain.mesh_file = r.File(node, "mesh_file", true);
    }

    const YAML::Node obstacles = r.At(node, "obstacles", false);
    if (obstacles && obstacles.IsSequence()) {
        for (size_t i = 0; i < obstacles.size(); i++) {
            const std::string path = "terrain.obstacles[" + std::to_string(i) + "]";
            const YAML::Node& obs = obstacles[i];

            ObstacleSpec obstacle;
            obstacle.shape = ReadCollisionShape(r, obs, path);
            obstacle.position = r.Vec(obs, "position", true, VNULL);
            obstacle.rotation = r.Rotation(obs, "rotation");
            obstacle.fixed = r.Bool(obs, "fixed", true);
            obstacle.friction = r.InRange(obs, "friction", false, 0.9, 0.0, 3.0);
            if (!obstacle.fixed) obstacle.mass = r.Positive(obs, "mass", true, 0);

            // The shape's own position is relative to the obstacle body, so an obstacle
            // that also sets it would be placed twice.
            obstacle.shape.position = VNULL;
            terrain.obstacles.push_back(obstacle);
        }
    }

    terrain.source_file = r.Str(node, "source.file", false, "");
    terrain.source_hash = r.Str(node, "source.hash", false, "");
    terrain.source_crs = r.Str(node, "source.crs", false, "");
    return terrain;
}

ScenarioSpec ReadScenario(Reader& r, const YAML::Node& root) {
    ScenarioSpec scenario;
    const YAML::Node node = r.At(root, "scenario", true);
    if (!node) return scenario;

    scenario.name = r.Str(node, "name", true, "");
    scenario.version = r.Int(node, "version", false, 1);
    scenario.initial_position = r.Vec(node, "initial_position", false, VNULL);
    scenario.initial_yaw_rad = r.Num(node, "initial_yaw_deg", false, 0) * CH_DEG_TO_RAD;
    scenario.drop_height = r.InRange(node, "drop_height", false, 0.05, 0.0, 2.0);
    scenario.end_time = r.Positive(node, "end_time", true, 20);
    scenario.warmup_time = r.Num(node, "warmup_time", false, 0.5);
    scenario.sample_rate_hz = r.Positive(node, "sample_rate_hz", false, 200);
    scenario.require_provenance = r.Str(node, "require_provenance", false, "placeholder");

    if (scenario.warmup_time >= scenario.end_time) {
        r.Error("scenario.warmup_time", "warm-up covers the whole run, leaving no samples");
    }

    const YAML::Node driver = r.At(node, "driver", true);
    if (driver) {
        const std::string type = r.Str(driver, "type", true, "constant");
        static const std::map<std::string, DriverSpec::Type> kTypes{
            {"constant", DriverSpec::Type::CONSTANT},     {"ramp", DriverSpec::Type::RAMP},
            {"slip_sweep", DriverSpec::Type::SLIP_SWEEP}, {"replay", DriverSpec::Type::REPLAY},
            {"stopped", DriverSpec::Type::STOPPED},
        };
        auto found = kTypes.find(type);
        if (found == kTypes.end()) {
            r.Error("scenario.driver.type", "unknown driver type \"" + type + "\"");
        } else {
            scenario.driver.type = found->second;
        }

        scenario.driver.value = r.Num(driver, "value", type != "stopped" && type != "replay", 0);
        scenario.driver.ramp_time = r.Num(driver, "ramp_time", false, 1.0);
        scenario.driver.steering = r.InRange(driver, "steering", false, 0, -1, 1);
        scenario.driver.sweep_hold_s = r.Positive(driver, "sweep_hold_s", false, 5.0);

        if (scenario.driver.type == DriverSpec::Type::SLIP_SWEEP) {
            const YAML::Node values = r.At(driver, "sweep_values", true);
            if (values && values.IsSequence()) {
                for (const auto& v : values) scenario.driver.sweep_values.push_back(v.as<double>());
            }
            if (scenario.driver.sweep_values.empty()) {
                r.Error("scenario.driver.sweep_values", "a slip sweep needs at least one value");
            }
        }
        if (scenario.driver.type == DriverSpec::Type::REPLAY) {
            scenario.driver.replay_file = r.File(driver, "replay_file", true);
        }
    }

    const YAML::Node termination = r.At(node, "termination", false);
    if (termination && termination.IsSequence()) {
        static const std::map<std::string, TerminationSpec::Type> kTypes{
            {"distance", TerminationSpec::Type::DISTANCE},
            {"rollover", TerminationSpec::Type::ROLLOVER},
            {"immobility", TerminationSpec::Type::IMMOBILITY},
            {"sinkage", TerminationSpec::Type::SINKAGE},
            {"motor_temperature", TerminationSpec::Type::MOTOR_TEMPERATURE},
            {"battery_empty", TerminationSpec::Type::BATTERY_EMPTY},
        };
        for (size_t i = 0; i < termination.size(); i++) {
            const YAML::Node& t = termination[i];
            const std::string type = r.Str(t, "type", true, "");
            auto found = kTypes.find(type);
            if (found == kTypes.end()) {
                r.Error("scenario.termination[" + std::to_string(i) + "].type",
                        "unknown termination type \"" + type + "\"");
                continue;
            }
            TerminationSpec rule;
            rule.type = found->second;
            rule.threshold = r.Num(t, "threshold", true, 0);
            rule.window_s = r.Num(t, "window_s", rule.type == TerminationSpec::Type::IMMOBILITY, 0);
            rule.pass = r.Bool(t, "pass", rule.type == TerminationSpec::Type::DISTANCE);
            scenario.termination.push_back(rule);
        }
    }

    const YAML::Node checks = r.At(node, "checks", false);
    if (checks && checks.IsSequence()) {
        static const std::map<std::string, CheckSpec::Op> kOps{
            {"lt", CheckSpec::Op::LT},
            {"le", CheckSpec::Op::LE},
            {"gt", CheckSpec::Op::GT},
            {"ge", CheckSpec::Op::GE},
        };
        for (size_t i = 0; i < checks.size(); i++) {
            const YAML::Node& c = checks[i];
            CheckSpec check;
            check.metric = r.Str(c, "metric", true, "");
            check.value = r.Num(c, "value", true, 0);
            check.description = r.Str(c, "description", false, "");

            const std::string op = r.Str(c, "op", true, "lt");
            auto found = kOps.find(op);
            if (found == kOps.end()) {
                r.Error("scenario.checks[" + std::to_string(i) + "].op",
                        "expected lt, le, gt or ge, got \"" + op + "\"");
            } else {
                check.op = found->second;
            }
            scenario.checks.push_back(check);
        }
    }

    const YAML::Node groups = r.At(node, "metric_groups", false);
    if (groups && groups.IsSequence()) {
        for (const auto& g : groups) scenario.metric_groups.push_back(g.as<std::string>());
    }
    return scenario;
}

// ---------------------------------------------------------------------------- //
//                            cross-section validation                          //
// ---------------------------------------------------------------------------- //

void CheckReferences(Reader& r, const RunManifest& manifest) {
    std::set<std::string> body_names{"chassis"};
    for (const auto& pivot : manifest.rover.pivots) {
        if (!body_names.insert(pivot.name).second) {
            r.Error("rover.pivots", "duplicate body name \"" + pivot.name + "\"");
        }
    }
    // Pivot parents are checked against the names declared *before* them, so the
    // suspension chain has to be listed root-first and cannot contain a cycle.
    std::set<std::string> declared{"chassis"};
    for (const auto& pivot : manifest.rover.pivots) {
        if (!declared.count(pivot.parent)) {
            r.Error("rover.pivots." + pivot.name, "parent \"" + pivot.parent +
                                                      "\" is not the chassis and is not declared "
                                                      "earlier in the list");
        }
        declared.insert(pivot.name);
    }

    std::set<std::string> wheel_ids;
    for (const auto& station : manifest.rover.wheels) {
        if (!wheel_ids.insert(station.id).second) {
            r.Error("rover.wheels", "duplicate wheel id \"" + station.id + "\"");
        }
        if (!body_names.count(station.parent)) {
            r.Error("rover.wheels." + station.id,
                    "parent \"" + station.parent + "\" is not a declared body");
        }
        if (!manifest.wheels.count(station.wheel_package)) {
            r.Error("rover.wheels." + station.id,
                    "wheel package \"" + station.wheel_package + "\" is not in this manifest");
        }
        if (!manifest.actuators.count(station.actuator)) {
            r.Error("rover.wheels." + station.id,
                    "actuator \"" + station.actuator + "\" is not in this manifest");
        }
    }
    if (manifest.rover.wheels.empty()) {
        r.Error("rover.wheels", "a rover needs at least one driven wheel");
    }

    std::set<std::string> soil_names;
    for (const auto& soil : manifest.terrain.soils) {
        if (!soil_names.insert(soil.name).second) {
            r.Error("terrain.soils", "duplicate soil name \"" + soil.name + "\"");
        }
    }
    if (!manifest.terrain.soils.empty() && !soil_names.count(manifest.terrain.default_soil)) {
        r.Error("terrain.default_soil",
                "\"" + manifest.terrain.default_soil + "\" is not one of the defined soils");
    }
    for (size_t i = 0; i < manifest.terrain.regions.size(); i++) {
        if (!soil_names.count(manifest.terrain.regions[i].soil)) {
            r.Error("terrain.regions[" + std::to_string(i) + "].soil",
                    "\"" + manifest.terrain.regions[i].soil + "\" is not one of the defined soils");
        }
    }

    if (manifest.terrain.active_domains && !manifest.terrain.soils.empty()) {
        // SCM only ray casts inside the active domains, so a domain narrower than the
        // wheel silently drops part of the contact patch.
        for (const auto& station : manifest.rover.wheels) {
            auto wheel = manifest.wheels.find(station.wheel_package);
            if (wheel == manifest.wheels.end()) continue;
            if (manifest.terrain.active_domain_margin < wheel->second.radius * 0.1) {
                r.Error("terrain.active_domain_margin",
                        "margin is small relative to wheel \"" + wheel->second.name +
                            "\"; SCM ray casting may miss the edges of the contact patch");
                break;
            }
        }
    }

    if (manifest.rover.differential.type != DifferentialSpec::Type::NONE) {
        int rockers = 0;
        for (const auto& pivot : manifest.rover.pivots) {
            if (pivot.parent == "chassis") rockers++;
        }
        if (rockers != 2) {
            r.Error("rover.differential",
                    "a differential couples exactly two chassis pivots, but " +
                        std::to_string(rockers) + " were declared");
        }
    }

    // A check naming a metric the run never records always passes, which is the worst
    // possible failure mode for a pass/fail gate.
    for (const auto& check : manifest.scenario.checks) {
        if (check.metric.empty()) continue;
        if (check.metric.find('.') == std::string::npos) {
            r.Error("scenario.checks",
                    "metric \"" + check.metric + "\" is not a <group>.<name> summary key");
        }
    }
}

void CheckProvenance(Reader& r, const RunManifest& manifest) {
    const int required = ProvenanceRank(manifest.scenario.require_provenance);
    if (required <= 0) return;

    auto check = [&](const std::string& what, const std::string& provenance) {
        if (ProvenanceRank(provenance) < required) {
            r.Error("scenario.require_provenance", what + " has provenance \"" + provenance +
                                                       "\", weaker than the required \"" +
                                                       manifest.scenario.require_provenance + "\"");
        }
    };

    check("rover.chassis", manifest.rover.chassis.provenance);
    for (const auto& pivot : manifest.rover.pivots) {
        check("rover.pivots." + pivot.name, pivot.mass_properties.provenance);
    }
    for (const auto& [name, wheel] : manifest.wheels) {
        check("wheels." + name, wheel.mass_properties.provenance);
    }
    for (const auto& [name, actuator] : manifest.actuators) {
        check("actuators." + name, actuator.provenance);
    }
    for (const auto& soil : manifest.terrain.soils) {
        check("terrain.soils." + soil.name, soil.provenance);
    }
    check("battery", manifest.battery.provenance);
}

}  // namespace

// ---------------------------------------------------------------------------- //
//                                  public API                                  //
// ---------------------------------------------------------------------------- //

ConfigError::ConfigError(std::vector<std::string> problems)
    : std::runtime_error([&] {
          std::ostringstream out;
          out << problems.size() << (problems.size() == 1 ? " problem" : " problems")
              << " in the run manifest:";
          for (const auto& problem : problems) out << "\n  - " << problem;
          return out.str();
      }()),
      m_problems(std::move(problems)) {}

int ProvenanceRank(const std::string& provenance) {
    if (provenance == "measured") return 4;
    if (provenance == "datasheet") return 3;
    if (provenance == "cad") return 2;
    if (provenance == "estimated") return 1;
    return 0;  // "placeholder" and anything unrecognised
}

const WheelSpec& RunManifest::Wheel(const std::string& name) const {
    auto found = wheels.find(name);
    if (found == wheels.end()) throw std::runtime_error("no wheel package named \"" + name + "\"");
    return found->second;
}

const ActuatorSpec& RunManifest::Actuator(const std::string& name) const {
    auto found = actuators.find(name);
    if (found == actuators.end()) throw std::runtime_error("no actuator named \"" + name + "\"");
    return found->second;
}

const SoilSpec& RunManifest::Soil(const std::string& name) const {
    for (const auto& soil : terrain.soils) {
        if (soil.name == name) return soil;
    }
    throw std::runtime_error("no soil named \"" + name + "\"");
}

RunManifest LoadManifest(const std::string& manifest_path) {
    fs::path path(manifest_path);
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        throw ConfigError({"manifest not found: " + path.string()});
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const YAML::Exception& e) {
        throw ConfigError({std::string("manifest is not valid YAML: ") + e.what()});
    }
    if (!root.IsMap()) {
        throw ConfigError({"manifest must be a YAML mapping"});
    }

    Reader r(root, fs::absolute(path, ec).parent_path());

    RunManifest manifest;
    manifest.schema_version = r.Int(root, "schema_version", true, 0);
    if (manifest.schema_version != kManifestSchemaVersion) {
        throw ConfigError({"manifest schema_version is " + std::to_string(manifest.schema_version) +
                           ", but this build reads " + std::to_string(kManifestSchemaVersion) +
                           ". Regenerate the manifest with a matching `sim chrono`."});
    }

    manifest.run = ReadRun(r, root);
    manifest.run.manifest_path = fs::absolute(path, ec).string();
    manifest.simulation = ReadSimulation(r, root);

    const YAML::Node wheels = r.At(root, "wheels", true);
    if (wheels && wheels.IsSequence()) {
        for (size_t i = 0; i < wheels.size(); i++) {
            WheelSpec wheel = ReadWheel(r, wheels[i], "wheels[" + std::to_string(i) + "]");
            if (!wheel.name.empty()) manifest.wheels[wheel.name] = std::move(wheel);
        }
    } else if (wheels) {
        r.Error("wheels", "expected a list of resolved wheel packages");
    }

    const YAML::Node actuators = r.At(root, "actuators", true);
    if (actuators && actuators.IsSequence()) {
        for (const auto& node : actuators) {
            ActuatorSpec actuator = ReadActuator(r, node);
            if (!actuator.name.empty()) manifest.actuators[actuator.name] = std::move(actuator);
        }
    } else if (actuators) {
        r.Error("actuators", "expected a list of resolved actuator definitions");
    }

    manifest.battery = ReadBattery(r, root);
    manifest.rover = ReadRover(r, root);
    manifest.terrain = ReadTerrain(r, root);
    manifest.scenario = ReadScenario(r, root);

    CheckReferences(r, manifest);
    CheckProvenance(r, manifest);

    if (r.Failed()) throw ConfigError(r.Problems());
    return manifest;
}

}  // namespace trickfire
