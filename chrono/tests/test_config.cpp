// Manifest loading and validation.
//
// Most of these assert that a *bad* manifest is rejected. That is the point of the
// loader: the development plan's argument is that a wheel at ten times its real scale,
// or a soil nobody measured, will otherwise run happily and produce confident, wrong
// numbers. A validator that only proves good input works has not been tested.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "check.hpp"
#include "config/load.hpp"

using namespace trickfire;
namespace fs = std::filesystem;

namespace {

fs::path TempDir() {
    static fs::path root = fs::temp_directory_path() / "trickfire_config_tests";
    fs::create_directories(root);
    return root;
}

/// A minimal manifest that loads cleanly. Tests mutate one thing at a time from here, so
/// each failure message names exactly one cause.
std::string BaseManifest(const std::string& mesh_path) {
    return R"(schema_version: 1
run: {id: test, seed: 1, output_dir: ., headless: true}
simulation: {step_size: 0.001, threads: 1, contact_method: smc, up_axis: z, gravity: 9.81}
wheels:
  - name: w
    radius: 0.2
    width: 0.1
    roll_axis: [0, 1, 0]
    mass_properties: {mass: 2.0, com: [0, 0, 0], inertia_xx: [0.02, 0.04, 0.02], provenance: measured}
    visual: {file: )" +
           mesh_path + R"(}
    collision: [{type: cylinder, radius: 0.2, length: 0.1}]
actuators:
  - name: a
    identity: {model: m, hardware_revision: r, reduction_ratio: 9.0}
    motor: {torque_constant_nm_per_a: 0.09, resistance_ohm: 0.35}
    limits: {bus_voltage_v: 24.0, continuous_current_a: 18.0, peak_current_a: 40.0}
battery:
  name: b
  ocv_curve: [[0.0, 20.0], [1.0, 25.0]]
  capacity_ah: 10.0
  internal_resistance_ohm: 0.05
  current_limit_a: 100.0
rover:
  name: r
  chassis:
    mass_properties: {mass: 10.0, com: [0, 0, 0], inertia_xx: [1.0, 1.0, 1.0], provenance: cad}
    collision: [{type: box, dimensions: [0.5, 0.4, 0.2]}]
  pivots:
    - name: rocker_left
      parent: chassis
      position: [0, 0.2, 0.1]
      axis: [0, 1, 0]
      mass_properties: {mass: 1.0, com: [0, 0, 0], inertia_xx: [0.1, 0.1, 0.1], provenance: cad}
    - name: rocker_right
      parent: chassis
      position: [0, -0.2, 0.1]
      axis: [0, 1, 0]
      mass_properties: {mass: 1.0, com: [0, 0, 0], inertia_xx: [0.1, 0.1, 0.1], provenance: cad}
  differential: {type: none}
  wheels:
    - {id: fl, parent: rocker_left, position: [0.3, 0.2, 0], axis: [0, 1, 0], wheel_package: w, actuator: a}
    - {id: fr, parent: rocker_right, position: [0.3, -0.2, 0], axis: [0, 1, 0], wheel_package: w, actuator: a}
terrain:
  name: t
  type: rigid_flat
  size_x: 10.0
  size_y: 4.0
scenario:
  name: s
  end_time: 1.0
  warmup_time: 0.1
  sample_rate_hz: 100
  driver: {type: constant, value: 1.0}
)";
}

/// Write a manifest, load it, and report what happened.
struct Outcome {
    bool loaded = false;
    std::vector<std::string> problems;
    RunManifest manifest;
};

Outcome Load(const std::string& text, const std::string& name) {
    const fs::path dir = TempDir() / name;
    fs::create_directories(dir);

    // A real mesh file, because the loader checks that referenced files exist.
    const fs::path mesh = dir / "visual.obj";
    std::ofstream(mesh) << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";

    const fs::path path = dir / "manifest.yaml";
    std::ofstream(path) << text;

    Outcome outcome;
    try {
        outcome.manifest = LoadManifest(path.string());
        outcome.loaded = true;
    } catch (const ConfigError& e) {
        outcome.problems = e.Problems();
    }
    return outcome;
}

/// Replace the first occurrence of `from` with `to`, so a test can corrupt one field.
std::string Replace(std::string text, const std::string& from, const std::string& to) {
    const size_t at = text.find(from);
    if (at == std::string::npos) {
        std::printf("FAIL  test setup: \"%s\" not found in the base manifest\n", from.c_str());
        check::failures++;
        check::total++;
        return text;
    }
    return text.replace(at, from.size(), to);
}

bool Mentions(const Outcome& outcome, const std::string& fragment) {
    for (const auto& problem : outcome.problems) {
        if (problem.find(fragment) != std::string::npos) return true;
    }
    return false;
}

void TestBaseLoads() {
    const auto outcome = Load(BaseManifest("visual.obj"), "base");
    if (!outcome.loaded) {
        for (const auto& problem : outcome.problems) std::printf("      %s\n", problem.c_str());
    }
    check::True(outcome.loaded, "a well-formed manifest loads");
    if (!outcome.loaded) return;

    check::True(outcome.manifest.rover.wheels.size() == 2, "both wheel stations are read");
    check::True(outcome.manifest.wheels.count("w") == 1, "the wheel package is indexed by name");
    check::Near(outcome.manifest.simulation.step_size, 1e-3, 1e-12, "the step size is read");
    check::True(outcome.manifest.terrain.type == TerrainSpec::Type::RIGID_FLAT,
                "the terrain type is read");
}

void TestRelativePathsResolve() {
    const auto outcome = Load(BaseManifest("visual.obj"), "relpath");
    check::True(outcome.loaded, "a mesh path relative to the manifest resolves");
    if (outcome.loaded) {
        check::True(fs::path(outcome.manifest.Wheel("w").visual.file).is_absolute(),
                    "and is made absolute, so the run does not depend on the working directory");
    }
}

void TestMissingFileRejected() {
    const auto outcome = Load(BaseManifest("nope.obj"), "missing_mesh");
    check::True(!outcome.loaded, "a manifest naming a mesh that does not exist is rejected");
    check::True(Mentions(outcome, "file not found"), "and says the file is missing");
}

void TestPlaceholderRejected() {
    const auto text = Replace(BaseManifest("visual.obj"), "torque_constant_nm_per_a: 0.09",
                              "torque_constant_nm_per_a: REPLACE_WITH_DATASHEET_VALUE");
    const auto outcome = Load(text, "placeholder");
    check::True(!outcome.loaded, "an unmeasured REPLACE_WITH value is rejected");
    check::True(Mentions(outcome, "REPLACE_WITH"), "and the message names the placeholder");
}

void TestOversizedWheelRejected() {
    // The headline case from the plan: a wheel exported in millimetres and read as metres
    // comes out a thousand times too large, and would otherwise run.
    const auto text = Replace(BaseManifest("visual.obj"), "radius: 0.2\n    width: 0.1",
                              "radius: 200.0\n    width: 100.0");
    const auto outcome = Load(text, "oversize");
    check::True(!outcome.loaded, "a wheel far outside a plausible size is rejected");
    check::True(Mentions(outcome, "radius"), "and the message names the radius");
}

void TestZeroMassRejected() {
    const auto text = Replace(BaseManifest("visual.obj"), "{mass: 2.0, com: [0, 0, 0]",
                              "{mass: 0.0, com: [0, 0, 0]");
    const auto outcome = Load(text, "zero_mass");
    check::True(!outcome.loaded, "a zero-mass wheel is rejected");
}

void TestTiltedAxisRejected() {
    // An axis that is nearly but not exactly cardinal produces a rover that drives in a
    // slow curve, which is far harder to notice than a load error.
    const auto text =
        Replace(BaseManifest("visual.obj"),
                "{id: fl, parent: rocker_left, position: [0.3, 0.2, 0], axis: [0, 1, 0]",
                "{id: fl, parent: rocker_left, position: [0.3, 0.2, 0], axis: [0.1, 1, 0]");
    const auto outcome = Load(text, "tilted_axis");
    check::True(!outcome.loaded, "a non-cardinal wheel axis is rejected");
    check::True(Mentions(outcome, "cardinal"), "and the message explains what is wanted");
}

void TestImpossibleInertiaRejected() {
    // Principal moments that break the triangle inequality cannot describe a real body,
    // and are the usual symptom of a mismatched-unit CAD export.
    const auto text = Replace(BaseManifest("visual.obj"), "inertia_xx: [0.02, 0.04, 0.02]",
                              "inertia_xx: [0.01, 0.01, 1.0]");
    const auto outcome = Load(text, "bad_inertia");
    check::True(!outcome.loaded, "an inertia tensor that no real body could have is rejected");
    check::True(Mentions(outcome, "triangle inequality"), "and the message says why");
}

void TestMeshChassisCollisionRejected() {
    const auto text =
        Replace(BaseManifest("visual.obj"), "collision: [{type: box, dimensions: [0.5, 0.4, 0.2]}]",
                "collision: [{type: mesh, file: visual.obj}]");
    const auto outcome = Load(text, "mesh_chassis");
    check::True(!outcome.loaded, "triangle-mesh chassis collision is rejected");
    check::True(Mentions(outcome, "rejected"), "and the message explains the alternative");
}

void TestDanglingReferencesRejected() {
    const auto text =
        Replace(BaseManifest("visual.obj"), "wheel_package: w, actuator: a}\n    - {id: fr",
                "wheel_package: nonexistent, actuator: a}\n    - {id: fr");
    const auto outcome = Load(text, "dangling_wheel");
    check::True(!outcome.loaded,
                "a station naming a wheel package that is not present is rejected");
    check::True(Mentions(outcome, "nonexistent"), "and the message names it");
}

void TestUnknownParentRejected() {
    const auto text =
        Replace(BaseManifest("visual.obj"), "parent: rocker_left", "parent: not_a_body");
    const auto outcome = Load(text, "bad_parent");
    check::True(!outcome.loaded, "a station hanging off a body that does not exist is rejected");
}

void TestDuplicateWheelIdRejected() {
    const auto text = Replace(BaseManifest("visual.obj"), "{id: fr,", "{id: fl,");
    const auto outcome = Load(text, "duplicate_id");
    check::True(!outcome.loaded, "two stations with the same id are rejected");
    check::True(Mentions(outcome, "duplicate"), "and the message says so");
}

void TestDifferentialNeedsTwoRockers() {
    auto text = Replace(BaseManifest("visual.obj"), "differential: {type: none}",
                        "differential: {type: kinematic}");
    text = Replace(text, "parent: chassis\n      position: [0, -0.2, 0.1]",
                   "parent: rocker_left\n      position: [0, -0.2, 0.1]");
    const auto outcome = Load(text, "diff_rockers");
    check::True(!outcome.loaded, "a differential with only one chassis pivot is rejected");
    check::True(Mentions(outcome, "differential"), "and the message names the differential");
}

void TestDeformableTerrainNeedsSoil() {
    const auto text = Replace(BaseManifest("visual.obj"), "type: rigid_flat",
                              "type: scm_flat\n  grid_delta: 0.04");
    const auto outcome = Load(text, "no_soil");
    check::True(!outcome.loaded, "a deformable terrain with no soil is rejected");
    check::True(Mentions(outcome, "soil"), "and the message asks for one");
}

void TestWarmupCannotSwallowTheRun() {
    const auto text = Replace(BaseManifest("visual.obj"), "warmup_time: 0.1", "warmup_time: 2.0");
    const auto outcome = Load(text, "warmup");
    check::True(!outcome.loaded, "a warm-up longer than the run is rejected");
}

void TestSchemaVersionMismatch() {
    const auto text =
        Replace(BaseManifest("visual.obj"), "schema_version: 1", "schema_version: 99");
    const auto outcome = Load(text, "version");
    check::True(!outcome.loaded, "a manifest from a newer CLI is rejected rather than half-read");
    check::True(Mentions(outcome, "schema_version"), "and the message names the version");
}

void TestProvenanceGate() {
    // A scenario can demand that its inputs were actually measured. The base manifest's
    // chassis is "cad", so requiring "measured" must stop the run.
    const auto text =
        Replace(BaseManifest("visual.obj"), "driver: {type: constant, value: 1.0}",
                "driver: {type: constant, value: 1.0}\n  require_provenance: measured");
    const auto outcome = Load(text, "provenance");
    check::True(!outcome.loaded, "a scenario requiring measured inputs refuses estimated ones");
    check::True(Mentions(outcome, "provenance"), "and the message explains which input");
}

void TestEveryProblemIsReported() {
    // The loader collects problems rather than stopping at the first, so one editing pass
    // can fix them all.
    auto text = Replace(BaseManifest("visual.obj"), "radius: 0.2\n    width: 0.1",
                        "radius: -1.0\n    width: 0.1");
    text = Replace(text, "capacity_ah: 10.0", "capacity_ah: -5.0");
    const auto outcome = Load(text, "multiple");
    check::True(!outcome.loaded, "a manifest with several problems is rejected");
    check::Greater(static_cast<double>(outcome.problems.size()), 1.0,
                   "and every problem is reported, not only the first");
}

void TestProvenanceRanking() {
    check::True(ProvenanceRank("measured") > ProvenanceRank("datasheet"),
                "measured outranks datasheet");
    check::True(ProvenanceRank("datasheet") > ProvenanceRank("cad"), "datasheet outranks cad");
    check::True(ProvenanceRank("cad") > ProvenanceRank("estimated"), "cad outranks estimated");
    check::True(ProvenanceRank("estimated") > ProvenanceRank("placeholder"),
                "estimated outranks placeholder");
    check::True(ProvenanceRank("nonsense") == 0, "an unrecognised provenance ranks lowest");
}

}  // namespace

int main() {
    TestBaseLoads();
    TestRelativePathsResolve();
    TestMissingFileRejected();
    TestPlaceholderRejected();
    TestOversizedWheelRejected();
    TestZeroMassRejected();
    TestTiltedAxisRejected();
    TestImpossibleInertiaRejected();
    TestMeshChassisCollisionRejected();
    TestDanglingReferencesRejected();
    TestUnknownParentRejected();
    TestDuplicateWheelIdRejected();
    TestDifferentialNeedsTwoRockers();
    TestDeformableTerrainNeedsSoil();
    TestWarmupCannotSwallowTheRun();
    TestSchemaVersionMismatch();
    TestProvenanceGate();
    TestEveryProblemIsReported();
    TestProvenanceRanking();

    fs::remove_all(TempDir());
    return check::Finish("config");
}
