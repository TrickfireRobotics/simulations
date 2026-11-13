#include "wheel_catalog.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "chrono/core/ChDataPath.h"

namespace trickfire {

namespace {

namespace fs = std::filesystem;

std::string Trim(const std::string& s) {
    size_t begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

/// Applies "<obj_path with .obj replaced by .toml>" onto `option`, if that file exists.
/// Understands flat "key = value" lines with numeric values; everything else (blank
/// lines, "# comments", unknown keys) is ignored, so the file stays valid TOML even
/// though this is a hand-rolled parser rather than a real one.
void ApplyWheelConfig(const fs::path& obj_path, WheelOption& option) {
    fs::path config_path = obj_path;
    config_path.replace_extension(".toml");

    std::ifstream in(config_path);
    if (!in) return;

    std::string line;
    while (std::getline(in, line)) {
        line = line.substr(0, line.find('#'));

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = Trim(line.substr(0, eq));
        std::string value = Trim(line.substr(eq + 1));

        std::istringstream value_stream(value);
        double number;
        if (!(value_stream >> number)) continue;

        if (key == "scale")
            option.mesh_scale = number;
        else if (key == "rotation_deg")
            option.mesh_rotation_deg = number;
        else if (key == "mass")
            option.mass = number;
        else if (key == "inertia")
            option.inertia = number;
        else if (key == "radius")
            option.radius = number;
    }
}

}  // namespace

std::vector<WheelOption> BuildWheelCatalog() {
    std::vector<WheelOption> catalog;
    catalog.push_back({"cylindrical", false, "", 1.0, 0.0, {}, {}, {}});
    catalog.push_back({"lugged",
                       true,
                       chrono::GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"),
                       1.0,
                       0.0,
                       {},
                       {},
                       {}});

    fs::path wheels_dir = fs::path(chrono::GetChronoDataPath()) / "models" / "wheels";

    std::error_code ec;
    if (fs::is_directory(wheels_dir, ec)) {
        std::vector<fs::path> obj_files;
        for (const auto& entry : fs::directory_iterator(wheels_dir, ec)) {
            if (entry.path().extension() == ".obj") obj_files.push_back(entry.path());
        }
        std::sort(obj_files.begin(), obj_files.end());

        for (const auto& path : obj_files) {
            WheelOption option;
            option.name = path.stem().string();
            option.is_mesh = true;
            option.mesh_path = path.string();
            ApplyWheelConfig(path, option);
            catalog.push_back(std::move(option));
        }
    }

    return catalog;
}

int FindWheelOption(const std::vector<WheelOption>& catalog, const std::string& name) {
    for (size_t i = 0; i < catalog.size(); i++) {
        if (catalog[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace trickfire
