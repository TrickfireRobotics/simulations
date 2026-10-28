#include <cstdlib>
#include <iostream>
#include <string>

#include "chrono/core/ChDataPath.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "config/load.hpp"
#include "reporting/environment.hpp"
#include "simulation.hpp"

#ifdef TRICKFIRE_HAS_GUI
#include "gui/viewer.hpp"
#endif

using namespace trickfire;

namespace {

constexpr const char* kUsage = R"(usage: sim --manifest <run.yaml> [options]

The simulator reads one resolved run manifest and nothing else. Manifests are produced
by `sim chrono run` / `sim chrono sweep`, which merge the scenario, rover, wheel,
actuator, battery and terrain configs and write the result next to the results.

options:
  --manifest <path>   resolved run manifest (required)
  --headless          run without a window, overriding the manifest
  --gui               open the viewer, overriding the manifest
  --validate          load and validate the manifest, then exit
  --environment       print the Chrono commit this binary runs against, then exit
  -h, --help          this message
)";

struct Options {
    std::string manifest;
    bool headless = false;
    bool force_headless = false;
    bool force_gui = false;
    bool validate_only = false;
    bool print_environment = false;
};

/// Parse argv, returning false when the program should stop (help, or a bad argument).
bool ParseArguments(int argc, char** argv, Options& options, int& exit_code) {
    for (int i = 1; i < argc; i++) {
        const std::string argument = argv[i];

        if (argument == "-h" || argument == "--help") {
            std::cout << kUsage;
            exit_code = 0;
            return false;
        }
        if (argument == "--headless") {
            options.force_headless = true;
        } else if (argument == "--gui") {
            options.force_gui = true;
        } else if (argument == "--validate") {
            options.validate_only = true;
        } else if (argument == "--environment") {
            options.print_environment = true;
        } else if (argument == "--manifest") {
            if (i + 1 >= argc) {
                std::cerr << "--manifest needs a path\n";
                exit_code = 2;
                return false;
            }
            options.manifest = argv[++i];
        } else {
            std::cerr << "unknown argument \"" << argument << "\"\n\n" << kUsage;
            exit_code = 2;
            return false;
        }
    }

    if (options.force_headless && options.force_gui) {
        std::cerr << "--headless and --gui contradict each other\n";
        exit_code = 2;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    chrono::SetChronoDataPath(CHRONO_DATA_PATH);
    chrono::vehicle::SetVehicleDataPath(CHRONO_VEHICLE_DATA_PATH);

    Options options;
    int exit_code = 0;
    if (!ParseArguments(argc, argv, options, exit_code)) return exit_code;

    if (options.print_environment) {
        const std::string commit = ChronoCommit();
        std::cout << "chrono_commit: " << (commit.empty() ? "unknown" : commit) << "\n";
        return commit.empty() ? 1 : 0;
    }

    if (options.manifest.empty()) {
        std::cerr << "no manifest given\n\n" << kUsage;
        return 2;
    }

    RunManifest manifest;
    try {
        manifest = LoadManifest(options.manifest);
    } catch (const ConfigError& e) {
        std::cerr << e.what() << "\n";
        return 3;
    }

    if (options.validate_only) {
        std::cout << "manifest ok: " << manifest.rover.wheels.size() << " wheels, terrain \""
                  << manifest.terrain.name << "\", scenario \"" << manifest.scenario.name << "\"\n";
        return 0;
    }

    options.headless = manifest.run.headless;
    if (options.force_headless) options.headless = true;
    if (options.force_gui) options.headless = false;

    try {
        Simulation simulation(std::move(manifest));

        if (options.headless) {
            simulation.Run();
        } else {
#ifdef TRICKFIRE_HAS_GUI
            RunViewer(simulation);
#else
            std::cerr << "this binary was built without the viewer; use --headless\n";
            return 4;
#endif
        }

        const auto& result = simulation.Result();
        const bool passed = simulation.Passed();

        std::cout << (passed ? "PASS" : "FAIL") << "  " << result.rule;
        if (!result.detail.empty()) std::cout << " (" << result.detail << ")";
        std::cout << "\n";

        // A failed run is not a crashed run: the output is complete and the report should
        // still be built, so the exit code distinguishes the two.
        return passed ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "run failed: " << e.what() << "\n";
        return 5;
    }
}
