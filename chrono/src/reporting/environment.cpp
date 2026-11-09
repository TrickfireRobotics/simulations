#include "reporting/environment.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include "chrono/ChVersion.h"

namespace trickfire {

namespace fs = std::filesystem;

namespace {

std::string ReadFirstLine(const fs::path& path) {
    std::ifstream in(path);
    if (!in) return "";
    std::string line;
    std::getline(in, line);
    // Strip trailing whitespace: `git rev-parse > file` leaves a newline, and a SHA with
    // a stray space in it will not compare equal between runs.
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ')) {
        line.pop_back();
    }
    return line;
}

std::string Hostname() {
    if (const char* env = std::getenv("HOSTNAME")) return env;
    return ReadFirstLine("/etc/hostname");
}

/// SHA-256 would need a dependency; the manifest is hashed by the Python CLI, which
/// records it in the manifest itself. This is a cheap content fingerprint so a run can
/// still be matched to its manifest when the CLI was bypassed.
std::string FileFingerprint(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return "";

    // FNV-1a, 64-bit: not cryptographic, but sufficient to tell two manifests apart.
    std::uint64_t hash = 0xcbf29ce484222325ull;
    char buffer[4096];
    while (in.read(buffer, sizeof(buffer)) || in.gcount() > 0) {
        for (std::streamsize i = 0; i < in.gcount(); i++) {
            hash ^= static_cast<unsigned char>(buffer[i]);
            hash *= 0x100000001b3ull;
        }
        if (!in) break;
    }

    std::ostringstream out;
    out << std::hex << hash;
    return out.str();
}

std::string Escape(const std::string& text) {
    std::string out;
    for (char c : text) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

}  // namespace

std::string ChronoCommit() {
    // Written by the container build next to the Chrono source tree.
    for (const char* candidate : {"/home/trickfire/chrono-commit.txt",
                                  TRICKFIRE_CHRONO_SOURCE_DIR "/../chrono-commit.txt"}) {
        std::string commit = ReadFirstLine(candidate);
        if (!commit.empty()) return commit;
    }
    return "";
}

void WriteEnvironment(const std::string& output_dir, const RunManifest& manifest) {
    std::error_code ec;
    fs::create_directories(output_dir, ec);

    std::ofstream out(fs::path(output_dir) / "environment.json");
    if (!out) return;

    const std::string commit = ChronoCommit();

    out << "{\n";
    out << "  \"chrono_version\": \"" << CHRONO_VERSION << "\",\n";
    out << "  \"chrono_commit\": \"" << Escape(commit) << "\",\n";
    out << "  \"chrono_source_dir\": \"" << Escape(TRICKFIRE_CHRONO_SOURCE_DIR) << "\",\n";
    out << "  \"build_type\": \""
#ifdef NDEBUG
        << "Release"
#else
        << "Debug"
#endif
        << "\",\n";
    out << "  \"compiler\": \""
#if defined(__clang__)
        << "clang " << __clang_major__ << "." << __clang_minor__
#elif defined(__GNUC__)
        << "gcc " << __GNUC__ << "." << __GNUC_MINOR__
#else
        << "unknown"
#endif
        << "\",\n";
    out << "  \"compiled\": \"" << __DATE__ << " " << __TIME__ << "\",\n";
    out << "  \"host\": \"" << Escape(Hostname()) << "\",\n";
    out << "  \"hardware_threads\": " << std::thread::hardware_concurrency() << ",\n";
    out << "  \"manifest_path\": \"" << Escape(manifest.run.manifest_path) << "\",\n";
    out << "  \"manifest_fingerprint\": \"" << FileFingerprint(manifest.run.manifest_path)
        << "\",\n";
    out << "  \"run_id\": \"" << Escape(manifest.run.id) << "\",\n";
    out << "  \"seed\": " << manifest.run.seed << ",\n";
    out << "  \"solver\": {\n";
    out << "    \"contact_method\": \""
        << (manifest.simulation.contact_method == SimulationSpec::ContactMethod::SMC ? "smc"
                                                                                     : "nsc")
        << "\",\n";
    out << "    \"step_size\": " << manifest.simulation.step_size << ",\n";
    out << "    \"threads\": " << manifest.simulation.threads << ",\n";
    out << "    \"up_axis\": \""
        << (manifest.simulation.up_axis == SimulationSpec::UpAxis::Z ? "z" : "y") << "\"\n";
    out << "  }\n";
    out << "}\n";
}

}  // namespace trickfire
