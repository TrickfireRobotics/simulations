#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "config/spec.hpp"

namespace trickfire {

/// Thrown when a manifest cannot be turned into a usable RunManifest. Carries every
/// problem found rather than only the first, because a half-validated manifest sends
/// someone back to the YAML repeatedly for what is really one editing pass.
class ConfigError : public std::runtime_error {
  public:
    explicit ConfigError(std::vector<std::string> problems);

    const std::vector<std::string>& Problems() const { return m_problems; }

  private:
    std::vector<std::string> m_problems;
};

/// Parse and validate a resolved run manifest.
///
/// Paths inside the manifest are resolved relative to the manifest's own directory
/// unless they are absolute, so a results directory stays self-contained and can be
/// re-run after being copied to another machine.
RunManifest LoadManifest(const std::string& manifest_path);

/// The textual provenance ranking used by ScenarioSpec::require_provenance.
/// Higher is stronger: placeholder < estimated < cad < datasheet < measured.
int ProvenanceRank(const std::string& provenance);

}  // namespace trickfire
