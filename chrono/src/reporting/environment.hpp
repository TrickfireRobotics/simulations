#pragma once

#include <string>

#include "config/spec.hpp"

namespace trickfire {

/// Write environment.json: the Chrono version and commit, this build's flags, the host,
/// the solver settings and the manifest hash.
///
/// This is what makes the Phase 0 exit gate checkable. Two runs whose summaries disagree
/// are only interesting if their environments match, and the only way to know that is to
/// record it at run time rather than trusting the container tag.
void WriteEnvironment(const std::string& output_dir, const RunManifest& manifest);

/// Chrono's commit SHA, read from the file the container image writes at build time.
/// Returns an empty string when the file is missing, which is itself worth reporting.
std::string ChronoCommit();

}  // namespace trickfire
