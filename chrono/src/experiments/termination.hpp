#pragma once

#include <string>
#include <vector>

#include "config/spec.hpp"
#include "experiments/sample.hpp"

namespace trickfire {

/// Why a run stopped.
struct Termination {
    enum class Reason { RUNNING, END_TIME, RULE, DIVERGED };

    Reason reason = Reason::RUNNING;
    std::string rule;    // which termination rule fired
    std::string detail;  // human-readable condition, e.g. "distance 20.4 m >= 20 m"
    double time = 0;
    bool pass = true;
};

/// Evaluates a scenario's termination rules once per sample.
///
/// Rules are checked in the order they appear, so a scenario can put its success
/// condition first and have it win a tie with a failure condition on the same step.
class TerminationMonitor {
  public:
    TerminationMonitor(std::vector<TerminationSpec> rules, double end_time);

    /// Returns the termination if the run should stop, or a RUNNING reason if not.
    Termination Check(const Sample& sample);

    void Reset();

  private:
    std::vector<TerminationSpec> m_rules;
    double m_end_time = 0;

    /// Progress history for the immobility rule, as (time, distance) pairs.
    std::vector<std::pair<double, double>> m_progress;
};

}  // namespace trickfire
