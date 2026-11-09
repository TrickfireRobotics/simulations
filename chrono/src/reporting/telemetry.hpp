#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "experiments/metrics.hpp"
#include "experiments/sample.hpp"
#include "experiments/termination.hpp"

namespace trickfire {

/// One notable thing that happened, written to events.jsonl as it occurs.
struct Event {
    double time = 0;
    std::string kind;     // "current_limit", "thermal_cutoff", "wheel_lift", ...
    std::string subject;  // wheel id, pivot name, or empty for rover-wide
    std::string detail;
};

/// Writes a run's output directory: the time-series CSV, the event log, and the summary.
///
/// CSV is written straight from C++ and compressed by the Python CLI afterwards. Pulling
/// in the Arrow stack just to write Parquet would be a large dependency for a file the
/// reporting step reads once.
class TelemetryWriter {
  public:
    /// Opens the output files and writes the CSV header derived from the first sample's
    /// shape, so the column set always matches the rover that actually ran.
    TelemetryWriter(const std::string& output_dir, const Sample& shape, const Metrics& metrics);
    ~TelemetryWriter();

    void WriteSample(const Sample& sample);

    void WriteEvent(const Event& event);

    /// Detect limit, contact and stability transitions by comparing against the previous
    /// sample, and log one event per edge rather than one per step.
    void DetectEvents(const Sample& sample);

    /// Write summary.json, including the pass/fail verdict. Called once at the end.
    void WriteSummary(const std::map<std::string, double>& summary,
                      const std::vector<CheckResult>& checks, const Termination& termination,
                      double wall_clock_s);

    /// Overall verdict: every check passed and the termination was a pass.
    static bool Passed(const std::vector<CheckResult>& checks, const Termination& termination);

    long SampleCount() const { return m_samples; }

  private:
    void WriteHeader(const Sample& shape);

    std::string m_dir;
    std::ofstream m_telemetry;
    std::ofstream m_events;
    std::vector<std::string> m_columns;

    const Metrics* m_metrics;
    long m_samples = 0;

    // Previous-step flags, so events are logged on transitions only.
    std::map<std::string, bool> m_flags;
};

}  // namespace trickfire
