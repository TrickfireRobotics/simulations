#pragma once

#include <map>
#include <string>
#include <vector>

#include "config/spec.hpp"
#include "experiments/sample.hpp"
#include "experiments/termination.hpp"

namespace trickfire {

/// One pass/fail assertion after evaluation.
struct CheckResult {
    std::string metric;
    std::string expression;  // "motor.peak_temperature_c < 90"
    double actual = 0;
    bool passed = false;
    bool evaluated = false;  // false when the metric was never recorded
    std::string description;
};

/// Running statistics for one channel: enough to report a mean, a peak and the time
/// spent above a threshold without keeping every sample in memory.
struct ChannelStats {
    double min = 0;
    double max = 0;
    double mean = 0;
    double last = 0;
    double integral = 0;  // time integral, for energy-like quantities
    double duration_above = 0;
    bool seen = false;

    void Add(double value, double dt, double above_threshold);
};

/// Accumulates summary metrics across a run.
///
/// Samples before the warm-up interval are recorded in the telemetry but excluded from
/// the summary, so the drop and settle transient does not set the peak-current or
/// peak-torque numbers a wheel comparison is ranked on.
class Metrics {
  public:
    Metrics(const ScenarioSpec& scenario, std::vector<std::string> wheel_ids,
            std::vector<std::string> pivot_names);

    void Add(const Sample& sample);

    /// Flat <group>.<name> summary, which the checks and the report both read.
    std::map<std::string, double> Summary() const;

    std::vector<CheckResult> EvaluateChecks(const std::vector<CheckSpec>& checks) const;

    /// Whether a metric group is enabled for this scenario.
    bool Wants(const std::string& group) const;

  private:
    void AddMobility(const Sample& sample, double dt);
    void AddWheels(const Sample& sample, double dt);
    void AddSuspension(const Sample& sample, double dt);
    void AddMotor(const Sample& sample, double dt);
    void AddEnergy(const Sample& sample, double dt);
    void AddStability(const Sample& sample, double dt);
    void AddNumerics(const Sample& sample, double dt);

    ChannelStats& Channel(const std::string& key);

    ScenarioSpec m_scenario;
    std::vector<std::string> m_wheel_ids;
    std::vector<std::string> m_pivot_names;

    std::map<std::string, ChannelStats> m_channels;

    double m_last_time = 0;
    double m_window = 0;  // measured time, excluding the warm-up
    bool m_started = false;
    Sample m_first;
    Sample m_latest;
    long m_samples = 0;
};

}  // namespace trickfire
