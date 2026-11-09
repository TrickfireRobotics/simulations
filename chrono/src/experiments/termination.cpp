#include "experiments/termination.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "chrono/utils/ChConstants.h"

namespace trickfire {

namespace {

std::string Describe(const std::string& quantity, double value, const char* comparison,
                     double threshold, const char* unit) {
    std::ostringstream out;
    out.precision(4);
    out << quantity << " " << value << " " << unit << " " << comparison << " " << threshold << " "
        << unit;
    return out.str();
}

const char* RuleName(TerminationSpec::Type type) {
    switch (type) {
        case TerminationSpec::Type::DISTANCE:
            return "distance";
        case TerminationSpec::Type::ROLLOVER:
            return "rollover";
        case TerminationSpec::Type::IMMOBILITY:
            return "immobility";
        case TerminationSpec::Type::SINKAGE:
            return "sinkage";
        case TerminationSpec::Type::MOTOR_TEMPERATURE:
            return "motor_temperature";
        case TerminationSpec::Type::BATTERY_EMPTY:
            return "battery_empty";
    }
    return "unknown";
}

}  // namespace

TerminationMonitor::TerminationMonitor(std::vector<TerminationSpec> rules, double end_time)
    : m_rules(std::move(rules)), m_end_time(end_time) {}

void TerminationMonitor::Reset() { m_progress.clear(); }

Termination TerminationMonitor::Check(const Sample& sample) {
    m_progress.emplace_back(sample.time, sample.distance);

    for (const auto& rule : m_rules) {
        Termination hit;
        hit.reason = Termination::Reason::RULE;
        hit.rule = RuleName(rule.type);
        hit.time = sample.time;
        hit.pass = rule.pass;

        switch (rule.type) {
            case TerminationSpec::Type::DISTANCE:
                if (sample.distance >= rule.threshold) {
                    hit.detail = Describe("distance", sample.distance, ">=", rule.threshold, "m");
                    return hit;
                }
                break;

            case TerminationSpec::Type::ROLLOVER: {
                const double tilt =
                    std::max(std::abs(sample.chassis_roll), std::abs(sample.chassis_pitch)) *
                    chrono::CH_RAD_TO_DEG;
                if (tilt >= rule.threshold) {
                    hit.detail = Describe("tilt", tilt, ">=", rule.threshold, "deg");
                    return hit;
                }
                break;
            }

            case TerminationSpec::Type::IMMOBILITY: {
                // Needs a full window of history before it can fire, so a run that has
                // not yet got moving is not immediately declared stuck.
                const double window_start = sample.time - rule.window_s;
                if (window_start < 0) break;

                // Drop samples that have fallen out of the window, keeping one before it
                // so the comparison spans the whole interval.
                while (m_progress.size() > 1 && m_progress[1].first <= window_start) {
                    m_progress.erase(m_progress.begin());
                }
                if (m_progress.front().first > window_start) break;

                const double progress = sample.distance - m_progress.front().second;
                if (progress < rule.threshold) {
                    hit.detail = Describe("progress over the last window", progress, "<",
                                          rule.threshold, "m");
                    return hit;
                }
                break;
            }

            case TerminationSpec::Type::SINKAGE: {
                double worst = 0;
                for (const auto& wheel : sample.wheels) worst = std::max(worst, wheel.sinkage);
                if (worst >= rule.threshold) {
                    hit.detail = Describe("worst wheel sinkage", worst, ">=", rule.threshold, "m");
                    return hit;
                }
                break;
            }

            case TerminationSpec::Type::MOTOR_TEMPERATURE: {
                double hottest = 0;
                for (const auto& wheel : sample.wheels) {
                    hottest = std::max(hottest, wheel.drive.temperature_c);
                }
                if (hottest >= rule.threshold) {
                    hit.detail = Describe("hottest motor", hottest, ">=", rule.threshold, "C");
                    return hit;
                }
                break;
            }

            case TerminationSpec::Type::BATTERY_EMPTY:
                if (sample.battery.soc <= rule.threshold) {
                    hit.detail =
                        Describe("state of charge", sample.battery.soc, "<=", rule.threshold, "");
                    return hit;
                }
                break;
        }
    }

    // Anything non-finite means the solver has diverged. Stopping here keeps the
    // telemetry written so far usable instead of filling it with nan.
    if (!std::isfinite(sample.position.Length()) || !std::isfinite(sample.speed)) {
        Termination diverged;
        diverged.reason = Termination::Reason::DIVERGED;
        diverged.rule = "diverged";
        diverged.detail = "chassis state is not finite - reduce the step size";
        diverged.time = sample.time;
        diverged.pass = false;
        return diverged;
    }

    if (sample.time >= m_end_time) {
        Termination finished;
        finished.reason = Termination::Reason::END_TIME;
        finished.rule = "end_time";
        finished.detail = Describe("time", sample.time, ">=", m_end_time, "s");
        finished.time = sample.time;
        finished.pass = true;
        return finished;
    }

    return Termination{};
}

}  // namespace trickfire
