#include "experiments/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "chrono/utils/ChConstants.h"

namespace trickfire {

namespace {

std::string Format(double value) {
    std::ostringstream out;
    out.precision(6);
    out << value;
    return out.str();
}

const char* OpSymbol(CheckSpec::Op op) {
    switch (op) {
        case CheckSpec::Op::LT:
            return "<";
        case CheckSpec::Op::LE:
            return "<=";
        case CheckSpec::Op::GT:
            return ">";
        case CheckSpec::Op::GE:
            return ">=";
    }
    return "?";
}

bool Compare(double actual, CheckSpec::Op op, double expected) {
    switch (op) {
        case CheckSpec::Op::LT:
            return actual < expected;
        case CheckSpec::Op::LE:
            return actual <= expected;
        case CheckSpec::Op::GT:
            return actual > expected;
        case CheckSpec::Op::GE:
            return actual >= expected;
    }
    return false;
}

}  // namespace

void ChannelStats::Add(double value, double dt, double above_threshold) {
    if (!std::isfinite(value)) return;

    if (!seen) {
        min = max = value;
        seen = true;
    } else {
        min = std::min(min, value);
        max = std::max(max, value);
    }

    // Time-weighted mean, so a variable step size does not bias the average towards
    // whichever region the solver happened to take small steps in.
    integral += value * dt;
    last = value;
    if (value > above_threshold) duration_above += dt;
}

Metrics::Metrics(const ScenarioSpec& scenario, std::vector<std::string> wheel_ids,
                 std::vector<std::string> pivot_names)
    : m_scenario(scenario),
      m_wheel_ids(std::move(wheel_ids)),
      m_pivot_names(std::move(pivot_names)) {}

bool Metrics::Wants(const std::string& group) const {
    if (m_scenario.metric_groups.empty()) return true;
    return std::find(m_scenario.metric_groups.begin(), m_scenario.metric_groups.end(), group) !=
           m_scenario.metric_groups.end();
}

ChannelStats& Metrics::Channel(const std::string& key) { return m_channels[key]; }

void Metrics::Add(const Sample& sample) {
    m_latest = sample;

    if (sample.time < m_scenario.warmup_time) {
        m_last_time = sample.time;
        return;
    }
    if (!m_started) {
        m_first = sample;
        m_started = true;
        m_last_time = sample.time;
        return;
    }

    const double dt = sample.time - m_last_time;
    m_last_time = sample.time;
    if (dt <= 0) return;

    m_window += dt;
    m_samples++;

    if (Wants("mobility")) AddMobility(sample, dt);
    if (Wants("wheel")) AddWheels(sample, dt);
    if (Wants("suspension")) AddSuspension(sample, dt);
    if (Wants("motor")) AddMotor(sample, dt);
    if (Wants("energy")) AddEnergy(sample, dt);
    if (Wants("stability")) AddStability(sample, dt);
    if (Wants("numerics")) AddNumerics(sample, dt);
}

void Metrics::AddMobility(const Sample& sample, double dt) {
    Channel("mobility.speed_m_s").Add(sample.speed, dt, 0);
    Channel("mobility.path_error_m").Add(std::abs(sample.path_error), dt, 0);
    Channel("mobility.grade_deg").Add(sample.grade * chrono::CH_RAD_TO_DEG, dt, 0);
}

void Metrics::AddWheels(const Sample& sample, double dt) {
    double slip_sum = 0;
    double sinkage_sum = 0;
    double drawbar_sum = 0;

    for (const auto& wheel : sample.wheels) {
        const std::string prefix = "wheel." + wheel.id + ".";
        Channel(prefix + "speed_rad_s").Add(wheel.speed, dt, 0);
        Channel(prefix + "longitudinal_slip").Add(wheel.longitudinal_slip, dt, 0);
        Channel(prefix + "lateral_slip_deg").Add(wheel.lateral_slip * chrono::CH_RAD_TO_DEG, dt, 0);
        Channel(prefix + "sinkage_m").Add(wheel.sinkage, dt, 0);
        Channel(prefix + "normal_load_n").Add(wheel.normal_load, dt, 0);
        Channel(prefix + "drawbar_pull_n").Add(wheel.drawbar_pull, dt, 0);

        slip_sum += wheel.longitudinal_slip;
        sinkage_sum += wheel.sinkage;
        drawbar_sum += wheel.drawbar_pull;
    }

    const double count = static_cast<double>(std::max<size_t>(1, sample.wheels.size()));
    Channel("wheel.mean_longitudinal_slip").Add(slip_sum / count, dt, 0);
    Channel("wheel.mean_sinkage_m").Add(sinkage_sum / count, dt, 0);
    Channel("wheel.total_drawbar_pull_n").Add(drawbar_sum, dt, 0);
}

void Metrics::AddSuspension(const Sample& sample, double dt) {
    for (const auto& pivot : sample.pivots) {
        const std::string prefix = "suspension." + pivot.name + ".";
        Channel(prefix + "angle_deg").Add(pivot.angle * chrono::CH_RAD_TO_DEG, dt, 0);
        Channel(prefix + "rate_deg_s").Add(pivot.velocity * chrono::CH_RAD_TO_DEG, dt, 0);
        Channel(prefix + "reaction_torque_nm").Add(pivot.reaction_torque, dt, 0);
    }
    Channel("suspension.chassis_roll_deg").Add(sample.chassis_roll * chrono::CH_RAD_TO_DEG, dt, 0);
    Channel("suspension.chassis_pitch_deg")
        .Add(sample.chassis_pitch * chrono::CH_RAD_TO_DEG, dt, 0);
    Channel("suspension.ground_clearance_m").Add(sample.ground_clearance, dt, 0);
}

void Metrics::AddMotor(const Sample& sample, double dt) {
    double power_sum = 0;

    for (const auto& wheel : sample.wheels) {
        const auto& drive = wheel.drive;
        const std::string prefix = "motor." + wheel.id + ".";

        Channel(prefix + "applied_torque_nm").Add(std::abs(drive.applied_torque), dt, 0);
        Channel(prefix + "requested_torque_nm").Add(std::abs(drive.requested_torque), dt, 0);
        Channel(prefix + "current_a").Add(std::abs(drive.current_a), dt, 0);
        Channel(prefix + "temperature_c").Add(drive.temperature_c, dt, 0);
        Channel(prefix + "voltage_utilisation").Add(drive.voltage_utilisation, dt, 0);
        Channel(prefix + "mech_power_w").Add(drive.mech_power_w, dt, 0);

        // Saturation is recorded as time rather than a flag count, so it is comparable
        // between runs of different length.
        Channel(prefix + "current_saturated_s").Add(drive.current_saturated ? 1.0 : 0.0, dt, 0.5);
        Channel(prefix + "voltage_saturated_s").Add(drive.voltage_saturated ? 1.0 : 0.0, dt, 0.5);
        Channel(prefix + "thermal_cutoff_s").Add(drive.thermal_cutoff ? 1.0 : 0.0, dt, 0.5);

        power_sum += drive.mech_power_w;
    }
    Channel("motor.total_mech_power_w").Add(power_sum, dt, 0);
}

void Metrics::AddEnergy(const Sample& sample, double dt) {
    Channel("energy.bus_power_w").Add(sample.battery.power_w, dt, 0);
    Channel("energy.bus_current_a").Add(sample.battery.current_a, dt, 0);
    Channel("energy.terminal_voltage_v").Add(sample.battery.terminal_v, dt, 0);
    Channel("energy.state_of_charge").Add(sample.battery.soc, dt, 0);
    Channel("energy.regen_rejected_w").Add(sample.battery.regen_rejected_w, dt, 0);
}

void Metrics::AddStability(const Sample& sample, double dt) {
    Channel("stability.margin").Add(sample.stability_margin, dt, 0);
    Channel("stability.roll_rate_deg_s")
        .Add(std::abs(sample.roll_rate) * chrono::CH_RAD_TO_DEG, dt, 0);
    Channel("stability.pitch_rate_deg_s")
        .Add(std::abs(sample.pitch_rate) * chrono::CH_RAD_TO_DEG, dt, 0);

    // Anything less than the full set on the ground is a wheel lift; the duration is what
    // distinguishes a bump from a genuine stability problem.
    const bool lifted = sample.wheels_in_contact < static_cast<int>(sample.wheels.size());
    Channel("stability.wheel_lift_s").Add(lifted ? 1.0 : 0.0, dt, 0.5);
}

void Metrics::AddNumerics(const Sample& sample, double dt) {
    Channel("numerics.step_size_s").Add(sample.step_size, dt, 0);
    Channel("numerics.real_time_factor").Add(sample.real_time_factor, dt, 0);
    Channel("numerics.solver_iterations").Add(sample.solver_iterations, dt, 0);
    Channel("numerics.contact_count").Add(sample.contact_count, dt, 0);
    Channel("numerics.scm_ray_casts").Add(sample.scm_ray_casts, dt, 0);
    Channel("numerics.scm_ray_hits").Add(sample.scm_ray_hits, dt, 0);
}

std::map<std::string, double> Metrics::Summary() const {
    std::map<std::string, double> summary;

    for (const auto& [key, stats] : m_channels) {
        if (!stats.seen) continue;
        summary[key + ".min"] = stats.min;
        summary[key + ".max"] = stats.max;
        summary[key + ".mean"] = m_window > 0 ? stats.integral / m_window : stats.last;
        summary[key + ".final"] = stats.last;
        if (stats.duration_above > 0) summary[key + ".duration_s"] = stats.duration_above;
    }

    summary["run.measured_duration_s"] = m_window;
    summary["run.samples"] = static_cast<double>(m_samples);

    if (m_started) {
        summary["mobility.distance_m"] = m_latest.distance - m_first.distance;
        summary["mobility.straight_distance_m"] = m_latest.straight_distance;
        summary["mobility.completion_time_s"] = m_latest.time;

        const double distance = summary["mobility.distance_m"];
        const double energy_wh = m_latest.battery.cumulative_wh;
        summary["energy.total_wh"] = energy_wh;
        // The headline number for a wheel comparison. Guarded because a run that never
        // moved would otherwise report an infinite or absurd efficiency.
        summary["energy.wh_per_m"] = distance > 0.01 ? energy_wh / distance : 0.0;
        summary["energy.peak_current_a"] = m_latest.battery.peak_current_a;
        summary["energy.min_terminal_voltage_v"] = m_latest.battery.min_terminal_v;
        summary["energy.voltage_sag_v"] =
            m_latest.battery.open_circuit_v - m_latest.battery.min_terminal_v;

        double peak_temperature = 0;
        double peak_torque = 0;
        for (const auto& wheel : m_latest.wheels) {
            peak_temperature = std::max(peak_temperature, wheel.drive.temperature_c);
        }
        for (const auto& [key, stats] : m_channels) {
            if (key.find("applied_torque_nm") != std::string::npos) {
                peak_torque = std::max(peak_torque, stats.max);
            }
            if (key.find("temperature_c") != std::string::npos) {
                peak_temperature = std::max(peak_temperature, stats.max);
            }
        }
        summary["motor.peak_temperature_c"] = peak_temperature;
        summary["motor.peak_torque_nm"] = peak_torque;
    }
    return summary;
}

std::vector<CheckResult> Metrics::EvaluateChecks(const std::vector<CheckSpec>& checks) const {
    const auto summary = Summary();
    std::vector<CheckResult> results;

    for (const auto& check : checks) {
        CheckResult result;
        result.metric = check.metric;
        result.description = check.description;
        result.expression = check.metric + " " + OpSymbol(check.op) + " " + Format(check.value);

        auto found = summary.find(check.metric);
        if (found == summary.end()) {
            // A check on a metric the run never produced fails rather than passing
            // silently: a gate that cannot be evaluated has told you nothing.
            result.evaluated = false;
            result.passed = false;
            results.push_back(result);
            continue;
        }
        result.evaluated = true;
        result.actual = found->second;
        result.passed = Compare(result.actual, check.op, check.value);
        results.push_back(result);
    }
    return results;
}

}  // namespace trickfire
