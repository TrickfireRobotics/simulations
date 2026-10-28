#include "reporting/telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "chrono/utils/ChConstants.h"

namespace trickfire {

namespace fs = std::filesystem;

namespace {

/// Minimal JSON string escaping. Keys and values here are metric names, wheel ids and
/// short messages, so only the mandatory escapes are needed.
std::string Escape(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (char c : text) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
        }
    }
    return out;
}

/// JSON has no nan or inf, so a diverged channel is written as null rather than
/// producing a file no parser will read.
std::string Number(double value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream out;
    out << std::setprecision(10) << value;
    return out.str();
}

}  // namespace

TelemetryWriter::TelemetryWriter(const std::string& output_dir, const Sample& shape,
                                 const Metrics& metrics)
    : m_dir(output_dir), m_metrics(&metrics) {
    std::error_code ec;
    fs::create_directories(m_dir, ec);

    m_telemetry.open(fs::path(m_dir) / "telemetry.csv");
    if (!m_telemetry) throw std::runtime_error("cannot write telemetry.csv in " + m_dir);

    m_events.open(fs::path(m_dir) / "events.jsonl");
    if (!m_events) throw std::runtime_error("cannot write events.jsonl in " + m_dir);

    WriteHeader(shape);
}

TelemetryWriter::~TelemetryWriter() {
    m_telemetry.flush();
    m_events.flush();
}

void TelemetryWriter::WriteHeader(const Sample& shape) {
    m_columns = {"time"};

    if (m_metrics->Wants("mobility")) {
        for (const char* name : {"pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "speed",
                                 "distance", "straight_distance", "path_error", "grade_deg"}) {
            m_columns.emplace_back(name);
        }
    }
    if (m_metrics->Wants("suspension")) {
        for (const char* name : {"chassis_roll_deg", "chassis_pitch_deg", "ground_clearance"}) {
            m_columns.emplace_back(name);
        }
        for (const auto& pivot : shape.pivots) {
            m_columns.push_back(pivot.name + "_angle_deg");
            m_columns.push_back(pivot.name + "_rate_deg_s");
            m_columns.push_back(pivot.name + "_reaction_nm");
        }
    }
    if (m_metrics->Wants("wheel")) {
        for (const auto& wheel : shape.wheels) {
            const std::string p = wheel.id + "_";
            for (const char* name : {"angle", "speed", "commanded", "slip", "lateral_slip_deg",
                                     "sinkage", "normal_load", "drawbar_pull"}) {
                m_columns.push_back(p + name);
            }
        }
    }
    if (m_metrics->Wants("motor")) {
        for (const auto& wheel : shape.wheels) {
            const std::string p = wheel.id + "_";
            for (const char* name : {"torque_req", "torque", "current", "current_limit", "back_emf",
                                     "voltage_util", "mech_power", "loss", "temperature"}) {
                m_columns.push_back(p + name);
            }
        }
    }
    if (m_metrics->Wants("energy")) {
        for (const char* name : {"bus_voltage", "bus_current", "bus_power", "soc", "cumulative_wh",
                                 "regen_rejected_w"}) {
            m_columns.emplace_back(name);
        }
    }
    if (m_metrics->Wants("stability")) {
        for (const char* name :
             {"roll_rate_deg_s", "pitch_rate_deg_s", "wheels_in_contact", "stability_margin"}) {
            m_columns.emplace_back(name);
        }
    }
    if (m_metrics->Wants("numerics")) {
        for (const char* name : {"step_size", "real_time_factor", "solver_iterations",
                                 "contact_count", "scm_ray_casts", "scm_ray_hits"}) {
            m_columns.emplace_back(name);
        }
    }

    for (size_t i = 0; i < m_columns.size(); i++) {
        m_telemetry << (i ? "," : "") << m_columns[i];
    }
    m_telemetry << "\n";
}

void TelemetryWriter::WriteSample(const Sample& sample) {
    std::ostringstream row;
    row << std::setprecision(8);

    auto field = [&, first = true](double value) mutable {
        if (!first) row << ",";
        first = false;
        if (std::isfinite(value)) {
            row << value;
        } else {
            // Empty rather than nan: pandas, numpy and every spreadsheet read a blank
            // as missing, while "nan" parses as a string in some of them.
            row << "";
        }
    };

    field(sample.time);

    if (m_metrics->Wants("mobility")) {
        field(sample.position.x());
        field(sample.position.y());
        field(sample.position.z());
        field(sample.velocity.x());
        field(sample.velocity.y());
        field(sample.velocity.z());
        field(sample.speed);
        field(sample.distance);
        field(sample.straight_distance);
        field(sample.path_error);
        field(sample.grade * chrono::CH_RAD_TO_DEG);
    }
    if (m_metrics->Wants("suspension")) {
        field(sample.chassis_roll * chrono::CH_RAD_TO_DEG);
        field(sample.chassis_pitch * chrono::CH_RAD_TO_DEG);
        field(sample.ground_clearance);
        for (const auto& pivot : sample.pivots) {
            field(pivot.angle * chrono::CH_RAD_TO_DEG);
            field(pivot.velocity * chrono::CH_RAD_TO_DEG);
            field(pivot.reaction_torque);
        }
    }
    if (m_metrics->Wants("wheel")) {
        for (const auto& wheel : sample.wheels) {
            field(wheel.angle);
            field(wheel.speed);
            field(wheel.commanded);
            field(wheel.longitudinal_slip);
            field(wheel.lateral_slip * chrono::CH_RAD_TO_DEG);
            field(wheel.sinkage);
            field(wheel.normal_load);
            field(wheel.drawbar_pull);
        }
    }
    if (m_metrics->Wants("motor")) {
        for (const auto& wheel : sample.wheels) {
            const auto& drive = wheel.drive;
            field(drive.requested_torque);
            field(drive.applied_torque);
            field(drive.current_a);
            field(drive.current_limit_a);
            field(drive.back_emf_v);
            field(drive.voltage_utilisation);
            field(drive.mech_power_w);
            field(drive.loss_w);
            field(drive.temperature_c);
        }
    }
    if (m_metrics->Wants("energy")) {
        field(sample.battery.terminal_v);
        field(sample.battery.current_a);
        field(sample.battery.power_w);
        field(sample.battery.soc);
        field(sample.battery.cumulative_wh);
        field(sample.battery.regen_rejected_w);
    }
    if (m_metrics->Wants("stability")) {
        field(sample.roll_rate * chrono::CH_RAD_TO_DEG);
        field(sample.pitch_rate * chrono::CH_RAD_TO_DEG);
        field(sample.wheels_in_contact);
        field(sample.stability_margin);
    }
    if (m_metrics->Wants("numerics")) {
        field(sample.step_size);
        field(sample.real_time_factor);
        field(sample.solver_iterations);
        field(sample.contact_count);
        field(sample.scm_ray_casts);
        field(sample.scm_ray_hits);
    }

    m_telemetry << row.str() << "\n";
    m_samples++;
}

void TelemetryWriter::WriteEvent(const Event& event) {
    m_events << "{\"time\":" << Number(event.time) << ",\"kind\":\"" << Escape(event.kind)
             << "\",\"subject\":\"" << Escape(event.subject) << "\",\"detail\":\""
             << Escape(event.detail) << "\"}\n";
}

void TelemetryWriter::DetectEvents(const Sample& sample) {
    auto edge = [&](const std::string& key, bool active, const std::string& kind,
                    const std::string& subject, const std::string& detail) {
        const bool was = m_flags[key];
        if (active == was) return;
        m_flags[key] = active;
        WriteEvent({sample.time, active ? kind : kind + "_cleared", subject, detail});
    };

    for (const auto& wheel : sample.wheels) {
        const auto& drive = wheel.drive;
        edge(wheel.id + ".current", drive.current_saturated, "current_limit", wheel.id,
             "current at the " + std::to_string(static_cast<int>(drive.current_limit_a)) +
                 " A limit");
        edge(wheel.id + ".voltage", drive.voltage_saturated, "voltage_limit", wheel.id,
             "back emf leaves no headroom on the bus");
        edge(wheel.id + ".thermal", drive.thermal_cutoff, "thermal_cutoff", wheel.id,
             "motor at " + std::to_string(static_cast<int>(drive.temperature_c)) + " C");
        edge(wheel.id + ".speed", drive.speed_limited, "speed_limit", wheel.id,
             "output speed above the rated limit");
        edge(wheel.id + ".contact", wheel.normal_load <= 0, "wheel_lift", wheel.id,
             "no normal load on this wheel");
    }

    edge("battery.limit", sample.battery.current_limited, "battery_current_limit", "",
         "pack current limit reached");
    edge("battery.regen", sample.battery.regen_rejected_w > 1.0, "regen_rejected", "",
         "regenerated power had nowhere to go");
    edge("battery.empty", sample.battery.empty, "battery_empty", "", "state of charge at zero");
}

bool TelemetryWriter::Passed(const std::vector<CheckResult>& checks,
                             const Termination& termination) {
    if (!termination.pass) return false;
    return std::all_of(checks.begin(), checks.end(),
                       [](const CheckResult& check) { return check.passed; });
}

void TelemetryWriter::WriteSummary(const std::map<std::string, double>& summary,
                                   const std::vector<CheckResult>& checks,
                                   const Termination& termination, double wall_clock_s) {
    std::ofstream out(fs::path(m_dir) / "summary.json");
    if (!out) throw std::runtime_error("cannot write summary.json in " + m_dir);

    const bool passed = Passed(checks, termination);

    out << "{\n";
    out << "  \"passed\": " << (passed ? "true" : "false") << ",\n";
    out << "  \"wall_clock_s\": " << Number(wall_clock_s) << ",\n";
    out << "  \"samples\": " << m_samples << ",\n";

    const char* reason = "running";
    switch (termination.reason) {
        case Termination::Reason::END_TIME:
            reason = "end_time";
            break;
        case Termination::Reason::RULE:
            reason = "rule";
            break;
        case Termination::Reason::DIVERGED:
            reason = "diverged";
            break;
        case Termination::Reason::RUNNING:
            reason = "interrupted";
            break;
    }
    out << "  \"termination\": {\"reason\": \"" << reason << "\", \"rule\": \""
        << Escape(termination.rule) << "\", \"detail\": \"" << Escape(termination.detail)
        << "\", \"time\": " << Number(termination.time)
        << ", \"pass\": " << (termination.pass ? "true" : "false") << "},\n";

    out << "  \"checks\": [\n";
    for (size_t i = 0; i < checks.size(); i++) {
        const auto& check = checks[i];
        out << "    {\"metric\": \"" << Escape(check.metric) << "\", \"expression\": \""
            << Escape(check.expression) << "\", \"actual\": " << Number(check.actual)
            << ", \"passed\": " << (check.passed ? "true" : "false")
            << ", \"evaluated\": " << (check.evaluated ? "true" : "false")
            << ", \"description\": \"" << Escape(check.description) << "\"}"
            << (i + 1 < checks.size() ? "," : "") << "\n";
    }
    out << "  ],\n";

    out << "  \"metrics\": {\n";
    size_t written = 0;
    for (const auto& [key, value] : summary) {
        out << "    \"" << Escape(key) << "\": " << Number(value)
            << (++written < summary.size() ? "," : "") << "\n";
    }
    out << "  }\n";
    out << "}\n";
}

}  // namespace trickfire
