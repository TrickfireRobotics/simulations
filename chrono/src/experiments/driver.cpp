#include "experiments/driver.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace trickfire {

namespace {

/// Wheel ids are "<position><side>" - fl, ml, bl, fr, mr, br - so the trailing letter
/// decides which side of the skid-steer bias a wheel is on.
double SideOf(const std::string& id) {
    if (id.empty()) return 1.0;
    switch (id.back()) {
        case 'l':
        case 'L':
            return 1.0;
        case 'r':
        case 'R':
            return -1.0;
        default:
            throw std::runtime_error("wheel id \"" + id +
                                     "\" does not end in l or r, so its side is unknown");
    }
}

std::vector<std::vector<double>> ReadReplay(const std::string& path, size_t wheel_count) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open replay file: " + path);

    std::vector<std::vector<double>> rows;
    std::string line;
    size_t line_number = 0;

    while (std::getline(in, line)) {
        line_number++;
        if (line.empty() || line[0] == '#') continue;

        std::vector<double> row;
        std::istringstream fields(line);
        std::string field;
        bool numeric = true;

        while (std::getline(fields, field, ',')) {
            try {
                row.push_back(std::stod(field));
            } catch (const std::exception&) {
                numeric = false;
                break;
            }
        }
        // The first non-comment line is usually a header, so a non-numeric row is skipped
        // rather than treated as an error.
        if (!numeric) continue;

        if (row.size() != wheel_count + 1) {
            throw std::runtime_error(path + ":" + std::to_string(line_number) + ": expected " +
                                     std::to_string(wheel_count + 1) +
                                     " columns (time plus one per wheel), got " +
                                     std::to_string(row.size()));
        }
        rows.push_back(std::move(row));
    }
    if (rows.empty()) throw std::runtime_error("replay file has no data rows: " + path);

    std::sort(rows.begin(), rows.end(),
              [](const std::vector<double>& a, const std::vector<double>& b) {
                  return a.front() < b.front();
              });
    return rows;
}

}  // namespace

Driver::Driver(DriverSpec spec, std::vector<std::string> wheel_ids)
    : m_spec(std::move(spec)), m_wheel_ids(std::move(wheel_ids)) {
    for (const auto& id : m_wheel_ids) m_side.push_back(SideOf(id));
    m_requests.assign(m_wheel_ids.size(), 0.0);

    if (m_spec.type == DriverSpec::Type::REPLAY) {
        m_replay = ReadReplay(m_spec.replay_file, m_wheel_ids.size());
    }
}

void Driver::Reset() {
    m_requests.assign(m_wheel_ids.size(), 0.0);
    m_replay_cursor = 0;
    m_base = 0;
}

double Driver::Profile(double time) const {
    switch (m_spec.type) {
        case DriverSpec::Type::STOPPED:
            return 0.0;

        case DriverSpec::Type::CONSTANT:
            // Still ramped in: stepping six drives to full command at t = 0 is a load case
            // the rover never sees, and it dominates the peak-current metric.
            if (m_spec.ramp_time <= 0) return m_spec.value;
            return m_spec.value * std::min(1.0, time / m_spec.ramp_time);

        case DriverSpec::Type::RAMP:
            if (m_spec.ramp_time <= 0) return m_spec.value;
            return m_spec.value * std::min(1.0, time / m_spec.ramp_time);

        case DriverSpec::Type::SLIP_SWEEP: {
            if (m_spec.sweep_values.empty()) return 0.0;
            const auto index = static_cast<size_t>(time / m_spec.sweep_hold_s);
            return m_spec.sweep_values[std::min(index, m_spec.sweep_values.size() - 1)];
        }

        case DriverSpec::Type::REPLAY:
            return 0.0;  // handled directly in Advance
    }
    return 0.0;
}

const std::vector<double>& Driver::Advance(double time) {
    if (m_spec.type == DriverSpec::Type::REPLAY) {
        // Hold the most recent recorded row. Interpolating would invent commands the
        // hardware never issued, and the actuator's own command path already models the
        // controller's sample-and-hold.
        while (m_replay_cursor + 1 < m_replay.size() &&
               m_replay[m_replay_cursor + 1].front() <= time) {
            m_replay_cursor++;
        }
        const auto& row = m_replay[m_replay_cursor];
        for (size_t i = 0; i < m_requests.size(); i++) m_requests[i] = row[i + 1];

        m_base = 0;
        for (double request : m_requests) m_base += request;
        m_base /= static_cast<double>(m_requests.size());
        return m_requests;
    }

    m_base = Profile(time);

    // Skid steer: one side is scaled down and the other up, keeping the mean command at
    // the requested value so a turn does not also change the commanded speed.
    for (size_t i = 0; i < m_requests.size(); i++) {
        m_requests[i] = m_base * (1.0 - m_side[i] * m_spec.steering);
    }
    return m_requests;
}

}  // namespace trickfire
