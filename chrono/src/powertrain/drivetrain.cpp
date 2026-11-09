#include "powertrain/drivetrain.hpp"

#include <stdexcept>

namespace trickfire {

Drivetrain::Drivetrain(const std::map<std::string, ActuatorSpec>& actuators,
                       const std::vector<std::pair<std::string, std::string>>& wheel_actuators,
                       const BatterySpec& battery, double step_size)
    : m_battery(battery, step_size) {
    for (const auto& [wheel_id, actuator_name] : wheel_actuators) {
        auto found = actuators.find(actuator_name);
        if (found == actuators.end()) {
            throw std::runtime_error("wheel \"" + wheel_id + "\" references unknown actuator \"" +
                                     actuator_name + "\"");
        }
        m_drives.push_back(std::make_unique<Actuator>(found->second, step_size));
        m_wheel_ids.push_back(wheel_id);
    }
    m_torques.assign(m_drives.size(), 0.0);
}

void Drivetrain::Reset() {
    for (auto& drive : m_drives) drive->Reset();
    m_battery.Reset();
    m_torques.assign(m_drives.size(), 0.0);
}

const std::vector<double>& Drivetrain::Advance(double time, const std::vector<double>& requests,
                                               const std::vector<double>& speeds) {
    if (requests.size() != m_drives.size() || speeds.size() != m_drives.size()) {
        throw std::runtime_error("drivetrain got " + std::to_string(requests.size()) +
                                 " requests and " + std::to_string(speeds.size()) + " speeds for " +
                                 std::to_string(m_drives.size()) + " drives");
    }

    const double bus_voltage = m_battery.TerminalVoltage();

    double total_current = 0;
    for (size_t i = 0; i < m_drives.size(); i++) {
        m_torques[i] = m_drives[i]->Advance(time, requests[i], speeds[i], bus_voltage);
        total_current += m_drives[i]->BusCurrent(bus_voltage);
    }

    // The pack's answer is applied uniformly. Scaling each drive by the same factor keeps
    // left-right torque balanced under a current limit, which is how a shared bus behaves;
    // limiting them one at a time would invent a steering input.
    const double scale = m_battery.Advance(total_current);
    if (scale < 1.0) {
        for (auto& torque : m_torques) torque *= scale;
    }
    return m_torques;
}

}  // namespace trickfire
