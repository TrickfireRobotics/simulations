#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "powertrain/actuator.hpp"
#include "powertrain/battery.hpp"

namespace trickfire {

/// The six drives plus the pack they share.
///
/// Advance() is called once per simulation step with the measured wheel speeds and the
/// per-wheel requests, and returns the torque to apply at each wheel. The pack is settled
/// after the drives have sized their currents, and if it cannot supply the total the
/// torques are scaled back together - which is what actually happens on the rover, where
/// a sagging bus limits every drive at once.
class Drivetrain {
  public:
    Drivetrain(const std::map<std::string, ActuatorSpec>& actuators,
               const std::vector<std::pair<std::string, std::string>>& wheel_actuators,
               const BatterySpec& battery, double step_size);

    /// Compute this step's wheel torques. `requests` and `speeds` are indexed the same
    /// way as the wheel list handed to the constructor.
    const std::vector<double>& Advance(double time, const std::vector<double>& requests,
                                       const std::vector<double>& speeds);

    void Reset();

    size_t Size() const { return m_drives.size(); }
    const Actuator& Drive(size_t index) const { return *m_drives[index]; }
    const std::string& WheelId(size_t index) const { return m_wheel_ids[index]; }
    const Battery& Pack() const { return m_battery; }

    /// Reflected rotor inertia for a wheel station, which the rover folds into the wheel
    /// body's spin inertia.
    double ReflectedInertia(size_t index) const { return m_drives[index]->ReflectedInertia(); }

    /// Total electrical energy drawn from the pack, in Wh.
    double EnergyWh() const { return m_battery.State().cumulative_wh; }

  private:
    std::vector<std::unique_ptr<Actuator>> m_drives;
    std::vector<std::string> m_wheel_ids;
    Battery m_battery;

    std::vector<double> m_torques;
};

}  // namespace trickfire
