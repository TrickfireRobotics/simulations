#pragma once

#include <string>
#include <vector>

#include "config/spec.hpp"

namespace trickfire {

/// Turns a scenario's driver profile into one request per wheel.
///
/// The request is in whatever units the actuators' command path expects: rad/s of output
/// shaft speed, or Nm. Steering is skid-steer only, applied as a left-right bias, because
/// the rover has no steering joints.
class Driver {
  public:
    Driver(DriverSpec spec, std::vector<std::string> wheel_ids);

    /// Requests for this step, indexed the same way as the wheel ids.
    const std::vector<double>& Advance(double time);

    void Reset();

    /// Commanded forward speed before the steering bias, for path-error bookkeeping.
    double BaseCommand() const { return m_base; }

  private:
    double Profile(double time) const;

    DriverSpec m_spec;
    std::vector<std::string> m_wheel_ids;
    /// +1 for a left-side wheel, -1 for a right-side one, taken from the wheel id suffix.
    std::vector<double> m_side;

    std::vector<double> m_requests;
    std::vector<std::vector<double>> m_replay;  // rows of [time, per-wheel requests...]
    size_t m_replay_cursor = 0;
    double m_base = 0;
};

}  // namespace trickfire
