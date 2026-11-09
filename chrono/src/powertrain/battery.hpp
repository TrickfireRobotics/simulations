#pragma once

#include "config/spec.hpp"

namespace trickfire {

struct BatteryState {
    double soc = 1.0;  // 0..1
    double open_circuit_v = 0;
    double terminal_v = 0;
    double current_a = 0;      // positive out of the pack
    double power_w = 0;        // at the terminals
    double cumulative_wh = 0;  // net energy delivered
    double peak_current_a = 0;
    double min_terminal_v = 0;

    double regen_accepted_w = 0;
    double regen_rejected_w = 0;

    bool current_limited = false;
    bool empty = false;
};

/// Pack-level electrical model: open-circuit voltage against state of charge, a series
/// internal plus wiring resistance, a current limit and an auxiliary load.
///
/// The drives and the pack are coupled explicitly: each drive sizes its current against
/// the terminal voltage from the previous step, and the pack then recomputes the terminal
/// voltage from the total current. At a 2 ms step the pack's electrical time constant is
/// far shorter than anything mechanical, so the lag is not what limits accuracy here -
/// the resistance and OCV numbers are.
class Battery {
  public:
    Battery(BatterySpec spec, double step_size);

    /// Terminal voltage to offer the drives this step.
    double TerminalVoltage() const { return m_state.terminal_v; }

    /// Settle the pack against the total drive current drawn this step (positive out).
    /// Returns the achievable fraction of that current, which the drivetrain uses to
    /// scale back every drive when the pack cannot supply what they asked for.
    double Advance(double drive_current_a);

    void Reset();

    const BatteryState& State() const { return m_state; }
    const BatterySpec& Spec() const { return m_spec; }

    /// Open-circuit voltage at a given state of charge, linearly interpolated and
    /// clamped to the ends of the curve.
    double OpenCircuitVoltage(double soc) const;

  private:
    BatterySpec m_spec;
    double m_step = 0;
    BatteryState m_state;
};

}  // namespace trickfire
