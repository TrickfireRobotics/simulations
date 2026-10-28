#pragma once

#include <deque>
#include <string>

#include "config/spec.hpp"

namespace trickfire {

/// Everything one drive reports after a step. All mechanical quantities are on the
/// output (wheel) side unless the name says "motor"; electrical quantities are on the
/// battery bus.
struct ActuatorState {
    double command = 0;           // request after rate limiting, sampling and delay
    double requested_torque = 0;  // Nm, controller output before any limit
    double applied_torque = 0;    // Nm, what actually reaches the wheel
    double output_speed = 0;      // rad/s
    double motor_speed = 0;       // rad/s

    double current_a = 0;        // motor phase current, signed
    double current_limit_a = 0;  // limit in force this step, after derating
    double back_emf_v = 0;
    double bus_voltage_v = 0;        // what the pack offered this step
    double voltage_utilisation = 0;  // |back emf| / bus voltage, 0..1+

    double bus_power_w = 0;   // positive draws from the pack, negative regenerates
    double mech_power_w = 0;  // applied_torque * output_speed
    double loss_w = 0;        // copper, gearbox and friction losses

    double temperature_c = 0;
    double derate_factor = 1;  // 1 = no thermal derating, 0 = cut off
    double peak_budget_s = 0;  // remaining time allowed above continuous current

    bool current_saturated = false;
    bool voltage_saturated = false;
    bool thermal_cutoff = false;
    bool speed_limited = false;
};

/// One RMD-X8-class drive: command shaping, controller, motor, protection and a lumped
/// thermal node.
///
/// The model is output-referred. Rotor inertia is folded into the wheel body as the
/// reflected inertia N^2 * J_rotor (see ReflectedInertia), which is exact for a rigid
/// gearbox and far better behaved at a 2 ms step than an explicit two-inertia pair.
///
/// Sign convention: positive output torque drives the wheel in the positive direction
/// about its spin axis, and positive bus power is drawn from the pack. Gearbox loss is
/// applied in whichever direction power is flowing, so a back-driven drive returns less
/// than the wheel puts in.
class Actuator {
  public:
    Actuator(ActuatorSpec spec, double step_size);

    /// Reflected rotor inertia about the output axis. The rover adds this to the wheel
    /// body's spin inertia rather than simulating a separate rotor state.
    double ReflectedInertia() const;

    /// Advance the drive by one step.
    ///
    /// `request` is a speed in rad/s or a torque in Nm depending on the spec's command
    /// mode, `output_speed` is the measured wheel speed about its spin axis, and
    /// `bus_voltage` is the pack terminal voltage from the previous step. Returns the
    /// torque to apply to the wheel this step.
    double Advance(double time, double request, double output_speed, double bus_voltage);

    /// Current the drive wants from the pack, available after Advance.
    double BusCurrent(double bus_voltage) const;

    void Reset();

    const ActuatorState& State() const { return m_state; }
    const ActuatorSpec& Spec() const { return m_spec; }
    const std::string& Name() const { return m_spec.name; }

  private:
    double ShapeCommand(double time, double request);
    double RunController(double command, double output_speed);
    double CurrentLimit() const;
    void UpdateThermal(double loss_w);

    ActuatorSpec m_spec;
    double m_step = 0;

    // Command path
    std::deque<std::pair<double, double>> m_delay_queue;  // (time, command)
    double m_last_command = 0;
    double m_held_command = 0;  // zero-order-hold output when sampled
    double m_last_sample_time = -1;

    // Controller
    double m_integral = 0;
    double m_last_error = 0;
    bool m_have_last_error = false;

    // Protection and thermal
    double m_temperature = 0;
    double m_peak_budget = 0;

    ActuatorState m_state;
};

}  // namespace trickfire
