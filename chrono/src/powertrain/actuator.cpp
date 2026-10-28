#include "powertrain/actuator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace trickfire {

namespace {

double Sign(double value) { return value > 0 ? 1.0 : (value < 0 ? -1.0 : 0.0); }

double Clamp(double value, double limit) {
    if (limit <= 0) return value;
    return std::clamp(value, -limit, limit);
}

}  // namespace

Actuator::Actuator(ActuatorSpec spec, double step_size)
    : m_spec(std::move(spec)), m_step(step_size) {
    Reset();
}

void Actuator::Reset() {
    m_delay_queue.clear();
    m_last_command = 0;
    m_held_command = 0;
    m_last_sample_time = -1;
    m_integral = 0;
    m_last_error = 0;
    m_have_last_error = false;
    m_temperature = m_spec.thermal.ambient_c;
    m_peak_budget = m_spec.protection.peak_duration_s;

    m_state = ActuatorState{};
    m_state.temperature_c = m_temperature;
    m_state.peak_budget_s = m_peak_budget;
    m_state.current_limit_a = m_spec.protection.continuous_current_a;
}

double Actuator::ReflectedInertia() const {
    const double n = m_spec.motor.gear_ratio;
    return n * n * m_spec.motor.rotor_inertia;
}

// ---------------------------------------------------------------------------- //
//                                 command path                                 //
// ---------------------------------------------------------------------------- //

double Actuator::ShapeCommand(double time, double request) {
    // Sampled update rate: the controller only sees a new request at its own loop rate,
    // holding the previous one in between.
    double sampled = request;
    if (m_spec.command.update_rate_hz > 0) {
        const double period = 1.0 / m_spec.command.update_rate_hz;
        if (m_last_sample_time < 0 || time - m_last_sample_time >= period) {
            m_held_command = request;
            m_last_sample_time = time;
        }
        sampled = m_held_command;
    }

    // Rate limit, applied to the sampled request so a slew limit cannot be defeated by
    // a faster sample rate.
    double limited = sampled;
    if (m_spec.command.rate_limit > 0) {
        const double max_delta = m_spec.command.rate_limit * m_step;
        limited = m_last_command + std::clamp(sampled - m_last_command, -max_delta, max_delta);
    }
    m_last_command = limited;

    // Transport delay. The queue holds future commands tagged with the time they become
    // visible, which keeps the delay exact rather than rounded to whole steps.
    if (m_spec.command.delay_s <= 0) return limited;

    m_delay_queue.emplace_back(time + m_spec.command.delay_s, limited);

    double effective = 0;
    while (!m_delay_queue.empty() && m_delay_queue.front().first <= time) {
        effective = m_delay_queue.front().second;
        m_delay_queue.pop_front();
    }
    return effective;
}

double Actuator::RunController(double command, double output_speed) {
    if (m_spec.command.mode == CommandPathSpec::Mode::TORQUE) {
        return Clamp(command, m_spec.controller.torque_limit_nm);
    }

    const double error = command - output_speed;

    // Trapezoidal integration with an explicit anti-windup clamp. Without the clamp the
    // integral keeps growing while the current limiter holds the output down, and the
    // drive then overshoots badly the moment traction returns.
    m_integral += 0.5 * (error + (m_have_last_error ? m_last_error : error)) * m_step;
    if (m_spec.controller.integrator_limit > 0) {
        m_integral = std::clamp(m_integral, -m_spec.controller.integrator_limit,
                                m_spec.controller.integrator_limit);
    }

    double derivative = 0;
    if (m_have_last_error && m_step > 0) derivative = (error - m_last_error) / m_step;
    m_last_error = error;
    m_have_last_error = true;

    const double torque = m_spec.controller.kp * error + m_spec.controller.ki * m_integral +
                          m_spec.controller.kd * derivative;
    return Clamp(torque, m_spec.controller.torque_limit_nm);
}

// ---------------------------------------------------------------------------- //
//                              protection and heat                            //
// ---------------------------------------------------------------------------- //

double Actuator::CurrentLimit() const {
    const auto& p = m_spec.protection;

    // The peak budget models the controller's I^2t allowance: it drains while the drive
    // pulls more than its continuous rating and refills while it pulls less.
    double limit = m_peak_budget > 0 ? p.peak_current_a : p.continuous_current_a;

    const auto& t = m_spec.thermal;
    if (t.cutoff_c <= 0 || t.derate_start_c <= 0) return limit;

    switch (p.cutoff) {
        case ProtectionSpec::Cutoff::DISABLE:
            // No warning region: full output right up to the trip, then nothing.
            return m_temperature >= t.cutoff_c ? 0 : limit;

        case ProtectionSpec::Cutoff::DERATE:
        case ProtectionSpec::Cutoff::FOLD_BACK: {
            if (m_temperature <= t.derate_start_c) return limit;

            const double span = t.cutoff_c - t.derate_start_c;
            const double ramp =
                span > 0 ? std::clamp(1.0 - (m_temperature - t.derate_start_c) / span, 0.0, 1.0)
                         : 0.0;

            // Fold-back limps on at a reduced current instead of stopping the rover dead,
            // which is what lets a thermal-limit run still report distance travelled.
            const double floor =
                p.cutoff == ProtectionSpec::Cutoff::FOLD_BACK ? 0.25 * p.continuous_current_a : 0.0;
            return std::max(floor, limit * ramp);
        }
    }
    return limit;
}

void Actuator::UpdateThermal(double loss_w) {
    const auto& t = m_spec.thermal;
    if (t.thermal_capacitance_j_per_c <= 0 || t.thermal_resistance_c_per_w <= 0) {
        m_temperature = t.ambient_c;
        return;
    }

    // Lumped single node: C dT/dt = P_loss - (T - T_ambient) / R.
    // Solved with the exact exponential rather than explicit Euler, so a step larger
    // than the thermal time constant stays stable instead of oscillating.
    const double tau = t.thermal_resistance_c_per_w * t.thermal_capacitance_j_per_c;
    const double steady = t.ambient_c + loss_w * t.thermal_resistance_c_per_w;
    const double decay = std::exp(-m_step / tau);
    m_temperature = steady + (m_temperature - steady) * decay;
}

// ---------------------------------------------------------------------------- //
//                                    stepping                                  //
// ---------------------------------------------------------------------------- //

double Actuator::Advance(double time, double request, double output_speed, double bus_voltage) {
    const auto& motor = m_spec.motor;
    const auto& prot = m_spec.protection;

    const double n = motor.gear_ratio;
    const double eff = motor.gear_efficiency;
    const double motor_speed = n * output_speed;

    m_state = ActuatorState{};
    m_state.output_speed = output_speed;
    m_state.motor_speed = motor_speed;

    const double command = ShapeCommand(time, request);
    m_state.command = command;

    double torque_request = RunController(command, output_speed);
    m_state.requested_torque = torque_request;

    // Output speed limit: refuse torque that would push the drive further past its rated
    // speed, but still allow braking torque.
    if (motor.speed_limit_rad_s > 0 && std::abs(output_speed) > motor.speed_limit_rad_s) {
        if (Sign(torque_request) == Sign(output_speed)) {
            torque_request = 0;
            m_state.speed_limited = true;
        }
    }

    // Refer the output-side request back to the motor shaft. Which way the gearbox loss
    // applies depends on which way power is flowing, not on the sign of the torque.
    const bool driving = torque_request * output_speed >= 0;
    const double motor_torque_request =
        driving ? torque_request / (n * eff) : torque_request * eff / n;

    double current = motor_torque_request / motor.torque_constant;

    // Bus voltage headroom. The winding can only be pushed to (V - back emf) / R, which
    // is what makes a real drive lose torque as it speeds up.
    const double back_emf = motor.back_emf_constant * motor_speed;
    m_state.back_emf_v = back_emf;
    m_state.bus_voltage_v = bus_voltage;

    const double usable_voltage = prot.bus_voltage_limit_v > 0
                                      ? std::min(bus_voltage, prot.bus_voltage_limit_v)
                                      : bus_voltage;

    if (motor.resistance_ohm > 0 && usable_voltage > 0) {
        const double headroom = usable_voltage - std::abs(back_emf);
        const double voltage_current = std::max(0.0, headroom) / motor.resistance_ohm;
        if (std::abs(current) > voltage_current) {
            current = Sign(current) * voltage_current;
            m_state.voltage_saturated = true;
        }
    }
    m_state.voltage_utilisation = usable_voltage > 0 ? std::abs(back_emf) / usable_voltage : 0;

    // Current protection and thermal derating.
    //
    // The peak budget is drained against the *demanded* current, not the current that
    // survives the clamp. Using the clamped value lets a saturated drive refill its
    // budget the instant the limiter holds it at the continuous rating, so it earns peak
    // allowance back every other step and sustains roughly half of peak indefinitely -
    // which is not what an I^2t limiter does.
    const double demanded = std::abs(current);

    const double limit = CurrentLimit();
    m_state.current_limit_a = limit;
    if (demanded > limit) {
        current = Sign(current) * limit;
        m_state.current_saturated = true;
    }
    m_state.thermal_cutoff = limit <= 0;

    if (prot.peak_duration_s > 0) {
        if (demanded > prot.continuous_current_a) {
            m_peak_budget = std::max(0.0, m_peak_budget - m_step);
        } else {
            // Refills over the same timescale it drains, which is the simplest assumption
            // that keeps a duty cycle above continuous from running indefinitely.
            m_peak_budget = std::min(prot.peak_duration_s, m_peak_budget + m_step);
        }
    }
    m_state.peak_budget_s = m_peak_budget;
    m_state.current_a = current;

    // Back out what the motor actually produces, net of its own drag.
    const double electromagnetic_torque = current * motor.torque_constant;
    const double drag =
        motor.viscous_damping * motor_speed + motor.coulomb_friction_nm * Sign(motor_speed);
    const double motor_torque = electromagnetic_torque - drag;

    const bool output_driving = motor_torque * motor_speed >= 0;
    const double applied = output_driving ? motor_torque * n * eff : motor_torque * n / eff;

    m_state.applied_torque = applied;
    m_state.mech_power_w = applied * output_speed;

    const double copper_loss = current * current * motor.resistance_ohm;
    const double motor_mech_power = motor_torque * motor_speed;
    const double gear_loss = std::abs(motor_mech_power) * (1.0 - eff);
    const double friction_loss = std::abs(drag * motor_speed);
    m_state.loss_w = copper_loss + gear_loss + friction_loss;

    // Electrical input is the power crossing the motor terminals: what the rotor takes
    // plus what the winding burns. Negative means the drive is regenerating.
    m_state.bus_power_w = electromagnetic_torque * motor_speed + copper_loss;

    UpdateThermal(copper_loss + friction_loss);
    m_state.temperature_c = m_temperature;
    const auto& t = m_spec.thermal;
    if (t.cutoff_c > 0 && t.derate_start_c > 0) {
        const double span = t.cutoff_c - t.derate_start_c;
        m_state.derate_factor =
            span > 0 ? std::clamp(1.0 - (m_temperature - t.derate_start_c) / span, 0.0, 1.0) : 1.0;
    }

    return applied;
}

double Actuator::BusCurrent(double bus_voltage) const {
    if (bus_voltage <= 0) return 0;
    return m_state.bus_power_w / bus_voltage;
}

}  // namespace trickfire
