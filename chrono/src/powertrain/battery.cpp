#include "powertrain/battery.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace trickfire {

Battery::Battery(BatterySpec spec, double step_size) : m_spec(std::move(spec)), m_step(step_size) {
    Reset();
}

void Battery::Reset() {
    m_state = BatteryState{};
    m_state.soc = m_spec.initial_soc;
    m_state.open_circuit_v = OpenCircuitVoltage(m_state.soc);
    m_state.terminal_v = m_state.open_circuit_v;
    m_state.min_terminal_v = m_state.terminal_v;
}

double Battery::OpenCircuitVoltage(double soc) const {
    const auto& curve = m_spec.ocv_curve;
    if (curve.empty()) return 0;
    if (curve.size() == 1) return curve.front().second;

    soc = std::clamp(soc, 0.0, 1.0);
    if (soc <= curve.front().first) return curve.front().second;
    if (soc >= curve.back().first) return curve.back().second;

    for (size_t i = 1; i < curve.size(); i++) {
        if (soc <= curve[i].first) {
            const auto& [x0, y0] = curve[i - 1];
            const auto& [x1, y1] = curve[i];
            const double span = x1 - x0;
            if (span <= 0) return y1;
            return y0 + (y1 - y0) * (soc - x0) / span;
        }
    }
    return curve.back().second;
}

double Battery::Advance(double drive_current_a) {
    const double resistance = m_spec.internal_resistance_ohm + m_spec.wiring_resistance_ohm;
    const double ocv = OpenCircuitVoltage(m_state.soc);
    m_state.open_circuit_v = ocv;

    // Auxiliary electronics draw whether or not the rover is moving, so they come out of
    // the same bus and show up in the energy accounting.
    const double aux_current =
        (m_state.terminal_v > 0) ? m_spec.auxiliary_load_w / m_state.terminal_v : 0.0;
    double total = drive_current_a + aux_current;

    double scale = 1.0;
    m_state.current_limited = false;
    m_state.regen_accepted_w = 0;
    m_state.regen_rejected_w = 0;

    if (total > 0 && m_spec.current_limit_a > 0 && total > m_spec.current_limit_a) {
        // The pack cannot refuse the auxiliary load, so the shortfall is taken entirely
        // out of the drives.
        const double allowed_drive = std::max(0.0, m_spec.current_limit_a - aux_current);
        scale = drive_current_a > 0 ? allowed_drive / drive_current_a : 0.0;
        total = allowed_drive + aux_current;
        m_state.current_limited = true;
    } else if (total < 0) {
        const double regen_power = -total * m_state.terminal_v;
        if (!m_spec.accept_regen) {
            // Regenerated power has nowhere to go: the drives still produce the braking
            // torque, but the pack takes none of it back.
            m_state.regen_rejected_w = regen_power;
            scale = drive_current_a < 0 ? (0.0 - aux_current) / drive_current_a : 0.0;
            scale = std::clamp(scale, 0.0, 1.0);
            total = aux_current;
        } else {
            const double limit =
                m_spec.regen_current_limit_a > 0 ? m_spec.regen_current_limit_a : -total;
            if (-total > limit) {
                m_state.regen_rejected_w = (-total - limit) * m_state.terminal_v;
                m_state.regen_accepted_w = limit * m_state.terminal_v;
                total = -limit;
                m_state.current_limited = true;
            } else {
                m_state.regen_accepted_w = regen_power;
            }
        }
    }

    // A pack at zero charge can still be back-fed, but it cannot deliver.
    if (m_state.soc <= 0 && total > 0) {
        m_state.empty = true;
        scale = 0;
        total = 0;
    }

    m_state.current_a = total;
    m_state.terminal_v = std::max(0.0, ocv - total * resistance);
    m_state.power_w = m_state.terminal_v * total;

    if (m_spec.capacity_ah > 0) {
        const double amp_hours = total * m_step / 3600.0;
        m_state.soc = std::clamp(m_state.soc - amp_hours / m_spec.capacity_ah, 0.0, 1.0);
    }
    m_state.cumulative_wh += m_state.power_w * m_step / 3600.0;
    m_state.peak_current_a = std::max(m_state.peak_current_a, std::abs(total));
    m_state.min_terminal_v = m_state.min_terminal_v > 0
                                 ? std::min(m_state.min_terminal_v, m_state.terminal_v)
                                 : m_state.terminal_v;
    m_state.empty = m_state.soc <= 0;

    return std::clamp(scale, 0.0, 1.0);
}

}  // namespace trickfire
