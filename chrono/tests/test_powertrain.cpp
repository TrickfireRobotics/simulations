// Motor, protection, thermal and battery equations, checked against hand calculations.
//
// These are the cheapest tests that would catch a sign error or a unit slip in the
// drivetrain, and they run without building a scene. The numbers here are derived from
// the model's own equations, not copied from a previous run, so they fail if the model
// changes rather than merely if it changes behaviour.

#include <cmath>

#include "check.hpp"
#include "powertrain/actuator.hpp"
#include "powertrain/battery.hpp"
#include "powertrain/drivetrain.hpp"

using namespace trickfire;

namespace {

/// A drive with round numbers: kt = 0.1 Nm/A, R = 0.5 ohm, 10:1 at 90%, 24 V, 10/30 A.
ActuatorSpec RoundNumbers() {
    ActuatorSpec spec;
    spec.name = "test";
    spec.model = "test";
    spec.command.mode = CommandPathSpec::Mode::TORQUE;
    spec.controller.torque_limit_nm = 0;  // unlimited, so the motor layer is isolated

    spec.motor.torque_constant = 0.1;
    spec.motor.back_emf_constant = 0.1;
    spec.motor.resistance_ohm = 0.5;
    spec.motor.rotor_inertia = 1e-4;
    spec.motor.gear_ratio = 10.0;
    spec.motor.gear_efficiency = 0.9;

    spec.protection.continuous_current_a = 10.0;
    spec.protection.peak_current_a = 30.0;
    spec.protection.peak_duration_s = 1.0;
    spec.protection.bus_voltage_limit_v = 24.0;

    spec.thermal.ambient_c = 25.0;
    return spec;
}

BatterySpec Pack() {
    BatterySpec spec;
    spec.name = "test";
    spec.ocv_curve = {{0.0, 20.0}, {1.0, 25.0}};
    spec.capacity_ah = 10.0;
    spec.internal_resistance_ohm = 0.05;
    spec.current_limit_a = 100.0;
    spec.initial_soc = 1.0;
    return spec;
}

void TestReflectedInertia() {
    Actuator drive(RoundNumbers(), 1e-3);
    // N^2 * J = 100 * 1e-4
    check::Near(drive.ReflectedInertia(), 1e-2, 1e-12,
                "reflected rotor inertia is N^2 times the rotor's own");
}

void TestTorquePath() {
    Actuator drive(RoundNumbers(), 1e-3);

    // Ask for 5 Nm at the output while stalled. Driving, so the gearbox loss divides:
    // motor torque = 5 / (10 * 0.9) = 0.5556 Nm, current = 5.556 A, within every limit.
    // The torque that comes back out is 0.5556 * 10 * 0.9 = 5 Nm again.
    const double applied = drive.Advance(0.0, 5.0, 0.0, 24.0);
    check::Near(applied, 5.0, 1e-9, "a request inside every limit is delivered exactly");
    check::Near(drive.State().current_a, 5.0 / (10.0 * 0.9) / 0.1, 1e-9,
                "current follows torque / (kt * N * efficiency)");
}

void TestCurrentLimit() {
    Actuator drive(RoundNumbers(), 1e-3);

    // 100 Nm needs 111 A; the peak budget is full, so it clamps to 30 A.
    const double applied = drive.Advance(0.0, 100.0, 0.0, 24.0);
    check::Near(drive.State().current_a, 30.0, 1e-9, "current clamps to the peak rating");
    check::True(drive.State().current_saturated, "saturation is reported");
    // 30 A * 0.1 Nm/A * 10 * 0.9 = 27 Nm
    check::Near(applied, 27.0, 1e-9, "the delivered torque matches the clamped current");
}

void TestPeakBudgetDrains() {
    ActuatorSpec spec = RoundNumbers();
    spec.protection.peak_duration_s = 0.01;  // ten steps at 1 ms
    Actuator drive(spec, 1e-3);

    for (int i = 0; i < 12; i++) drive.Advance(i * 1e-3, 100.0, 0.0, 24.0);

    check::Near(drive.State().peak_budget_s, 0.0, 1e-12, "the peak budget drains to zero");
    check::Near(drive.State().current_a, 10.0, 1e-9,
                "once the budget is spent the limit falls to continuous");
}

void TestVoltageCeiling() {
    ActuatorSpec spec = RoundNumbers();
    spec.protection.peak_current_a = 1000.0;  // take current out of the picture
    spec.protection.continuous_current_a = 1000.0;
    Actuator drive(spec, 1e-3);

    // At 20 rad/s output the motor turns at 200 rad/s, so back EMF is 20 V. With 24 V
    // available and 0.5 ohm, the winding can only take (24 - 20) / 0.5 = 8 A.
    drive.Advance(0.0, 1000.0, 20.0, 24.0);
    check::Near(drive.State().back_emf_v, 20.0, 1e-9, "back EMF is ke times motor speed");
    check::Near(drive.State().current_a, 8.0, 1e-9,
                "current is limited by the voltage left after back EMF");
    check::True(drive.State().voltage_saturated, "voltage saturation is reported");

    // This is the behaviour that makes a real drive lose torque as it speeds up, so it
    // is worth asserting the consequence as well as the cause.
    check::Less(drive.State().applied_torque, 27.0,
                "available torque falls once back EMF eats the bus voltage");
}

void TestSpeedLimit() {
    ActuatorSpec spec = RoundNumbers();
    spec.motor.speed_limit_rad_s = 5.0;
    Actuator drive(spec, 1e-3);

    drive.Advance(0.0, 5.0, 6.0, 24.0);
    check::True(drive.State().speed_limited, "driving past the speed limit is refused");
    check::Near(drive.State().applied_torque, 0.0, 1e-9, "no accelerating torque above the limit");

    // Braking must still be allowed, or the drive cannot slow down once overspeeding.
    Actuator braking(spec, 1e-3);
    braking.Advance(0.0, -5.0, 6.0, 24.0);
    check::Less(braking.State().applied_torque, 0.0, "braking torque is still allowed");
}

void TestThermalSteadyState() {
    ActuatorSpec spec = RoundNumbers();
    spec.thermal.thermal_resistance_c_per_w = 2.0;
    spec.thermal.thermal_capacitance_j_per_c = 10.0;  // tau = 20 s
    Actuator drive(spec, 1e-2);

    // Hold 10 A: copper loss is 10^2 * 0.5 = 50 W, so the steady state is
    // 25 + 50 * 2 = 125 C. Step well past five time constants.
    const double torque = 10.0 * 0.1 * 10.0 * 0.9;  // the request that draws exactly 10 A
    for (int i = 0; i < 20000; i++) drive.Advance(i * 1e-2, torque, 0.0, 24.0);

    check::Close(drive.State().temperature_c, 125.0, 0.01,
                 "steady temperature is ambient plus loss times thermal resistance");
}

void TestThermalTimeConstant() {
    ActuatorSpec spec = RoundNumbers();
    spec.thermal.thermal_resistance_c_per_w = 2.0;
    spec.thermal.thermal_capacitance_j_per_c = 10.0;  // tau = 20 s
    Actuator drive(spec, 1e-2);

    const double torque = 10.0 * 0.1 * 10.0 * 0.9;
    // After exactly one time constant a first-order step response has covered
    // 1 - 1/e = 63.2% of its range.
    for (int i = 0; i < 2000; i++) drive.Advance(i * 1e-2, torque, 0.0, 24.0);

    const double expected = 25.0 + (125.0 - 25.0) * (1.0 - std::exp(-1.0));
    check::Close(drive.State().temperature_c, expected, 0.01,
                 "the thermal step response matches a first-order lag");
}

void TestThermalDerating() {
    ActuatorSpec spec = RoundNumbers();
    spec.thermal.thermal_resistance_c_per_w = 2.0;
    spec.thermal.thermal_capacitance_j_per_c = 10.0;
    spec.thermal.derate_start_c = 60.0;
    spec.thermal.cutoff_c = 110.0;
    spec.protection.cutoff = ProtectionSpec::Cutoff::DERATE;
    Actuator drive(spec, 1e-2);

    for (int i = 0; i < 40000; i++) drive.Advance(i * 1e-2, 100.0, 0.0, 24.0);

    check::Less(drive.State().current_limit_a, 30.0,
                "the current limit is derated once the motor is hot");
    check::Less(drive.State().temperature_c, 130.0,
                "derating bounds the temperature instead of letting it run away");
}

void TestPowerBalance() {
    Actuator drive(RoundNumbers(), 1e-3);

    // 5 Nm at 2 rad/s output: 10 W of mechanical output. Motor side is 0.5556 Nm at
    // 20 rad/s = 11.11 W, which is the 10 W out plus 1.11 W of gearbox loss. Copper loss
    // is 5.556^2 * 0.5 = 15.43 W. Electrical input must be the sum.
    drive.Advance(0.0, 5.0, 2.0, 24.0);
    const auto& state = drive.State();

    check::Near(state.mech_power_w, 10.0, 1e-9, "mechanical power is torque times speed");

    const double copper = state.current_a * state.current_a * 0.5;
    const double motor_mech = state.current_a * 0.1 * (10.0 * 2.0);
    check::Near(state.bus_power_w, motor_mech + copper, 1e-6,
                "electrical input equals rotor power plus copper loss");
    check::Greater(state.bus_power_w, state.mech_power_w,
                   "more power goes in than comes out of the gearbox");
}

void TestBatterySag() {
    Battery pack(Pack(), 1e-3);
    check::Near(pack.TerminalVoltage(), 25.0, 1e-9, "a full pack sits at its open-circuit voltage");

    // 100 A through 0.05 ohm is a 5 V drop, so the terminals sit at 20 V.
    pack.Advance(100.0);
    check::Near(pack.State().terminal_v, 20.0, 1e-6, "terminal voltage sags by I times R");
    check::Near(pack.State().current_a, 100.0, 1e-6, "the drawn current is recorded");
}

void TestBatteryCurrentLimit() {
    BatterySpec spec = Pack();
    spec.current_limit_a = 50.0;
    Battery pack(spec, 1e-3);

    // Asking for 200 A against a 50 A limit should scale the drives back to a quarter.
    const double scale = pack.Advance(200.0);
    check::Near(scale, 0.25, 1e-9, "the pack scales the drives back to what it can supply");
    check::True(pack.State().current_limited, "the current limit is reported");
}

void TestBatteryRegenRejected() {
    BatterySpec spec = Pack();
    spec.accept_regen = false;
    Battery pack(spec, 1e-3);

    pack.Advance(-20.0);
    check::Greater(pack.State().regen_rejected_w, 0.0,
                   "a pack that cannot take regen reports the power as rejected");
    check::Near(pack.State().regen_accepted_w, 0.0, 1e-12, "and accepts none of it");
}

void TestBatteryDischarge() {
    BatterySpec spec = Pack();
    spec.capacity_ah = 1.0;
    Battery pack(spec, 1.0);  // one-second steps make the arithmetic obvious

    // 36 A for 100 s is 1 Ah, which is the whole pack.
    for (int i = 0; i < 100; i++) pack.Advance(36.0);
    check::Near(pack.State().soc, 0.0, 0.02, "state of charge tracks the amp-hours drawn");
}

void TestBatteryOcvInterpolation() {
    BatterySpec spec = Pack();
    spec.ocv_curve = {{0.0, 20.0}, {0.5, 23.0}, {1.0, 25.0}};
    Battery pack(spec, 1e-3);

    check::Near(pack.OpenCircuitVoltage(0.25), 21.5, 1e-9, "OCV interpolates between points");
    check::Near(pack.OpenCircuitVoltage(0.75), 24.0, 1e-9, "on both segments of the curve");
    check::Near(pack.OpenCircuitVoltage(-1.0), 20.0, 1e-9, "and clamps below the curve");
    check::Near(pack.OpenCircuitVoltage(2.0), 25.0, 1e-9, "and above it");
}

void TestDrivetrainSharesTheBus() {
    std::map<std::string, ActuatorSpec> actuators{{"test", RoundNumbers()}};
    std::vector<std::pair<std::string, std::string>> stations{
        {"fl", "test"}, {"fr", "test"}, {"bl", "test"}, {"br", "test"}};

    BatterySpec pack = Pack();
    pack.current_limit_a = 5.0;  // far too little for four drives at once

    Drivetrain drivetrain(actuators, stations, pack, 1e-3);
    const auto& torques = drivetrain.Advance(0.0, {100.0, 100.0, 100.0, 100.0}, {0, 0, 0, 0});

    check::True(torques.size() == 4, "one torque per station");
    // Every drive is scaled by the same factor, so a current limit cannot invent a
    // steering input on a shared bus.
    for (size_t i = 1; i < torques.size(); i++) {
        check::Near(torques[i], torques[0], 1e-9, "a pack-limited bus scales every drive equally");
    }
    check::Less(torques[0], 27.0, "and the torque is below what the drive alone would give");
}

}  // namespace

int main() {
    TestReflectedInertia();
    TestTorquePath();
    TestCurrentLimit();
    TestPeakBudgetDrains();
    TestVoltageCeiling();
    TestSpeedLimit();
    TestThermalSteadyState();
    TestThermalTimeConstant();
    TestThermalDerating();
    TestPowerBalance();
    TestBatterySag();
    TestBatteryCurrentLimit();
    TestBatteryRegenRejected();
    TestBatteryDischarge();
    TestBatteryOcvInterpolation();
    TestDrivetrainSharesTheBus();
    return check::Finish("powertrain");
}
