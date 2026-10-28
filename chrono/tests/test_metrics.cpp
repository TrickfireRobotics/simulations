// Metric accumulation, pass/fail checks and termination rules.
//
// The arithmetic matters as much as the physics: a time-weighted mean that silently
// becomes a sample mean, or a check on a metric nobody records that quietly passes, will
// not crash anything - it will just make a wheel comparison wrong.

#include <cmath>
#include <string>
#include <vector>

#include "check.hpp"
#include "experiments/metrics.hpp"
#include "experiments/termination.hpp"

using namespace trickfire;

namespace {

ScenarioSpec Scenario(double warmup = 0.0) {
    ScenarioSpec scenario;
    scenario.name = "test";
    scenario.end_time = 10.0;
    scenario.warmup_time = warmup;
    scenario.sample_rate_hz = 100;
    return scenario;
}

Sample MakeSample(double time, double speed, double distance) {
    Sample sample;
    sample.time = time;
    sample.speed = speed;
    sample.distance = distance;
    sample.stability_margin = 1.0;
    sample.wheels_in_contact = 2;

    for (const char* id : {"fl", "fr"}) {
        WheelSample wheel;
        wheel.id = id;
        wheel.speed = speed * 10.0;
        wheel.longitudinal_slip = 0.1;
        wheel.sinkage = 0.01;
        wheel.normal_load = 50.0;
        wheel.drive.applied_torque = 2.0;
        wheel.drive.current_a = 5.0;
        wheel.drive.temperature_c = 30.0;
        sample.wheels.push_back(wheel);
    }

    PivotSample pivot;
    pivot.name = "rocker_left";
    pivot.angle = 0.1;
    sample.pivots.push_back(pivot);

    sample.battery.soc = 0.9;
    sample.battery.cumulative_wh = distance * 0.5;
    sample.battery.terminal_v = 24.0;
    sample.battery.peak_current_a = 10.0;
    sample.battery.open_circuit_v = 25.0;
    sample.battery.min_terminal_v = 23.0;
    return sample;
}

void TestChannelStatistics() {
    ChannelStats stats;
    // Two seconds at 1, then one second at 4. The time-weighted mean is
    // (1*2 + 4*1) / 3 = 2, whereas a plain sample mean would give 2.5.
    stats.Add(1.0, 1.0, 0.0);
    stats.Add(1.0, 1.0, 0.0);
    stats.Add(4.0, 1.0, 0.0);

    check::Near(stats.min, 1.0, 1e-12, "the minimum is tracked");
    check::Near(stats.max, 4.0, 1e-12, "the maximum is tracked");
    check::Near(stats.integral / 3.0, 2.0, 1e-12, "the mean is time-weighted, not per-sample");
    check::Near(stats.last, 4.0, 1e-12, "the final value is kept");
}

void TestDurationAbove() {
    ChannelStats stats;
    stats.Add(1.0, 0.5, 0.5);  // above
    stats.Add(0.0, 0.5, 0.5);  // below
    stats.Add(1.0, 0.5, 0.5);  // above
    check::Near(stats.duration_above, 1.0, 1e-12,
                "time above a threshold is accumulated, not counted");
}

void TestNonFiniteIgnored() {
    ChannelStats stats;
    stats.Add(2.0, 1.0, 0.0);
    stats.Add(std::nan(""), 1.0, 0.0);
    stats.Add(std::numeric_limits<double>::infinity(), 1.0, 0.0);
    check::Near(stats.max, 2.0, 1e-12, "a diverged sample does not poison the statistics");
    check::Near(stats.last, 2.0, 1e-12, "and does not become the final value");
}

void TestWarmupExcluded() {
    Metrics metrics(Scenario(1.0), {"fl", "fr"}, {"rocker_left"});

    // A large spike inside the warm-up window must not set the peak.
    metrics.Add(MakeSample(0.0, 0.0, 0.0));
    metrics.Add(MakeSample(0.5, 99.0, 0.0));
    for (int i = 1; i <= 20; i++) metrics.Add(MakeSample(1.0 + i * 0.1, 1.0, i * 0.1));

    const auto summary = metrics.Summary();
    check::True(summary.count("mobility.speed_m_s.max") == 1, "the speed channel is recorded");
    check::Near(summary.at("mobility.speed_m_s.max"), 1.0, 1e-9,
                "a spike during the warm-up does not set the peak");
}

void TestDistanceAndEnergy() {
    Metrics metrics(Scenario(0.0), {"fl", "fr"}, {"rocker_left"});
    for (int i = 0; i <= 20; i++) metrics.Add(MakeSample(i * 0.1, 1.0, i * 0.1));

    const auto summary = metrics.Summary();
    check::Near(summary.at("mobility.distance_m"), 2.0, 1e-9, "distance is end minus start");
    check::Near(summary.at("energy.total_wh"), 1.0, 1e-9, "energy comes from the pack's total");
    check::Near(summary.at("energy.wh_per_m"), 0.5, 1e-9, "energy per metre divides the two");
    check::Near(summary.at("energy.voltage_sag_v"), 2.0, 1e-9,
                "voltage sag is open-circuit minus the worst terminal voltage");
}

void TestEnergyPerMetreGuarded() {
    // A run that never moved must not report an absurd efficiency.
    Metrics metrics(Scenario(0.0), {"fl", "fr"}, {"rocker_left"});
    for (int i = 0; i <= 20; i++) metrics.Add(MakeSample(i * 0.1, 0.0, 0.0));

    const auto summary = metrics.Summary();
    check::Near(summary.at("energy.wh_per_m"), 0.0, 1e-12,
                "a run that did not move reports zero energy per metre, not infinity");
}

void TestPerWheelChannels() {
    Metrics metrics(Scenario(0.0), {"fl", "fr"}, {"rocker_left"});
    for (int i = 0; i <= 10; i++) metrics.Add(MakeSample(i * 0.1, 1.0, i * 0.1));

    const auto summary = metrics.Summary();
    check::True(summary.count("wheel.fl.sinkage_m.mean") == 1, "per-wheel channels are recorded");
    check::True(summary.count("motor.fr.current_a.max") == 1, "per-drive channels are recorded");
    check::True(summary.count("suspension.rocker_left.angle_deg.mean") == 1,
                "per-pivot channels are recorded");
    check::Near(summary.at("wheel.mean_longitudinal_slip.mean"), 0.1, 1e-9,
                "the fleet-average slip is the mean across wheels");
}

void TestMetricGroupsFilter() {
    ScenarioSpec scenario = Scenario(0.0);
    scenario.metric_groups = {"mobility"};
    Metrics metrics(scenario, {"fl", "fr"}, {"rocker_left"});

    check::True(metrics.Wants("mobility"), "a requested group is enabled");
    check::True(!metrics.Wants("motor"), "an unrequested group is disabled");

    for (int i = 0; i <= 10; i++) metrics.Add(MakeSample(i * 0.1, 1.0, i * 0.1));
    const auto summary = metrics.Summary();
    check::True(summary.count("motor.fl.current_a.max") == 0, "a disabled group records nothing");
}

void TestChecksPassAndFail() {
    Metrics metrics(Scenario(0.0), {"fl", "fr"}, {"rocker_left"});
    for (int i = 0; i <= 20; i++) metrics.Add(MakeSample(i * 0.1, 1.0, i * 0.1));

    std::vector<CheckSpec> checks;
    checks.push_back({"mobility.distance_m", CheckSpec::Op::GT, 1.0, "moved far enough"});
    checks.push_back({"mobility.distance_m", CheckSpec::Op::LT, 1.0, "deliberately fails"});

    const auto results = metrics.EvaluateChecks(checks);
    check::True(results.size() == 2, "every check is evaluated");
    check::True(results[0].passed, "a satisfied check passes");
    check::True(!results[1].passed, "an unsatisfied check fails");
    check::Near(results[0].actual, 2.0, 1e-9, "the actual value is reported");
}

void TestCheckOnMissingMetricFails() {
    // The worst possible failure mode for a gate is to pass because it could not be
    // evaluated, so this asserts the opposite.
    Metrics metrics(Scenario(0.0), {"fl", "fr"}, {"rocker_left"});
    for (int i = 0; i <= 10; i++) metrics.Add(MakeSample(i * 0.1, 1.0, i * 0.1));

    std::vector<CheckSpec> checks;
    checks.push_back({"nothing.records.this", CheckSpec::Op::LT, 1.0, ""});

    const auto results = metrics.EvaluateChecks(checks);
    check::True(!results[0].evaluated, "a check on an unrecorded metric is marked unevaluated");
    check::True(!results[0].passed, "and fails rather than passing silently");
}

void TestTerminationDistance() {
    TerminationMonitor monitor({{TerminationSpec::Type::DISTANCE, 5.0, 0.0, true}}, 100.0);

    auto early = monitor.Check(MakeSample(1.0, 1.0, 4.9));
    check::True(early.reason == Termination::Reason::RUNNING, "short of the target it keeps going");

    auto hit = monitor.Check(MakeSample(2.0, 1.0, 5.1));
    check::True(hit.reason == Termination::Reason::RULE, "reaching the target stops the run");
    check::True(hit.pass, "and counts as a pass");
    check::True(hit.rule == "distance", "and names the rule");
}

void TestTerminationEndTime() {
    TerminationMonitor monitor({}, 5.0);
    check::True(monitor.Check(MakeSample(4.9, 1.0, 1.0)).reason == Termination::Reason::RUNNING,
                "before the end time the run continues");

    auto finished = monitor.Check(MakeSample(5.0, 1.0, 1.0));
    check::True(finished.reason == Termination::Reason::END_TIME, "the end time stops the run");
    check::True(finished.pass, "and reaching it is a pass");
}

void TestTerminationRollover() {
    TerminationMonitor monitor({{TerminationSpec::Type::ROLLOVER, 45.0, 0.0, false}}, 100.0);

    Sample tilted = MakeSample(1.0, 0.0, 0.0);
    tilted.chassis_roll = 50.0 * chrono::CH_DEG_TO_RAD;

    auto hit = monitor.Check(tilted);
    check::True(hit.reason == Termination::Reason::RULE, "exceeding the tilt limit stops the run");
    check::True(!hit.pass, "and counts as a failure");
}

void TestTerminationImmobility() {
    TerminationMonitor monitor({{TerminationSpec::Type::IMMOBILITY, 0.1, 2.0, false}}, 100.0);

    // Needs a full window of history first, so a run that has not got going yet is not
    // immediately declared stuck.
    for (int i = 0; i <= 10; i++) {
        auto result = monitor.Check(MakeSample(i * 0.1, 0.0, 0.0));
        check::True(result.reason == Termination::Reason::RUNNING,
                    "no immobility verdict before a full window has elapsed");
        if (result.reason != Termination::Reason::RUNNING) break;
    }

    Termination hit;
    for (int i = 11; i <= 40; i++) {
        hit = monitor.Check(MakeSample(i * 0.1, 0.0, 0.0));
        if (hit.reason != Termination::Reason::RUNNING) break;
    }
    check::True(hit.reason == Termination::Reason::RULE, "a stuck rover is eventually detected");
    check::True(hit.rule == "immobility", "and the rule is named");
}

void TestImmobilityNotTriggeredWhileMoving() {
    TerminationMonitor monitor({{TerminationSpec::Type::IMMOBILITY, 0.1, 2.0, false}}, 100.0);
    for (int i = 0; i <= 60; i++) {
        auto result = monitor.Check(MakeSample(i * 0.1, 1.0, i * 0.1));
        check::True(result.reason == Termination::Reason::RUNNING,
                    "a moving rover is never declared immobile");
        if (result.reason != Termination::Reason::RUNNING) break;
    }
}

void TestTerminationDiverged() {
    TerminationMonitor monitor({}, 100.0);
    Sample broken = MakeSample(1.0, 1.0, 1.0);
    broken.position = chrono::ChVector3d(std::nan(""), 0, 0);

    auto hit = monitor.Check(broken);
    check::True(hit.reason == Termination::Reason::DIVERGED, "a diverged solver stops the run");
    check::True(!hit.pass, "and is a failure");
}

void TestRuleOrderWins() {
    // A scenario can put its success condition first and have it win a tie with a
    // failure condition that fires on the same step.
    TerminationMonitor monitor({{TerminationSpec::Type::DISTANCE, 5.0, 0.0, true},
                                {TerminationSpec::Type::SINKAGE, 0.001, 0.0, false}},
                               100.0);
    auto hit = monitor.Check(MakeSample(1.0, 1.0, 6.0));
    check::True(hit.rule == "distance", "the first matching rule wins");
    check::True(hit.pass, "so the run is recorded as a pass");
}

}  // namespace

int main() {
    TestChannelStatistics();
    TestDurationAbove();
    TestNonFiniteIgnored();
    TestWarmupExcluded();
    TestDistanceAndEnergy();
    TestEnergyPerMetreGuarded();
    TestPerWheelChannels();
    TestMetricGroupsFilter();
    TestChecksPassAndFail();
    TestCheckOnMissingMetricFails();
    TestTerminationDistance();
    TestTerminationEndTime();
    TestTerminationRollover();
    TestTerminationImmobility();
    TestImmobilityNotTriggeredWhileMoving();
    TestTerminationDiverged();
    TestRuleOrderWins();
    return check::Finish("metrics");
}
