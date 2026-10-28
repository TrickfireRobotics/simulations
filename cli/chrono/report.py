"""Reports: plots, comparison tables and a self-contained HTML page.

A wheel study's output has to be readable by someone who did not run it, which means
every number carries its units, every comparison shows the spread across seeds, and
nothing is collapsed into a single unexplained score. Where several seeds exist the
tables report mean +/- a 95% confidence interval, so a difference smaller than the
run-to-run noise is visibly not a difference.
"""

from __future__ import annotations

import base64
import html
import io
import math
import statistics
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..output import info, warn
from . import results

# Metrics every report shows when the study does not name its own objectives.
DEFAULT_OBJECTIVES = [
    {"metric": "mobility.distance_m", "direction": "maximise", "label": "Distance", "unit": "m"},
    {
        "metric": "energy.wh_per_m",
        "direction": "minimise",
        "label": "Energy per metre",
        "unit": "Wh/m",
    },
    {
        "metric": "motor.peak_temperature_c",
        "direction": "minimise",
        "label": "Peak motor temp",
        "unit": "C",
    },
    {
        "metric": "wheel.mean_longitudinal_slip.mean",
        "direction": "minimise",
        "label": "Mean slip",
        "unit": "",
    },
    {
        "metric": "wheel.mean_sinkage_m.mean",
        "direction": "minimise",
        "label": "Mean sinkage",
        "unit": "m",
    },
    {
        "metric": "stability.margin.mean",
        "direction": "maximise",
        "label": "Stability margin",
        "unit": "",
    },
]

# Channel groups plotted for a single run, as (title, y-axis label, column prefixes).
TIMESERIES_PANELS = [
    ("Speed and distance", "m/s, m", ["speed", "distance"]),
    ("Chassis attitude", "deg", ["chassis_roll_deg", "chassis_pitch_deg"]),
    ("Suspension angles", "deg", ["_angle_deg"]),
    ("Wheel slip", "", ["_slip"]),
    ("Sinkage", "m", ["_sinkage"]),
    ("Drive torque", "Nm", ["_torque"]),
    ("Drive current", "A", ["_current"]),
    ("Motor temperature", "C", ["_temperature"]),
    ("Bus", "V, A", ["bus_voltage", "bus_current"]),
    ("Energy", "Wh", ["cumulative_wh"]),
]


@dataclass
class Group:
    """Runs that differ only by seed, which is what a confidence interval is taken over."""

    key: tuple
    label: str
    runs: list[results.RunResult]

    def values(self, metric: str) -> list[float]:
        found = [r.metric(metric) for r in self.runs]
        return [v for v in found if v is not None and math.isfinite(v)]

    def stat(self, metric: str) -> tuple[float | None, float | None, int]:
        """Mean, half-width of the 95% CI, and the number of runs behind it."""
        values = self.values(metric)
        if not values:
            return None, None, 0
        mean = statistics.fmean(values)
        if len(values) < 2:
            return mean, None, 1

        # Student's t for a 95% two-sided interval at small n. Hard-coded rather than
        # pulling in scipy: a study with more than ~10 seeds is not what this is for, and
        # 1.96 is close enough beyond that.
        t = {
            2: 12.71,
            3: 4.303,
            4: 3.182,
            5: 2.776,
            6: 2.571,
            7: 2.447,
            8: 2.365,
            9: 2.306,
            10: 2.262,
        }.get(len(values), 1.96)
        half = t * statistics.stdev(values) / math.sqrt(len(values))
        return mean, half, len(values)

    @property
    def pass_rate(self) -> float:
        return sum(1 for r in self.runs if r.passed) / len(self.runs) if self.runs else 0.0


def group_runs(runs: list[results.RunResult]) -> list[Group]:
    """Group runs by everything except their seed."""
    buckets: dict[tuple, list[results.RunResult]] = defaultdict(list)

    for run in runs:
        wheels = run.manifest.get("rover", {}).get("wheels", [])
        packages = tuple(sorted({w.get("wheel_package", "") for w in wheels}))
        actuators = tuple(sorted({w.get("actuator", "") for w in wheels}))
        scenario = run.manifest.get("scenario", {})
        terrain = run.manifest.get("terrain", {})

        # Scenario overrides that a sweep's grid varied belong in the key, otherwise a
        # step-size sweep would average every step size together.
        grid = tuple(sorted((run.manifest.get("run", {}).get("grid") or {}).items()))

        key = (
            packages,
            actuators,
            terrain.get("name", ""),
            scenario.get("name", ""),
            terrain.get("pitch_deg", 0),
            terrain.get("grid_delta", 0),
            run.manifest.get("simulation", {}).get("step_size", 0),
            grid,
        )
        buckets[key].append(run)

    groups = []
    for key, bucket in sorted(buckets.items(), key=lambda item: str(item[0])):
        groups.append(
            Group(
                key,
                _group_label(bucket[0], key),
                sorted(bucket, key=lambda r: r.manifest.get("run", {}).get("seed", 0)),
            )
        )
    return groups


def _group_label(run: results.RunResult, key: tuple) -> str:
    parts = []
    if key[0] and key[0][0]:
        parts.append("+".join(p for p in key[0] if p))
    if key[1] and key[1][0]:
        parts.append(key[1][0])
    if key[2]:
        parts.append(key[2])
    if key[3]:
        parts.append(key[3])
    for name, value in key[7]:
        parts.append(f"{name}={value}")
    # Step size only appears when a study actually varied it, so an ordinary report is
    # not cluttered with it.
    return " / ".join(parts) or run.run_id


# ---------------------------------------------------------------------------- #
#                                     plots                                    #
# ---------------------------------------------------------------------------- #


def _matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt
    except ImportError:
        return None


def _figure_to_data_uri(figure) -> str:
    """Embed a figure in the page, so report.html is one portable file."""
    buffer = io.BytesIO()
    figure.savefig(buffer, format="png", dpi=110, bbox_inches="tight")
    encoded = base64.b64encode(buffer.getvalue()).decode()
    return f"data:image/png;base64,{encoded}"


def plot_timeseries(run: results.RunResult) -> list[tuple[str, str]]:
    """Per-channel-group plots for one run."""
    plt = _matplotlib()
    path = run.telemetry_path
    if plt is None or path is None:
        return []

    try:
        header, columns = results.read_telemetry(path)
    except Exception as exc:  # noqa: BLE001 - a truncated run should still report
        warn(f"{run.run_id}: could not read telemetry ({exc})")
        return []

    if "time" not in header:
        return []
    time = columns[header.index("time")]

    images = []
    for title, ylabel, prefixes in TIMESERIES_PANELS:
        selected = [
            (name, columns[index])
            for index, name in enumerate(header)
            if name != "time" and any(p in name for p in prefixes)
        ]
        if not selected:
            continue

        figure, axis = plt.subplots(figsize=(8, 2.6))
        for name, series in selected:
            axis.plot(time, series, linewidth=1.0, label=name)
        axis.set_title(title, fontsize=10)
        axis.set_xlabel("time (s)", fontsize=8)
        axis.set_ylabel(ylabel, fontsize=8)
        axis.tick_params(labelsize=7)
        axis.grid(True, linewidth=0.3, alpha=0.35)
        if len(selected) <= 8:
            axis.legend(fontsize=6, ncol=min(4, len(selected)), loc="best", framealpha=0.6)
        images.append((title, _figure_to_data_uri(figure)))
        plt.close(figure)
    return images


def plot_objective(groups: list[Group], objective: dict) -> tuple[str, str] | None:
    """A bar chart of one objective across groups, with error bars across seeds."""
    plt = _matplotlib()
    if plt is None:
        return None

    labels, means, errors = [], [], []
    for group in groups:
        mean, half, count = group.stat(objective["metric"])
        if mean is None:
            continue
        labels.append(group.label)
        means.append(mean)
        errors.append(half if half is not None and count > 1 else 0.0)

    if not means:
        return None

    figure, axis = plt.subplots(figsize=(max(6, 1.1 * len(labels)), 3.2))
    axis.bar(range(len(means)), means, yerr=errors, capsize=4, color="#2b6cb0", alpha=0.85)
    axis.set_xticks(range(len(labels)))
    axis.set_xticklabels(labels, rotation=30, ha="right", fontsize=7)

    unit = f" ({objective['unit']})" if objective.get("unit") else ""
    axis.set_ylabel(objective.get("label", objective["metric"]) + unit, fontsize=8)
    direction = objective.get("direction", "maximise")
    axis.set_title(
        f"{objective.get('label', objective['metric'])} ({'lower' if direction == 'minimise' else 'higher'} is better)",
        fontsize=10,
    )
    axis.tick_params(labelsize=7)
    axis.grid(True, axis="y", linewidth=0.3, alpha=0.35)

    image = _figure_to_data_uri(figure)
    plt.close(figure)
    return objective.get("label", objective["metric"]), image


# ---------------------------------------------------------------------------- #
#                                     HTML                                     #
# ---------------------------------------------------------------------------- #


STYLE = """
:root { color-scheme: light dark; }
body { font: 14px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif;
       margin: 0 auto; padding: 2rem 1.5rem 5rem; max-width: 1100px; }
h1 { font-size: 1.6rem; margin-bottom: 0.2rem; }
h2 { font-size: 1.15rem; margin-top: 2.2rem; border-bottom: 1px solid #8884;
     padding-bottom: 0.3rem; }
h3 { font-size: 1rem; margin-top: 1.4rem; }
.sub { opacity: 0.65; font-size: 0.85rem; margin-top: 0; }
table { border-collapse: collapse; width: 100%; margin: 0.8rem 0 1.4rem; font-size: 0.85rem; }
th, td { text-align: left; padding: 0.34rem 0.6rem; border-bottom: 1px solid #8883; }
th { font-weight: 600; opacity: 0.8; }
td.num { text-align: right; font-variant-numeric: tabular-nums; }
.best { font-weight: 700; }
.pass { color: #1a7f37; } .fail { color: #c9372c; } .na { opacity: 0.5; }
.warn { background: #ffb02e22; border-left: 3px solid #ffb02e;
        padding: 0.7rem 1rem; margin: 1rem 0; border-radius: 3px; }
img { max-width: 100%; display: block; margin: 0.6rem 0 1.2rem; }
code, pre { font-family: ui-monospace, "SF Mono", Menlo, monospace; font-size: 0.82em; }
pre { background: #8881; padding: 0.8rem; border-radius: 4px; overflow-x: auto; }
details { margin: 0.5rem 0; }
summary { cursor: pointer; font-weight: 600; }
.pill { display: inline-block; padding: 0.1rem 0.45rem; border-radius: 10px;
        font-size: 0.75rem; background: #8882; margin-right: 0.3rem; }
"""


def _format(value: float | None, half: float | None, count: int) -> str:
    if value is None:
        return "-"
    text = f"{value:.4g}"
    if half is not None and count > 1:
        text += f" &plusmn; {half:.3g}"
    return text


def build_report(study_path: Path) -> Path:
    """Build report.html for a study directory and return its path."""
    runs = results.load_study(study_path)
    if not runs:
        raise FileNotFoundError(f"no finished runs in {study_path}")

    groups = group_runs(runs)
    objectives = _objectives_for(study_path, runs)

    sections: list[str] = []
    sections.append(_header(study_path, runs, groups))
    sections.append(_provenance_warnings(runs))
    sections.append(_summary_table(groups, objectives))
    sections.append(_objective_charts(groups, objectives))
    sections.append(_checks_table(groups))
    sections.append(_run_sections(runs, len(groups)))
    sections.append(_inputs_section(runs))

    page = (
        "<!doctype html>\n<html lang='en'>\n<head>\n<meta charset='utf-8'>\n"
        f"<meta name='viewport' content='width=device-width, initial-scale=1'>\n"
        f"<title>{html.escape(study_path.name)}</title>\n<style>{STYLE}</style>\n"
        "</head>\n<body>\n" + "\n".join(s for s in sections if s) + "\n</body>\n</html>\n"
    )

    path = study_path / "report.html"
    path.write_text(page)
    info(f"report covers {len(runs)} run(s) in {len(groups)} group(s)")
    return path


def _objectives_for(study_path: Path, runs: list[results.RunResult]) -> list[dict]:
    """The study's own objectives if it recorded them, otherwise the defaults."""
    study_file = study_path / "study.yaml"
    if study_file.is_file():
        data = yaml.safe_load(study_file.read_text()) or {}
        if data.get("objectives"):
            return data["objectives"]

    # Only show defaults that at least one run actually recorded, so a scenario with a
    # reduced metric_groups does not produce a table of dashes.
    available = set()
    for run in runs:
        available |= set(run.metrics)
    return [o for o in DEFAULT_OBJECTIVES if o["metric"] in available] or DEFAULT_OBJECTIVES


def _header(study_path: Path, runs: list[results.RunResult], groups: list[Group]) -> str:
    passed = sum(1 for r in runs if r.passed)
    environment = runs[0].environment
    first = runs[0].manifest.get("run", {})

    pills = [
        f"{len(runs)} runs",
        f"{len(groups)} groups",
        f"{passed}/{len(runs)} passed",
    ]
    if first.get("git_commit"):
        pills.append(f"repo {first['git_commit'][:12]}")
    if environment.get("chrono_commit"):
        pills.append(f"chrono {environment['chrono_commit'][:12]}")
    if environment.get("host"):
        pills.append(environment["host"])

    return (
        f"<h1>{html.escape(study_path.name)}</h1>\n"
        f"<p class='sub'>"
        + " ".join(f"<span class='pill'>{html.escape(str(p))}</span>" for p in pills)
        + "</p>\n"
    )


def _provenance_warnings(runs: list[results.RunResult]) -> str:
    """Say plainly which inputs were not measured.

    This is the most important block on the page. A reader who takes a wheel ranking from
    a report built on estimated soil and an invented actuator needs to be told, in the
    report, that it is a ranking on a fictional system.
    """
    weak: dict[str, set[str]] = defaultdict(set)

    for run in runs:
        manifest = run.manifest
        for soil in manifest.get("terrain", {}).get("soils", []):
            quality = (soil.get("provenance") or {}).get("quality", "placeholder")
            if quality in {"placeholder", "estimated"}:
                weak[f"soil '{soil.get('name')}'"].add(quality)
        for actuator in manifest.get("actuators", []):
            quality = (actuator.get("provenance") or {}).get("quality", "placeholder")
            if quality in {"placeholder", "estimated"}:
                weak[f"actuator '{actuator.get('name')}'"].add(quality)
        battery = manifest.get("battery", {})
        quality = (battery.get("provenance") or {}).get("quality", "placeholder")
        if quality in {"placeholder", "estimated"}:
            weak[f"battery '{battery.get('name')}'"].add(quality)
        for wheel in manifest.get("wheels", []):
            quality = wheel.get("mass_properties", {}).get("provenance", "placeholder")
            if quality in {"placeholder", "estimated"}:
                weak[f"wheel '{wheel.get('name')}' mass properties"].add(quality)
        chassis = manifest.get("rover", {}).get("chassis", {}).get("mass_properties", {})
        if chassis.get("provenance") in {"placeholder", "estimated"}:
            weak["rover chassis mass properties"].add(chassis["provenance"])

    if not weak:
        return ""

    items = "".join(
        f"<li>{html.escape(name)} &mdash; <code>{'/'.join(sorted(q))}</code></li>"
        for name, q in sorted(weak.items())
    )
    return (
        "<div class='warn'><strong>Simulation is not validation.</strong> These inputs "
        "were not measured against hardware:<ul>" + items + "</ul>"
        "Rankings below are internally consistent and comparable to each other, but they "
        "are rankings on this model, not predictions about the rover. See "
        "<code>docs/chrono/development-plan.mdx</code>, Phase 6.</div>\n"
    )


def _summary_table(groups: list[Group], objectives: list[dict]) -> str:
    rows = []
    best: dict[str, float] = {}

    for objective in objectives:
        values = [
            group.stat(objective["metric"])[0]
            for group in groups
            if group.stat(objective["metric"])[0] is not None
        ]
        if values:
            best[objective["metric"]] = (
                min(values) if objective.get("direction") == "minimise" else max(values)
            )

    for group in groups:
        cells = [f"<td>{html.escape(group.label)}</td>"]
        rate = group.pass_rate
        verdict = "pass" if rate == 1 else ("fail" if rate == 0 else "")
        cells.append(
            f"<td class='num {verdict}'>{int(rate * len(group.runs))}/{len(group.runs)}</td>"
        )
        for objective in objectives:
            mean, half, count = group.stat(objective["metric"])
            classes = ["num"]
            if (
                mean is not None
                and objective["metric"] in best
                and abs(mean - best[objective["metric"]]) < 1e-12
            ):
                classes.append("best")
            cells.append(f"<td class='{' '.join(classes)}'>{_format(mean, half, count)}</td>")
        rows.append("<tr>" + "".join(cells) + "</tr>")

    headers = ["Configuration", "Passed"] + [
        html.escape(o.get("label", o["metric"]))
        + (f" ({html.escape(o['unit'])})" if o.get("unit") else "")
        + ("&nbsp;&darr;" if o.get("direction") == "minimise" else "&nbsp;&uarr;")
        for o in objectives
    ]

    return (
        "<h2>Comparison</h2>\n"
        "<p class='sub'>Mean across seeds, &plusmn; a 95% confidence interval where "
        "more than one seed was run. Arrows show which direction is better; bold is the "
        "best value in the column. A gap smaller than the intervals is not a difference."
        "</p>\n<table>\n<thead><tr>"
        + "".join(f"<th>{h}</th>" for h in headers)
        + "</tr></thead>\n<tbody>\n"
        + "\n".join(rows)
        + "\n</tbody>\n</table>\n"
    )


def _objective_charts(groups: list[Group], objectives: list[dict]) -> str:
    if len(groups) < 2:
        return ""

    blocks = []
    for objective in objectives:
        chart = plot_objective(groups, objective)
        if chart:
            title, uri = chart
            blocks.append(f"<img alt='{html.escape(title)}' src='{uri}'>")

    if not blocks:
        return ""
    return "<h2>Objectives</h2>\n" + "\n".join(blocks)


def _checks_table(groups: list[Group]) -> str:
    rows = []

    def row(label: str, seed, check: str, actual, state: str, meaning: str) -> str:
        number = f"{actual:.4g}" if isinstance(actual, (int, float)) else "-"
        return (
            f"<tr><td>{html.escape(label)}</td><td class='num'>{html.escape(str(seed))}</td>"
            f"<td><code>{html.escape(check)}</code></td>"
            f"<td class='num'>{number}</td>"
            f"<td class='fail'>{html.escape(state)}</td>"
            f"<td>{html.escape(meaning)}</td></tr>"
        )

    for group in groups:
        for run in group.runs:
            seed = run.manifest.get("run", {}).get("seed", "?")
            for check in run.checks:
                if check["passed"]:
                    continue
                # A check whose metric was never recorded is reported as "not evaluated"
                # rather than as a failure of the rover: the gate itself is broken.
                state = "not evaluated" if not check.get("evaluated", True) else "failed"
                rows.append(
                    row(
                        group.label,
                        seed,
                        check["expression"],
                        check.get("actual"),
                        state,
                        check.get("description", ""),
                    )
                )

            termination = run.termination
            if not termination.get("pass", True):
                rows.append(
                    row(
                        group.label,
                        seed,
                        f"termination: {termination.get('rule', '')}",
                        None,
                        "stopped",
                        termination.get("detail", ""),
                    )
                )

    if not rows:
        return "<h2>Failures</h2>\n<p>Every check passed in every run.</p>\n"
    return (
        "<h2>Failures</h2>\n<table>\n<thead><tr><th>Configuration</th><th>Seed</th>"
        "<th>Check</th><th>Actual</th><th>State</th><th>Meaning</th></tr></thead>\n"
        "<tbody>\n" + "\n".join(rows) + "\n</tbody>\n</table>\n"
    )


def _run_sections(runs: list[results.RunResult], group_count: int) -> str:
    """Time-series plots. Every run when there are few, otherwise one per group."""
    # Plotting fifty runs produces a page nobody opens, so a large sweep shows one
    # representative run per group and the rest stay in their own directories.
    if len(runs) > 8:
        chosen = []
        seen = set()
        for run in runs:
            key = run.label().rsplit("|", 1)[0]
            if key not in seen:
                seen.add(key)
                chosen.append(run)
        chosen = chosen[:8]
        note = (
            f"<p class='sub'>{len(runs)} runs in this study; showing one per "
            f"configuration. Every run's own telemetry is in its directory.</p>\n"
        )
    else:
        chosen = runs
        note = ""

    blocks = []
    for run in chosen:
        images = plot_timeseries(run)
        if not images:
            continue
        body = "\n".join(f"<img alt='{html.escape(title)}' src='{uri}'>" for title, uri in images)
        verdict = (
            "<span class='pass'>PASS</span>" if run.passed else "<span class='fail'>FAIL</span>"
        )
        blocks.append(
            f"<details><summary>{html.escape(run.label())} &mdash; {verdict}</summary>\n"
            f"{body}</details>"
        )

    if not blocks:
        return (
            "<h2>Time series</h2>\n<p class='sub'>No plots: matplotlib is not installed, "
            "or no telemetry was readable.</p>\n"
        )
    return "<h2>Time series</h2>\n" + note + "\n".join(blocks)


def _inputs_section(runs: list[results.RunResult]) -> str:
    """The traceable-inputs block: what produced these numbers."""
    first = runs[0]
    provenance = first.manifest.get("provenance", {})
    environment = first.environment

    lines = []
    for kind in ("scenario", "rover", "terrain", "battery"):
        entry = provenance.get(kind)
        if isinstance(entry, dict):
            chain = " <- ".join(Path(p).name for p in entry.get("extends_chain", []))
            lines.append(
                f"{kind:<10} {entry.get('file', '?')}" + (f"   [{chain}]" if chain else "")
            )

    for name, entry in (provenance.get("wheels") or {}).items():
        lines.append(
            f"wheel      {name}  rev {entry.get('revision', '?')}  "
            f"{entry.get('source_hash', '')[:24]}"
        )
    for name, entry in (provenance.get("actuators") or {}).items():
        lines.append(f"actuator   {name}  {entry.get('resolved_hash', '')[:24]}")
    for name, entry in (provenance.get("soils") or {}).items():
        lines.append(f"soil       {name}  {entry.get('resolved_hash', '')[:24]}")

    for key in (
        "chrono_version",
        "chrono_commit",
        "compiler",
        "build_type",
        "host",
        "hardware_threads",
    ):
        if environment.get(key) not in (None, ""):
            lines.append(f"{key:<10} {environment[key]}")

    solver = environment.get("solver", {})
    for key, value in solver.items():
        lines.append(f"{key:<10} {value}")

    return (
        "<h2>Traceable inputs</h2>\n<p class='sub'>Every run's own "
        "<code>manifest.yaml</code> holds the fully resolved inputs by value, so these "
        "results do not change when a config is edited later.</p>\n"
        "<pre>" + html.escape("\n".join(lines)) + "</pre>\n"
    )


# ---------------------------------------------------------------------------- #
#                                  comparison                                  #
# ---------------------------------------------------------------------------- #


def print_comparison(study_paths: list[Path], metrics: list[str]) -> None:
    """Print a cross-study comparison table to the terminal."""
    all_runs: list[results.RunResult] = []
    for path in study_paths:
        runs = results.load_study(path)
        if not runs:
            warn(f"no finished runs in {path}")
        all_runs += runs

    if not all_runs:
        warn("nothing to compare")
        return

    groups = group_runs(all_runs)
    keys = metrics or [o["metric"] for o in DEFAULT_OBJECTIVES]

    # Only keep metrics at least one run recorded, so the table has no all-dash columns.
    available = set()
    for run in all_runs:
        available |= set(run.metrics)
    keys = [k for k in keys if k in available]

    label_width = max(len(g.label) for g in groups) + 2

    # Keep the last two dotted segments, not just the last: half the summary keys end in
    # ".mean", so a one-segment header makes several columns read identically.
    def short(key: str) -> str:
        parts = key.split(".")
        return ".".join(parts[-2:]) if len(parts) > 1 else key

    header = f"{'configuration':<{label_width}} {'pass':>7}"
    for key in keys:
        header += f" {short(key)[:20]:>22}"
    print(header)
    print("-" * len(header))

    for group in groups:
        row = f"{group.label:<{label_width}} "
        row += f"{int(group.pass_rate * len(group.runs))}/{len(group.runs):<5}"
        for key in keys:
            mean, half, count = group.stat(key)
            if mean is None:
                row += f" {'-':>22}"
            elif half is not None and count > 1:
                row += f" {f'{mean:.4g} +- {half:.2g}':>22}"
            else:
                row += f" {mean:>22.4g}"
        print(row)

    weak = _provenance_warnings(all_runs)
    if weak:
        print()
        warn(
            "some inputs in this comparison were not measured against hardware; the "
            "report's HTML version lists which ones"
        )
