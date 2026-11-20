import type { Diagnostic } from "../types";
import { escapeHtml, renderLogEntry } from "./html";
import type { UiState } from "./state";

const toggle = (field: string, label: string, detail: string, checked: boolean): string => `
    <label class="toggle-row">
        <span><strong>${escapeHtml(label)}</strong><small>${escapeHtml(detail)}</small></span>
        <input class="toggle-input" type="checkbox" data-field="${field}" aria-label="${escapeHtml(label)}" ${checked ? "checked" : ""} />
        <span class="toggle-track" aria-hidden="true"><i></i></span>
    </label>`;

const diagnostic = (item: Diagnostic): string => `
    <li class="diagnostic status-${item.status}">
        <i aria-hidden="true"></i>
        <span><strong>${escapeHtml(item.label)}</strong><small>${escapeHtml(item.detail)}</small></span>
    </li>`;

export const diagnosticsView = (state: UiState): string => {
    if (state.checking) {
        return ["simulation files", "runtime", "display", "service"]
            .map(
                (label) => `
                    <li class="diagnostic is-loading">
                        <i aria-hidden="true"></i>
                        <span><strong>${label}</strong><small>checking...</small></span>
                    </li>`
            )
            .join("");
    }
    if (!state.diagnostics.length) {
        return `
            <li class="diagnostic is-empty">
                <i aria-hidden="true"></i>
                <span><strong>host unchecked</strong><small>check the host before the first run</small></span>
            </li>`;
    }
    return state.diagnostics.map(diagnostic).join("");
};

export const logsView = (state: UiState): string => {
    if (!state.logs.length) {
        return `<div class="log-empty">output appears here when a process starts</div>`;
    }
    return state.logs.map(renderLogEntry).join("");
};

const gazeboToggles = (state: UiState): string => {
    const items = [
        toggle("gazeboGui", "gazebo window", "3D simulation view", state.gazeboGui),
        state.target !== "drone" ? toggle("rviz", "rviz", "robot visualization", state.rviz) : "",
        state.mode === "docker" && state.target !== "drone"
            ? toggle("controller", "keyboard controller", "virtual gamepad", state.controller)
            : "",
        state.target === "viator" || state.target === "arm"
            ? toggle("jointGui", "joint controls", "arm joint panel", state.jointGui)
            : "",
        state.target === "viator" ? toggle("ttt", "TTT task", "spawn task model", state.ttt) : "",
        state.target === "viator"
            ? toggle("arTags", "AR tags", "spawn detection tags", state.arTags)
            : "",
        state.target === "viator"
            ? toggle("followRover", "follow rover", "track with the camera", state.followRover)
            : "",
        state.target === "drone"
            ? toggle("localQgc", "local QGroundControl", "skip Docker QGC", state.localQgc)
            : ""
    ];
    return items.join("");
};

const advancedView = (state: UiState): string => `
    <div class="advanced-fields">
        ${
            state.kind === "gazebo"
                ? `
                    <label class="field compact-field">
                        <span>ROS bridge port</span>
                        <input type="number" min="1" max="65535" value="${state.rosbridgePort}" data-field="rosbridgePort" />
                    </label>
                    <label class="field wide-field">
                        <span>extra ROS launch arguments</span>
                        <input type="text" value="${escapeHtml(state.extraArgs)}" data-field="extraArgs" placeholder="zed_frame_rate:=30" />
                    </label>`
                : ""
        }
        <div class="bridge-row">
            ${toggle("bridgeEnabled", "trickfire-urc bridge", "manage the sim-side Zenoh bridge", state.bridgeEnabled)}
        </div>
        ${
            state.bridgeEnabled
                ? `
                    <button class="path-picker" type="button" data-action="choose-urc">
                        <span>${state.urcRepositoryPath ? escapeHtml(state.urcRepositoryPath) : "choose trickfire-urc folder"}</span>
                        <b>browse</b>
                    </button>
                    <label class="field compact-field">
                        <span>point cloud limit</span>
                        <div class="input-unit"><input type="number" min="0.1" max="30" step="0.1" value="${escapeHtml(state.maxPointCloudHz)}" data-field="maxPointCloudHz" /><i>Hz</i></div>
                    </label>`
                : ""
        }
    </div>`;

export const settingsView = (state: UiState): string => {
    if (state.kind === "chrono") {
        const wheels = state.bootstrap?.wheels ?? ["lugged", "cylindrical"];
        return `
            <div class="settings-section">
                <div class="section-label"><strong>terrain model</strong><small>Chrono SCM configuration</small></div>
                <div class="field-grid">
                    <label class="field">
                        <span>wheel</span>
                        <select data-field="wheel">
                            ${wheels.map((wheel) => `<option value="${escapeHtml(wheel)}" ${wheel === state.wheel ? "selected" : ""}>${escapeHtml(wheel)}</option>`).join("")}
                        </select>
                    </label>
                    <label class="field range-field">
                        <span>interface scale <output>${state.uiScale.toFixed(1)}x</output></span>
                        <input type="range" min="0.8" max="2.4" step="0.1" value="${state.uiScale}" data-field="uiScale" />
                    </label>
                </div>
            </div>
            <button class="disclosure" type="button" data-action="toggle-advanced" aria-expanded="${state.advancedOpen}">
                <span>advanced + bridge</span><i>${state.advancedOpen ? "−" : "+"}</i>
            </button>
            ${state.advancedOpen ? advancedView(state) : ""}`;
    }

    return `
        <div class="settings-section build-section">
            <div class="section-label"><strong>workspace</strong><small>build behavior before launch</small></div>
            <div class="mini-segment" role="group" aria-label="Build behavior">
                <button type="button" data-action="set-build" data-value="fresh" class="${state.buildMode === "fresh" ? "is-active" : ""}">clean build</button>
                <button type="button" data-action="set-build" data-value="no-build" class="${state.buildMode === "no-build" ? "is-active" : ""}">reuse build</button>
                <button type="button" data-action="set-build" data-value="build-only" class="${state.buildMode === "build-only" ? "is-active" : ""}">build only</button>
            </div>
        </div>
        <div class="settings-section">
            <div class="section-label"><strong>processes</strong><small>only options for this target are shown</small></div>
            <div class="toggle-grid">${gazeboToggles(state)}</div>
        </div>
        <button class="disclosure" type="button" data-action="toggle-advanced" aria-expanded="${state.advancedOpen}">
            <span>advanced + bridge</span><i>${state.advancedOpen ? "−" : "+"}</i>
        </button>
        ${state.advancedOpen ? advancedView(state) : ""}`;
};
