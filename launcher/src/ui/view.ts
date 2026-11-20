import { diagnosticsView, logsView, settingsView } from "./fragments";
import { escapeHtml } from "./html";
import { statusSummary, type UiState } from "./state";

const repositoryName = (path: string | null | undefined): string =>
    path?.split(/[\\/]/).filter(Boolean).at(-1) ?? "choose simulations folder";

const commandStrip = (state: UiState): string => {
    const targets = state.bootstrap?.targets ?? [];
    const summary = statusSummary(state);
    const busy = state.runtime.active;
    const canLaunch = Boolean(state.bootstrap?.repositoryValid) && !busy;

    return `
        <section class="command-strip tone-${summary.tone}">
            <div class="command-meta">
                <span class="status-label"><i></i>${summary.label}</span>
                <span>${escapeHtml(state.bootstrap?.platform ?? "host")}</span>
            </div>
            <div class="command-row">
                <div class="command-path">
                    <div class="command-choice">
                        <small>runtime</small>
                        <div class="command-segment" role="group" aria-label="Runtime">
                            <button type="button" data-action="set-mode" data-value="docker" class="${state.mode === "docker" ? "is-active" : ""}">docker</button>
                            <button type="button" data-action="set-mode" data-value="native" class="${state.mode === "native" ? "is-active" : ""}">native</button>
                        </div>
                    </div>
                    <span class="path-slash">/</span>
                    <div class="command-choice">
                        <small>simulation</small>
                        <div class="command-segment" role="group" aria-label="Simulation">
                            <button type="button" data-action="set-kind" data-value="gazebo" class="${state.kind === "gazebo" ? "is-active" : ""}">gazebo</button>
                            <button type="button" data-action="set-kind" data-value="chrono" class="${state.kind === "chrono" ? "is-active" : ""}" ${state.mode === "native" ? "disabled" : ""}>chrono</button>
                        </div>
                    </div>
                    <span class="path-slash">/</span>
                    <label class="command-choice target-choice">
                        <small>${state.kind === "gazebo" ? "target" : "wheel"}</small>
                        ${
                            state.kind === "gazebo"
                                ? `<select data-field="target">${targets.map((target) => `<option value="${escapeHtml(target.name)}" ${target.name === state.target ? "selected" : ""}>${escapeHtml(target.label)}</option>`).join("")}</select>`
                                : `<strong>${escapeHtml(state.wheel)}</strong>`
                        }
                    </label>
                </div>
                ${
                    busy
                        ? `<button class="run-button stop" type="button" data-action="stop"><span>stop</span><small>${escapeHtml(state.runtime.label ?? "current process")}</small></button>`
                        : `<button class="run-button" type="button" data-action="launch" ${canLaunch ? "" : "disabled"}><span>run simulation</span><b>↗</b></button>`
                }
            </div>
            <div class="command-rule" aria-hidden="true"></div>
        </section>`;
};

const systemPanel = (state: UiState): string => {
    const summary = statusSummary(state);
    return `
        <aside class="panel system-panel">
            <header class="panel-header">
                <div><h2>host</h2><span class="panel-status tone-${summary.tone}"><i></i>${summary.label}</span></div>
                <button class="quiet-button" type="button" data-action="check" ${state.checking || state.runtime.active ? "disabled" : ""}>${state.checking ? "checking..." : "check"}</button>
            </header>
            <ul class="diagnostics">${diagnosticsView(state)}</ul>
            <div class="system-actions">
                ${state.mode === "docker" ? `<button type="button" data-action="initialize" ${state.runtime.active ? "disabled" : ""}>initialize docker</button>` : ""}
                ${state.mode === "docker" ? `<button type="button" data-action="open-vnc">open fallback display ↗</button>` : ""}
            </div>
        </aside>`;
};

const onboarding = (state: UiState): string => {
    if (!state.bootstrap || state.bootstrap.repositoryValid) return "";
    return `
        <div class="modal-backdrop">
            <section class="modal" role="dialog" aria-modal="true" aria-labelledby="setup-title">
                <span class="modal-mark">TF / SIM</span>
                <h2 id="setup-title">locate simulations</h2>
                <p>Choose the TrickFire simulations repository. The launcher stores this path and manages the environment from the host.</p>
                <button type="button" data-action="choose-repository">choose folder</button>
            </section>
        </div>`;
};

export const renderApp = (state: UiState): string => `
    <div class="app-shell">
        <header class="app-header">
            <div class="identity"><span>TF</span><i>/</i><strong>SIM</strong></div>
            <button class="repository-button" type="button" data-action="choose-repository" title="${escapeHtml(state.bootstrap?.repositoryPath ?? "Choose repository")}">
                <i class="${state.bootstrap?.repositoryValid ? "is-valid" : ""}"></i>
                <span><small>repository</small><strong>${escapeHtml(repositoryName(state.bootstrap?.repositoryPath))}</strong></span>
                <b>change</b>
            </button>
        </header>
        <main class="workspace">
            ${state.error ? `<div class="error-banner" role="alert"><span>${escapeHtml(state.error)}</span><button type="button" data-action="dismiss-error">dismiss</button></div>` : ""}
            ${commandStrip(state)}
            <div class="work-grid">
                <section class="panel settings-panel">
                    <header class="panel-header"><div><h1>run settings</h1><p>${state.kind === "gazebo" ? escapeHtml(state.bootstrap?.targets.find((target) => target.name === state.target)?.description ?? "Gazebo launch configuration") : "terrain physics configuration"}</p></div></header>
                    ${settingsView(state)}
                </section>
                ${systemPanel(state)}
            </div>
            <section class="panel output-panel">
                <header class="output-header">
                    <div><span class="terminal-prompt">›_</span><h2>output</h2>${state.runtime.active ? `<span class="runtime-label">${escapeHtml(state.runtime.label ?? "running")}</span>` : ""}</div>
                    <button class="quiet-button" type="button" data-action="clear-logs">clear</button>
                </header>
                <div class="log-view" id="log-view">${logsView(state)}</div>
            </section>
        </main>
    </div>
    ${onboarding(state)}`;
