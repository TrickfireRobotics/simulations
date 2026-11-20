import type { LauncherApi, ProcessOutput } from "../types";
import { renderLogEntry } from "./html";
import {
    buildLaunchRequest,
    createInitialState,
    reconcileBootstrap,
    selectTarget,
    type BuildMode,
    type UiState
} from "./state";
import { renderApp } from "./view";

export class LauncherUi {
    private readonly state: UiState = createInitialState();

    constructor(
        private readonly root: HTMLElement,
        private readonly api: LauncherApi
    ) {}

    async start(): Promise<void> {
        this.root.addEventListener("click", (event) => void this.handleClick(event));
        this.root.addEventListener("change", (event) => this.handleField(event));
        this.root.addEventListener("input", (event) => this.handleInput(event));
        this.api.onOutput((output) => this.onOutput(output));
        this.api.onRuntime((runtime) => {
            this.state.runtime = runtime;
            this.render();
        });
        await this.boot();
    }

    private render(): void {
        this.root.innerHTML = renderApp(this.state);
        requestAnimationFrame(() => this.scrollLogs());
    }

    private async boot(): Promise<void> {
        try {
            const bootstrap = await this.api.getBootstrap();
            reconcileBootstrap(this.state, bootstrap);
            this.state.runtime = bootstrap.runtime;
            this.render();
            if (bootstrap.repositoryValid) await this.checkSystem();
        } catch (error) {
            this.captureError(error);
        }
        this.render();
    }

    private async handleClick(event: Event): Promise<void> {
        const button = (event.target as HTMLElement).closest<HTMLButtonElement>(
            "button[data-action]"
        );
        if (!button || button.disabled) return;
        const value = button.dataset.value ?? "";

        switch (button.dataset.action) {
            case "choose-repository":
                await this.runAction(async () => {
                    reconcileBootstrap(this.state, await this.api.selectRepository());
                    if (this.state.bootstrap?.repositoryValid) await this.checkSystem();
                });
                break;
            case "dismiss-error":
                this.state.error = null;
                this.render();
                break;
            case "set-mode":
                this.state.mode = value === "native" ? "native" : "docker";
                if (this.state.mode === "native" && this.state.kind === "chrono") {
                    this.state.kind = "gazebo";
                }
                this.state.diagnostics = [];
                this.render();
                break;
            case "set-kind":
                this.state.kind = value === "chrono" ? "chrono" : "gazebo";
                this.render();
                break;
            case "set-build":
                this.state.buildMode = value as BuildMode;
                this.render();
                break;
            case "toggle-advanced":
                this.state.advancedOpen = !this.state.advancedOpen;
                this.render();
                break;
            case "choose-urc":
                await this.runAction(async () => {
                    const selected = await this.api.selectUrcRepository();
                    if (selected) this.state.urcRepositoryPath = selected;
                });
                break;
            case "check":
                await this.runAction(() => this.checkSystem());
                break;
            case "initialize":
                await this.runAction(async () => {
                    this.state.logs = [];
                    this.state.runtime = await this.api.initializeDocker();
                });
                break;
            case "open-vnc":
                await this.runAction(() => this.api.openNoVnc());
                break;
            case "clear-logs":
                this.state.logs = [];
                this.render();
                break;
            case "launch":
                await this.runAction(async () => {
                    this.state.logs = [];
                    this.state.runtime = await this.api.launch(buildLaunchRequest(this.state));
                });
                break;
            case "stop":
                await this.runAction(async () => {
                    this.state.runtime = await this.api.stop();
                });
                break;
        }
    }

    private handleField(event: Event): void {
        const input = (event.target as HTMLElement).closest<HTMLInputElement | HTMLSelectElement>(
            "[data-field]"
        );
        if (!input) return;
        const field = input.dataset.field;

        if (input instanceof HTMLInputElement && input.type === "checkbox") {
            const booleanFields = {
                gazeboGui: "gazeboGui",
                rviz: "rviz",
                controller: "controller",
                jointGui: "jointGui",
                ttt: "ttt",
                arTags: "arTags",
                followRover: "followRover",
                localQgc: "localQgc",
                bridgeEnabled: "bridgeEnabled"
            } as const;
            const key = booleanFields[field as keyof typeof booleanFields];
            if (key) this.state[key] = input.checked;
            this.render();
            return;
        }

        if (field === "target") selectTarget(this.state, input.value);
        if (field === "wheel") this.state.wheel = input.value;
        if (field === "rosbridgePort") this.state.rosbridgePort = Number(input.value);
        this.render();
    }

    private handleInput(event: Event): void {
        const input = (event.target as HTMLElement).closest<HTMLInputElement>("input[data-field]");
        if (!input) return;
        if (input.dataset.field === "uiScale") {
            this.state.uiScale = Number(input.value);
            input.closest("label")?.querySelector("output")?.replaceChildren(`${input.value}x`);
        }
        if (input.dataset.field === "extraArgs") this.state.extraArgs = input.value;
        if (input.dataset.field === "maxPointCloudHz") this.state.maxPointCloudHz = input.value;
    }

    private async checkSystem(): Promise<void> {
        this.state.checking = true;
        this.state.diagnostics = [];
        this.render();
        try {
            this.state.diagnostics = await this.api.runDiagnostics(this.state.mode);
        } finally {
            this.state.checking = false;
        }
    }

    private async runAction(action: () => Promise<void>): Promise<void> {
        this.state.error = null;
        try {
            await action();
        } catch (error) {
            this.captureError(error);
        }
        this.render();
    }

    private captureError(error: unknown): void {
        this.state.error = error instanceof Error ? error.message : String(error);
    }

    private onOutput(output: ProcessOutput): void {
        this.state.logs.push(output);
        if (this.state.logs.length > 600) this.state.logs.splice(0, this.state.logs.length - 600);
        const logView = this.root.querySelector<HTMLElement>("#log-view");
        if (!logView) return;
        logView.querySelector(".log-empty")?.remove();
        logView.insertAdjacentHTML("beforeend", renderLogEntry(output));
        this.scrollLogs();
    }

    private scrollLogs(): void {
        const logView = this.root.querySelector<HTMLElement>("#log-view");
        if (logView) logView.scrollTop = logView.scrollHeight;
    }
}
