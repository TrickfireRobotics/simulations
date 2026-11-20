import { dialog, shell, type BrowserWindow } from "electron";
import { existsSync } from "node:fs";
import { readFile } from "node:fs/promises";
import path from "node:path";
import type { BootstrapData, ExecutionMode, LaunchRequest, RuntimeState } from "../types";
import { buildBridgeStage, buildSimulationStage } from "./commands";
import { dockerInitializationStages, writeDockerEnvironment } from "./docker";
import { runDiagnostics } from "./diagnostics";
import { ProcessManager } from "./process-manager";
import { RepositoryStore } from "./repository";

export class LauncherController {
    private readonly repository = new RepositoryStore();
    private readonly processes: ProcessManager;

    constructor(private readonly window: BrowserWindow) {
        this.processes = new ProcessManager({
            output: (output) => this.window.webContents.send("launcher:output", output),
            runtime: (runtime) => this.window.webContents.send("launcher:runtime", runtime)
        });
    }

    loadSettings(): Promise<void> {
        return this.repository.load();
    }

    async getBootstrap(): Promise<BootstrapData> {
        return {
            repositoryPath: this.repository.path,
            repositoryValid: this.repository.valid,
            platform: process.platform,
            targets: this.repository.valid ? await this.repository.targets() : [],
            wheels: this.repository.valid
                ? await this.repository.wheels()
                : ["lugged", "cylindrical"],
            runtime: this.processes.runtime
        };
    }

    async selectRepository(): Promise<BootstrapData> {
        const result = await dialog.showOpenDialog(this.window, {
            title: "Choose the simulations folder",
            properties: ["openDirectory"]
        });
        if (!result.canceled && result.filePaths[0]) await this.repository.set(result.filePaths[0]);
        return this.getBootstrap();
    }

    async selectUrcRepository(): Promise<string | null> {
        const result = await dialog.showOpenDialog(this.window, {
            title: "Choose the trickfire-urc folder",
            properties: ["openDirectory"]
        });
        if (result.canceled || !result.filePaths[0]) return null;
        const selected = result.filePaths[0];
        if (!existsSync(path.join(selected, "simulation", "sim-bridge.sh"))) {
            throw new Error("That folder does not contain simulation/sim-bridge.sh.");
        }
        return selected;
    }

    runDiagnostics(mode: ExecutionMode) {
        return runDiagnostics(mode, this.repository.require());
    }

    async initializeDocker(): Promise<RuntimeState> {
        const repository = this.repository.require();
        await writeDockerEnvironment(repository);
        return this.processes.initialize(
            dockerInitializationStages(repository),
            "Initializing simulation environment"
        );
    }

    launch(request: LaunchRequest): RuntimeState {
        const repository = this.repository.require();
        const stage = buildSimulationStage(request, this.repository);
        const bridge = buildBridgeStage(request, repository);
        const label =
            request.kind === "gazebo"
                ? `Gazebo · ${request.gazebo?.target}`
                : `Chrono · ${request.chrono?.wheel}`;
        return this.processes.launch(stage, bridge, request.mode, label);
    }

    stop(): Promise<RuntimeState> {
        return this.processes.stop();
    }

    getRuntime(): RuntimeState {
        return this.processes.runtime;
    }

    async openNoVnc(): Promise<void> {
        let port = "6080";
        try {
            const envFile = await readFile(
                path.join(this.repository.require(), "docker", ".env"),
                "utf8"
            );
            const match = [...envFile.matchAll(/^NOVNC_PORT=(.+)$/gm)].at(-1);
            if (match?.[1]) port = match[1].trim();
        } catch {
            port = "6080";
        }
        await shell.openExternal(`http://localhost:${port}/vnc.html?autoconnect=true&resize=scale`);
    }
}
