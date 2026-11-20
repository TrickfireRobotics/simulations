import type {
    BootstrapData,
    Diagnostic,
    ExecutionMode,
    LauncherApi,
    ProcessOutput,
    RuntimeState
} from "./types";

const targets: BootstrapData["targets"] = [
    {
        name: "viator",
        label: "Viator",
        kind: "rover",
        description: "Complete rover, arm, cameras, GPS, and drivebase"
    },
    {
        name: "viator_chassis",
        label: "Viator Chassis",
        kind: "component",
        description: "Focused subsystem and mechanical testing"
    },
    {
        name: "arm",
        label: "Arm",
        kind: "arm",
        description: "Arm controls and collision testing in isolation"
    },
    {
        name: "drone",
        label: "Drone",
        kind: "drone",
        description: "ArduPilot SITL with QGroundControl"
    }
];

const outputListeners = new Set<(output: ProcessOutput) => void>();
const runtimeListeners = new Set<(runtime: RuntimeState) => void>();
let runtime: RuntimeState = {
    active: false,
    phase: "idle",
    label: null,
    startedAt: null
};

const bootstrap = (): BootstrapData => ({
    repositoryPath: "/home/trickfire/simulations",
    repositoryValid: true,
    platform: "linux",
    targets,
    wheels: ["lugged", "cylindrical", "apollo"],
    runtime
});

const delay = (milliseconds: number) => new Promise((resolve) => setTimeout(resolve, milliseconds));

const emitRuntime = (next: RuntimeState) => {
    runtime = next;
    for (const listener of runtimeListeners) listener(runtime);
};

const emitOutput = (line: string, source = "preview") => {
    const output: ProcessOutput = {
        source,
        stream: "system",
        line,
        timestamp: Date.now()
    };
    for (const listener of outputListeners) listener(output);
};

const diagnostics = (mode: ExecutionMode): Diagnostic[] => [
    {
        id: "repository",
        label: "Simulation files",
        status: "pass",
        detail: "/home/trickfire/simulations"
    },
    {
        id: "runtime",
        label: mode === "docker" ? "Docker engine" : "Pixi",
        status: "pass",
        detail: mode === "docker" ? "Docker 29.1 is available." : "Pixi is available."
    },
    {
        id: "container",
        label: "Simulation service",
        status: "pass",
        detail: "Preview data: service is running."
    },
    {
        id: "display",
        label: "Container display",
        status: "pass",
        detail: "Preview data: graphical display is reachable."
    }
];

export const createPreviewApi = (): LauncherApi => ({
    getBootstrap: async () => bootstrap(),
    selectRepository: async () => bootstrap(),
    selectUrcRepository: async () => "/home/trickfire/trickfire-urc",
    runDiagnostics: async (mode) => {
        await delay(350);
        return diagnostics(mode);
    },
    initializeDocker: async () => {
        const next: RuntimeState = {
            active: true,
            phase: "preparing",
            label: "Initializing simulation environment",
            startedAt: Date.now()
        };
        emitRuntime(next);
        emitOutput("Preview mode: environment initialization started.");
        return next;
    },
    launch: async (request) => {
        const next: RuntimeState = {
            active: true,
            phase: "running",
            label:
                request.kind === "gazebo"
                    ? `Gazebo · ${request.gazebo?.target}`
                    : `Chrono · ${request.chrono?.wheel}`,
            startedAt: Date.now()
        };
        emitRuntime(next);
        emitOutput("Preview mode: simulation launch requested.");
        return next;
    },
    stop: async () => {
        const next: RuntimeState = {
            active: false,
            phase: "idle",
            label: null,
            startedAt: null
        };
        emitOutput("Preview mode: process stopped.");
        emitRuntime(next);
        return next;
    },
    getRuntime: async () => runtime,
    openNoVnc: async () => emitOutput("Preview mode: noVNC would open on the host."),
    onOutput: (listener) => {
        outputListeners.add(listener);
        return () => outputListeners.delete(listener);
    },
    onRuntime: (listener) => {
        runtimeListeners.add(listener);
        return () => runtimeListeners.delete(listener);
    }
});
