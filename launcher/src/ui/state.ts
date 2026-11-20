import type {
    BootstrapData,
    Diagnostic,
    ExecutionMode,
    LaunchRequest,
    ProcessOutput,
    RuntimeState,
    SimulationKind
} from "../types";

export type BuildMode = "fresh" | "no-build" | "build-only";
export type StatusTone = "neutral" | "pass" | "warn" | "fail" | "running";

export interface UiState {
    bootstrap: BootstrapData | null;
    mode: ExecutionMode;
    kind: SimulationKind;
    target: string;
    wheel: string;
    buildMode: BuildMode;
    controller: boolean;
    localQgc: boolean;
    ttt: boolean;
    arTags: boolean;
    followRover: boolean;
    jointGui: boolean;
    rviz: boolean;
    gazeboGui: boolean;
    rosbridgePort: number;
    extraArgs: string;
    uiScale: number;
    bridgeEnabled: boolean;
    urcRepositoryPath: string;
    maxPointCloudHz: string;
    diagnostics: Diagnostic[];
    checking: boolean;
    runtime: RuntimeState;
    logs: ProcessOutput[];
    error: string | null;
    advancedOpen: boolean;
}

export const createInitialState = (): UiState => ({
    bootstrap: null,
    mode: "docker",
    kind: "gazebo",
    target: "viator",
    wheel: "lugged",
    buildMode: "fresh",
    controller: false,
    localQgc: false,
    ttt: false,
    arTags: false,
    followRover: true,
    jointGui: false,
    rviz: true,
    gazeboGui: true,
    rosbridgePort: 9090,
    extraArgs: "",
    uiScale: 1,
    bridgeEnabled: false,
    urcRepositoryPath: "",
    maxPointCloudHz: "2",
    diagnostics: [],
    checking: false,
    runtime: { active: false, phase: "idle", label: null, startedAt: null },
    logs: [],
    error: null,
    advancedOpen: false
});

export const statusSummary = (state: UiState): { label: string; tone: StatusTone } => {
    if (state.checking) return { label: "checking host", tone: "running" };
    if (!state.diagnostics.length) return { label: "host unchecked", tone: "neutral" };
    if (state.diagnostics.some((item) => item.status === "fail")) {
        return { label: "action needed", tone: "fail" };
    }
    if (state.diagnostics.some((item) => item.status === "warn")) {
        return { label: "ready with notes", tone: "warn" };
    }
    return { label: "ready", tone: "pass" };
};

export const selectTarget = (state: UiState, target: string): void => {
    state.target = target;
    if (target !== "viator") {
        state.ttt = false;
        state.arTags = false;
        state.followRover = true;
    }
    if (target !== "viator" && target !== "arm") state.jointGui = false;
    if (target !== "drone") state.localQgc = false;
    if (target === "drone") {
        state.controller = false;
        state.rviz = true;
    }
};

export const reconcileBootstrap = (state: UiState, bootstrap: BootstrapData): void => {
    state.bootstrap = bootstrap;
    const firstTarget = bootstrap.targets[0]?.name;
    if (!bootstrap.targets.some((target) => target.name === state.target) && firstTarget) {
        selectTarget(state, firstTarget);
    }
};

export const buildLaunchRequest = (state: UiState): LaunchRequest => ({
    mode: state.mode,
    kind: state.kind,
    gazebo:
        state.kind === "gazebo"
            ? {
                  target: state.target,
                  buildMode: state.buildMode,
                  controller: state.controller,
                  localQgc: state.localQgc,
                  ttt: state.ttt,
                  arTags: state.arTags,
                  followRover: state.followRover,
                  jointGui: state.jointGui,
                  rviz: state.rviz,
                  gazeboGui: state.gazeboGui,
                  rosbridgePort: state.rosbridgePort,
                  extraArgs: state.extraArgs.split(/\s+/).filter(Boolean)
              }
            : undefined,
    chrono:
        state.kind === "chrono"
            ? {
                  wheel: state.wheel,
                  uiScale: state.uiScale
              }
            : undefined,
    bridge: {
        enabled: state.bridgeEnabled,
        urcRepositoryPath: state.urcRepositoryPath,
        maxPointCloudHz: state.maxPointCloudHz ? Number(state.maxPointCloudHz) : null
    }
});
