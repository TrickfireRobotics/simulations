export type ExecutionMode = "docker" | "native";
export type SimulationKind = "gazebo" | "chrono";
export type DiagnosticStatus = "pass" | "warn" | "fail" | "running";

export interface RobotTarget {
    name: string;
    label: string;
    kind: "rover" | "arm" | "drone" | "component";
    description: string;
}

export interface BootstrapData {
    repositoryPath: string | null;
    repositoryValid: boolean;
    platform: NodeJS.Platform;
    targets: RobotTarget[];
    wheels: string[];
    runtime: RuntimeState;
}

export interface Diagnostic {
    id: string;
    label: string;
    status: DiagnosticStatus;
    detail: string;
}

export interface RuntimeState {
    active: boolean;
    phase: "idle" | "preparing" | "running" | "stopping" | "failed";
    label: string | null;
    startedAt: number | null;
}

export interface GazeboOptions {
    target: string;
    buildMode: "fresh" | "no-build" | "build-only";
    controller: boolean;
    localQgc: boolean;
    ttt: boolean;
    arTags: boolean;
    followRover: boolean;
    jointGui: boolean;
    rviz: boolean;
    gazeboGui: boolean;
    rosbridgePort: number;
    extraArgs: string[];
}

export interface ChronoOptions {
    wheel: string;
    uiScale: number;
}

export interface BridgeOptions {
    enabled: boolean;
    urcRepositoryPath: string;
    maxPointCloudHz: number | null;
}

export interface LaunchRequest {
    mode: ExecutionMode;
    kind: SimulationKind;
    gazebo?: GazeboOptions;
    chrono?: ChronoOptions;
    bridge: BridgeOptions;
}

export interface ProcessOutput {
    source: string;
    stream: "stdout" | "stderr" | "system";
    line: string;
    timestamp: number;
}

export interface LauncherApi {
    getBootstrap(): Promise<BootstrapData>;
    selectRepository(): Promise<BootstrapData>;
    selectUrcRepository(): Promise<string | null>;
    runDiagnostics(mode: ExecutionMode): Promise<Diagnostic[]>;
    initializeDocker(): Promise<RuntimeState>;
    launch(request: LaunchRequest): Promise<RuntimeState>;
    stop(): Promise<RuntimeState>;
    getRuntime(): Promise<RuntimeState>;
    openNoVnc(): Promise<void>;
    onOutput(listener: (output: ProcessOutput) => void): () => void;
    onRuntime(listener: (state: RuntimeState) => void): () => void;
}
