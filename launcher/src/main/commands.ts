import { existsSync } from "node:fs";
import path from "node:path";
import type { LaunchRequest } from "../types";
import type { RepositoryStore } from "./repository";
import { containerName, containerRepository, shellQuote, type Stage } from "./system";

const simulationArgs = (request: LaunchRequest, repository: RepositoryStore): string[] => {
    if (request.kind === "chrono") {
        if (!request.chrono) throw new Error("Chrono options are missing.");
        if (!/^[A-Za-z0-9_.-]+$/.test(request.chrono.wheel)) {
            throw new Error("Invalid wheel name.");
        }
        return ["sim", "chrono", "run", request.chrono.wheel];
    }

    if (!request.gazebo) throw new Error("Gazebo options are missing.");
    if (!repository.targetNames().includes(request.gazebo.target)) {
        throw new Error("Unknown Gazebo target.");
    }

    const options = request.gazebo;
    const args = ["sim", "gazebo", options.target];
    if (options.buildMode === "no-build") args.push("--no-build");
    if (options.buildMode === "build-only") args.push("--build-only");
    if (options.controller) args.push("--controller-ui");
    if (options.localQgc) args.push("--local-qgc");
    if (options.ttt) args.push("--ttt");
    if (options.arTags) args.push("--ar-tags");
    if (!options.followRover && options.target === "viator") args.push("--no-follow-rover");
    if (options.jointGui) args.push("--joint-gui");
    if (!options.rviz && options.target !== "drone") args.push("rviz:=false");
    if (!options.gazeboGui) args.push("gui:=false");
    if (options.rosbridgePort < 1 || options.rosbridgePort > 65535) {
        throw new Error("ROS bridge port must be between 1 and 65535.");
    }
    args.push(`rosbridge_port:=${options.rosbridgePort}`);
    for (const extra of options.extraArgs) {
        if (!/^[A-Za-z_][A-Za-z0-9_]*:=\S+$/.test(extra)) {
            throw new Error(`Invalid ROS launch argument: ${extra}`);
        }
        args.push(extra);
    }
    return args;
};

export const buildSimulationStage = (
    request: LaunchRequest,
    repository: RepositoryStore
): Stage => {
    const repositoryPath = repository.require();
    const args = simulationArgs(request, repository);
    const env = { ...process.env };
    if (request.kind === "chrono" && request.chrono) {
        env.TRICKFIRE_UI_SCALE = String(request.chrono.uiScale);
    }

    if (request.mode === "native") {
        if (request.kind === "chrono") throw new Error("Chrono is only available in Docker mode.");
        return {
            source: "simulation",
            command: "pixi",
            args: ["run", ...args],
            cwd: repositoryPath,
            env
        };
    }

    const innerEnv =
        request.kind === "chrono" && request.chrono
            ? `TRICKFIRE_UI_SCALE=${shellQuote(String(request.chrono.uiScale))} `
            : "";
    return {
        source: "simulation",
        command: "docker",
        args: [
            "exec",
            "-i",
            containerName,
            "bash",
            "-lc",
            `cd ${containerRepository} && ${innerEnv}${args.map(shellQuote).join(" ")}`
        ],
        cwd: repositoryPath
    };
};

export const buildBridgeStage = (request: LaunchRequest, repository: string): Stage | null => {
    if (!request.bridge.enabled) return null;
    const urcRepository = request.bridge.urcRepositoryPath;
    const script = path.join(urcRepository, "simulation", "sim-bridge.sh");
    if (!existsSync(script)) {
        throw new Error("Select a valid trickfire-urc repository for the bridge.");
    }
    const args = [script, request.mode === "docker" ? "docker" : "native"];
    if (request.bridge.maxPointCloudHz !== null) {
        args.push("--pub-max-frequency", `.*/point_cloud/.*=${request.bridge.maxPointCloudHz}`);
    }
    return {
        source: "bridge",
        command: process.platform === "win32" ? "bash.exe" : "bash",
        args,
        cwd: urcRepository || repository
    };
};
