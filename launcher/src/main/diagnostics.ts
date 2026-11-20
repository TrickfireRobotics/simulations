import { existsSync } from "node:fs";
import path from "node:path";
import type { Diagnostic, ExecutionMode } from "../types";
import { containerName, probe } from "./system";

const commandCheck = async (
    id: string,
    label: string,
    command: string,
    args: string[]
): Promise<Diagnostic> => {
    const result = await probe(command, args);
    return {
        id,
        label,
        status: result.ok ? "pass" : "fail",
        detail: result.ok
            ? result.output || "Available."
            : result.output || `${command} was not found.`
    };
};

const hostDisplayCheck = (): Diagnostic => {
    if (process.platform === "linux") {
        const display = process.env.WAYLAND_DISPLAY || process.env.DISPLAY;
        return {
            id: "display",
            label: "Host display",
            status: display ? "pass" : "warn",
            detail: display
                ? `Display session detected (${display}).`
                : "No desktop display detected. The VNC fallback is still available."
        };
    }
    if (process.platform === "darwin") {
        return {
            id: "display",
            label: "Display forwarding",
            status: "warn",
            detail: "XQuartz must be running with network clients enabled. Initialization tests the container side."
        };
    }
    return {
        id: "display",
        label: "Display forwarding",
        status: "warn",
        detail: "VcXsrv or X410 is needed for desktop windows. The VNC fallback works without either."
    };
};

const nativeDiagnostics = async (repository: string): Promise<Diagnostic[]> => {
    const checks: Diagnostic[] = [];
    if (process.platform === "win32") {
        checks.push({
            id: "platform",
            label: "Native support",
            status: "fail",
            detail: "Native mode is not available on Windows. Use Docker mode."
        });
    }
    checks.push(await commandCheck("pixi", "Pixi", "pixi", ["--version"]));
    const installed = existsSync(path.join(repository, ".pixi"));
    checks.push({
        id: "pixi-environment",
        label: "Pixi environment",
        status: installed ? "pass" : "warn",
        detail: installed
            ? "Dependencies are installed."
            : "Not installed yet. Run pixi install before launching."
    });
    checks.push(hostDisplayCheck());
    return checks;
};

const dockerDiagnostics = async (repository: string): Promise<Diagnostic[]> => {
    const checks: Diagnostic[] = [
        await commandCheck("docker", "Docker command", "docker", ["--version"]),
        await commandCheck("docker-daemon", "Docker engine", "docker", [
            "info",
            "--format",
            "{{.ServerVersion}}"
        ]),
        await commandCheck("compose", "Docker Compose", "docker", ["compose", "version", "--short"])
    ];
    const envPath = path.join(repository, "docker", ".env");
    checks.push({
        id: "host-config",
        label: "Host configuration",
        status: existsSync(envPath) ? "pass" : "warn",
        detail: existsSync(envPath)
            ? "docker/.env is ready."
            : "Initialize Docker to detect display, ports, and GPU support."
    });

    const image = await probe("docker", [
        "image",
        "inspect",
        "simulations:latest",
        "--format",
        "{{.Id}}"
    ]);
    checks.push({
        id: "image",
        label: "Simulation image",
        status: image.ok ? "pass" : "warn",
        detail: image.ok
            ? "The simulation image is available."
            : "Initialization will build the image."
    });

    const container = await probe("docker", [
        "inspect",
        containerName,
        "--format",
        "{{.State.Status}}"
    ]);
    const running = container.ok && container.output === "running";
    checks.push({
        id: "container",
        label: "Simulation service",
        status: running ? "pass" : "warn",
        detail: running
            ? "Container is running."
            : "Initialize Docker to create and start the container."
    });

    if (!running) {
        checks.push({
            id: "display",
            label: "Container display",
            status: "warn",
            detail: "Initialize Docker before testing display forwarding."
        });
        return checks;
    }

    const display = await probe("docker", [
        "exec",
        containerName,
        "bash",
        "-lc",
        'xdpyinfo -display "$DISPLAY" >/dev/null'
    ]);
    checks.push({
        id: "display",
        label: "Container display",
        status: display.ok ? "pass" : "warn",
        detail: display.ok
            ? "The container can open graphical windows on its configured display."
            : "Display forwarding did not answer. Reinitialize, or use the fallback display."
    });
    return checks;
};

export const runDiagnostics = async (
    mode: ExecutionMode,
    repository: string
): Promise<Diagnostic[]> => {
    const repositoryCheck: Diagnostic = {
        id: "repository",
        label: "Simulation files",
        status: "pass",
        detail: repository
    };
    const checks =
        mode === "native"
            ? await nativeDiagnostics(repository)
            : await dockerDiagnostics(repository);
    return [repositoryCheck, ...checks];
};
