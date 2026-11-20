import { spawn, type ChildProcess } from "node:child_process";
import { createInterface } from "node:readline";
import type { ExecutionMode, ProcessOutput, RuntimeState } from "../types";
import { containerName, idleState, probe, type Stage } from "./system";

interface ProcessEvents {
    output(output: ProcessOutput): void;
    runtime(runtime: RuntimeState): void;
}

export class ProcessManager {
    private state: RuntimeState = idleState();
    private activeProcess: ChildProcess | null = null;
    private readonly auxiliaryProcesses = new Set<ChildProcess>();
    private runToken = 0;
    private activeMode: ExecutionMode | null = null;

    constructor(private readonly events: ProcessEvents) {}

    get runtime(): RuntimeState {
        return this.state;
    }

    initialize(stages: Stage[], label: string): RuntimeState {
        this.assertIdle();
        this.activeMode = "docker";
        this.runSequence(stages, label);
        return this.state;
    }

    launch(stage: Stage, bridge: Stage | null, mode: ExecutionMode, label: string): RuntimeState {
        this.assertIdle();
        this.activeMode = mode;
        const token = ++this.runToken;

        if (bridge) this.startAuxiliary(bridge);
        this.setRuntime({ active: true, phase: "running", label, startedAt: Date.now() });
        const child = this.spawnStage(stage);
        this.activeProcess = child;
        child.once("error", (error) => this.finishRun(token, 1, error.message));
        child.once("close", (code) => this.finishRun(token, code ?? 1));
        return this.state;
    }

    async stop(): Promise<RuntimeState> {
        if (!this.state.active) return this.state;
        this.setRuntime({ ...this.state, phase: "stopping" });
        const processes = [this.activeProcess, ...this.auxiliaryProcesses].filter(
            (child): child is ChildProcess => child !== null
        );
        await Promise.all(processes.map((child) => this.killProcess(child)));

        if (this.activeMode === "docker") {
            await probe("docker", [
                "exec",
                containerName,
                "bash",
                "-lc",
                "pkill -INT -f '[r]os2 launch|[g]z sim|[v]sg_terrain' || true"
            ]);
        }
        this.activeProcess = null;
        this.auxiliaryProcesses.clear();
        this.activeMode = null;
        ++this.runToken;
        this.setRuntime(idleState());
        return this.state;
    }

    private startAuxiliary(stage: Stage): void {
        const child = this.spawnStage(stage);
        this.auxiliaryProcesses.add(child);
        child.once("close", (code) => {
            this.auxiliaryProcesses.delete(child);
            this.emitOutput(
                "bridge",
                code === 0 ? "system" : "stderr",
                `Bridge stopped with exit code ${code ?? "unknown"}.`
            );
        });
    }

    private runSequence(stages: Stage[], label: string): void {
        const token = ++this.runToken;
        this.setRuntime({ active: true, phase: "preparing", label, startedAt: Date.now() });
        const next = (index: number) => {
            if (token !== this.runToken) return;
            if (index >= stages.length) {
                this.activeProcess = null;
                this.activeMode = null;
                this.emitOutput("system", "system", "Environment is ready.");
                this.setRuntime(idleState());
                return;
            }
            const stage = stages[index];
            this.emitOutput(stage.source, "system", `Starting ${stage.source}...`);
            const child = this.spawnStage(stage);
            this.activeProcess = child;
            child.once("error", (error) => this.finishRun(token, 1, error.message));
            child.once("close", (code) => {
                if (token !== this.runToken) return;
                if (code === 0) next(index + 1);
                else this.finishRun(token, code ?? 1);
            });
        };
        next(0);
    }

    private spawnStage(stage: Stage): ChildProcess {
        const child = spawn(stage.command, stage.args, {
            cwd: stage.cwd,
            env: stage.env ?? process.env,
            detached: process.platform !== "win32",
            windowsHide: true,
            stdio: ["ignore", "pipe", "pipe"]
        });
        this.pipeOutput(child, stage.source, "stdout");
        this.pipeOutput(child, stage.source, "stderr");
        return child;
    }

    private pipeOutput(child: ChildProcess, source: string, stream: "stdout" | "stderr"): void {
        const readable = child[stream];
        if (!readable) return;
        createInterface({ input: readable }).on("line", (line) =>
            this.emitOutput(source, stream, line)
        );
    }

    private finishRun(token: number, code: number, error?: string): void {
        if (token !== this.runToken) return;
        for (const child of this.auxiliaryProcesses) void this.killProcess(child);
        this.auxiliaryProcesses.clear();
        this.activeProcess = null;
        this.activeMode = null;
        this.emitOutput(
            "system",
            code === 0 ? "system" : "stderr",
            error ?? (code === 0 ? "Process finished." : `Process exited with code ${code}.`)
        );
        this.setRuntime(code === 0 ? idleState() : { ...idleState(), phase: "failed" });
    }

    private async killProcess(child: ChildProcess): Promise<void> {
        if (!child.pid || child.killed) return;
        if (process.platform === "win32") {
            await probe("taskkill", ["/pid", String(child.pid), "/T", "/F"]);
            return;
        }
        try {
            process.kill(-child.pid, "SIGINT");
        } catch {
            child.kill("SIGINT");
        }
    }

    private assertIdle(): void {
        if (this.state.active) {
            throw new Error("Stop the current operation before starting another one.");
        }
    }

    private setRuntime(runtime: RuntimeState): void {
        this.state = runtime;
        this.events.runtime(runtime);
    }

    private emitOutput(source: string, stream: ProcessOutput["stream"], line: string): void {
        this.events.output({ source, stream, line, timestamp: Date.now() });
    }
}
