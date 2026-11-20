import { execFile } from "node:child_process";
import { promisify } from "node:util";
import type { RuntimeState } from "../types";

const execFileAsync = promisify(execFile);

export const containerName = "simulations";
export const containerRepository = "/home/trickfire/simulations";

export interface Stage {
    source: string;
    command: string;
    args: string[];
    cwd: string;
    env?: NodeJS.ProcessEnv;
}

export interface ProbeResult {
    ok: boolean;
    output: string;
}

export const idleState = (): RuntimeState => ({
    active: false,
    phase: "idle",
    label: null,
    startedAt: null
});

export const shellQuote = (value: string): string => `'${value.replaceAll("'", "'\"'\"'")}'`;

export const probe = async (command: string, args: string[]): Promise<ProbeResult> => {
    try {
        const { stdout, stderr } = await execFileAsync(command, args, {
            timeout: 8000,
            windowsHide: true
        });
        return { ok: true, output: String(stdout || stderr).trim() };
    } catch (error) {
        const failure = error as NodeJS.ErrnoException & { stderr?: string; stdout?: string };
        return {
            ok: false,
            output: String(failure.stderr || failure.stdout || failure.message || "").trim()
        };
    }
};
