import { app } from "electron";
import { existsSync, readFileSync } from "node:fs";
import { readFile, readdir, writeFile } from "node:fs/promises";
import path from "node:path";
import type { RobotTarget } from "../types";

interface Settings {
    repositoryPath: string | null;
}

const titleCase = (value: string): string =>
    value
        .split("_")
        .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
        .join(" ");

export class RepositoryStore {
    private settings: Settings = { repositoryPath: this.findInitialRepository() };

    async load(): Promise<void> {
        try {
            const saved = JSON.parse(await readFile(this.settingsPath(), "utf8")) as Settings;
            if (saved.repositoryPath && this.isValid(saved.repositoryPath)) this.settings = saved;
        } catch {
            await this.save();
        }
    }

    get path(): string | null {
        return this.settings.repositoryPath;
    }

    get valid(): boolean {
        return this.path !== null && this.isValid(this.path);
    }

    require(): string {
        if (!this.path || !this.isValid(this.path)) {
            throw new Error("Choose the TrickFire simulations folder first.");
        }
        return this.path;
    }

    isValid(candidate: string): boolean {
        return (
            existsSync(path.join(candidate, "robots.json")) &&
            existsSync(path.join(candidate, "docker", "docker-compose.yml")) &&
            existsSync(path.join(candidate, "pyproject.toml"))
        );
    }

    async set(selected: string): Promise<void> {
        if (!this.isValid(selected)) {
            throw new Error("That folder is not the TrickFire simulations repository.");
        }
        this.settings.repositoryPath = selected;
        await this.save();
    }

    targetNames(): string[] {
        try {
            const entries = JSON.parse(
                readFileSync(path.join(this.require(), "robots.json"), "utf8")
            ) as Array<{ name: string }>;
            return entries.map((entry) => entry.name);
        } catch {
            return [];
        }
    }

    async targets(): Promise<RobotTarget[]> {
        const entries = JSON.parse(
            await readFile(path.join(this.require(), "robots.json"), "utf8")
        ) as Array<{ name: string; type?: string }>;
        const descriptions: Record<RobotTarget["kind"], string> = {
            rover: "Complete rover, arm, cameras, GPS, and drivebase",
            arm: "Arm controls and collision testing in isolation",
            drone: "ArduPilot SITL with QGroundControl",
            component: "Focused subsystem and mechanical testing"
        };
        const targets = entries.map((entry) => {
            const kind: RobotTarget["kind"] =
                entry.type === "ardupilot"
                    ? "drone"
                    : entry.name === "arm"
                      ? "arm"
                      : entry.name === "viator"
                        ? "rover"
                        : "component";
            return {
                name: entry.name,
                label: titleCase(entry.name),
                kind,
                description: descriptions[kind]
            };
        });
        const priority = ["viator", "viator_chassis", "arm", "drone"];
        return targets.sort((a, b) => {
            const rank = (name: string) => {
                const index = priority.indexOf(name);
                return index < 0 ? priority.length : index;
            };
            return rank(a.name) - rank(b.name);
        });
    }

    async wheels(): Promise<string[]> {
        const wheels = ["lugged", "cylindrical"];
        try {
            const directory = path.join(this.require(), "chrono", "data", "models", "wheels");
            const files = await readdir(directory);
            wheels.push(
                ...files
                    .filter((file) => file.endsWith(".obj"))
                    .map((file) => path.parse(file).name)
                    .sort()
            );
        } catch {
            return wheels;
        }
        return [...new Set(wheels)];
    }

    private findInitialRepository(): string | null {
        const candidates = [
            process.env.SIM_REPOSITORY,
            process.cwd(),
            path.resolve(process.cwd(), ".."),
            path.resolve(__dirname, "../../..")
        ].filter((candidate): candidate is string => Boolean(candidate));
        return candidates.find((candidate) => this.isValid(candidate)) ?? null;
    }

    private settingsPath(): string {
        return path.join(app.getPath("userData"), "launcher.json");
    }

    private async save(): Promise<void> {
        await writeFile(this.settingsPath(), JSON.stringify(this.settings, null, 2), "utf8");
    }
}
