import type { LauncherApi } from "./types";

declare global {
    interface Window {
        launcher: LauncherApi;
    }

    const MAIN_WINDOW_VITE_DEV_SERVER_URL: string | undefined;
    const MAIN_WINDOW_VITE_NAME: string;
}

export {};
