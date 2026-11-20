import { app, BrowserWindow, dialog, ipcMain } from "electron";
import { writeFile } from "node:fs/promises";
import path from "node:path";
import type { ExecutionMode, LaunchRequest } from "./types";
import { LauncherController } from "./main/launcher-controller";

const isDevcontainer = process.env.TRICKFIRE_DEVCONTAINER === "1";
let controller: LauncherController | null = null;
let quitting = false;

const createWindow = async (): Promise<void> => {
    const window = new BrowserWindow({
        width: 1380,
        height: 900,
        minWidth: 820,
        minHeight: 700,
        backgroundColor: "#050505",
        title: "TrickFire Sim",
        webPreferences: {
            preload: path.join(__dirname, "preload.cjs"),
            contextIsolation: true,
            nodeIntegration: false,
            sandbox: true
        }
    });

    controller = new LauncherController(window);
    await controller.loadSettings();

    const screenshotPath = process.env.SIM_LAUNCHER_SCREENSHOT;
    if (screenshotPath) {
        window.webContents.once("did-finish-load", () => {
            setTimeout(async () => {
                const image = await window.webContents.capturePage();
                await writeFile(screenshotPath, image.toPNG());
            }, 3000);
        });
    }

    if (MAIN_WINDOW_VITE_DEV_SERVER_URL) {
        await window.loadURL(MAIN_WINDOW_VITE_DEV_SERVER_URL);
    } else {
        await window.loadFile(
            path.join(__dirname, `../renderer/${MAIN_WINDOW_VITE_NAME}/index.html`)
        );
    }
};

const registerIpc = (): void => {
    ipcMain.handle("launcher:get-bootstrap", () => controller?.getBootstrap());
    ipcMain.handle("launcher:select-repository", () => controller?.selectRepository());
    ipcMain.handle("launcher:select-urc-repository", () => controller?.selectUrcRepository());
    ipcMain.handle("launcher:diagnostics", (_event, mode: ExecutionMode) =>
        controller?.runDiagnostics(mode)
    );
    ipcMain.handle("launcher:initialize-docker", () => controller?.initializeDocker());
    ipcMain.handle("launcher:launch", (_event, request: LaunchRequest) =>
        controller?.launch(request)
    );
    ipcMain.handle("launcher:stop", () => controller?.stop());
    ipcMain.handle("launcher:get-runtime", () => controller?.getRuntime());
    ipcMain.handle("launcher:open-novnc", () => controller?.openNoVnc());
};

app.whenReady().then(async () => {
    if (isDevcontainer) {
        dialog.showErrorBox(
            "Run TrickFire Sim on the host",
            "The launcher manages Docker and cannot run from inside the simulations devcontainer. Use npm run dev:renderer to work on the interface here, or run npm run dev from a host terminal."
        );
        app.exit(1);
        return;
    }
    registerIpc();
    await createWindow();
    app.on("activate", () => {
        if (BrowserWindow.getAllWindows().length === 0) void createWindow();
    });
});

app.on("before-quit", (event) => {
    if (quitting || !controller?.getRuntime().active) return;
    event.preventDefault();
    void controller.stop().finally(() => {
        quitting = true;
        app.quit();
    });
});

app.on("window-all-closed", () => {
    if (process.platform !== "darwin") app.quit();
});
