import { contextBridge, ipcRenderer } from "electron";
import type { LauncherApi, ProcessOutput, RuntimeState } from "./types";

const api: LauncherApi = {
    getBootstrap: () => ipcRenderer.invoke("launcher:get-bootstrap"),
    selectRepository: () => ipcRenderer.invoke("launcher:select-repository"),
    selectUrcRepository: () => ipcRenderer.invoke("launcher:select-urc-repository"),
    runDiagnostics: (mode) => ipcRenderer.invoke("launcher:diagnostics", mode),
    initializeDocker: () => ipcRenderer.invoke("launcher:initialize-docker"),
    launch: (request) => ipcRenderer.invoke("launcher:launch", request),
    stop: () => ipcRenderer.invoke("launcher:stop"),
    getRuntime: () => ipcRenderer.invoke("launcher:get-runtime"),
    openNoVnc: () => ipcRenderer.invoke("launcher:open-novnc"),
    onOutput: (listener) => {
        const wrapped = (_event: Electron.IpcRendererEvent, output: ProcessOutput) =>
            listener(output);
        ipcRenderer.on("launcher:output", wrapped);
        return () => ipcRenderer.removeListener("launcher:output", wrapped);
    },
    onRuntime: (listener) => {
        const wrapped = (_event: Electron.IpcRendererEvent, state: RuntimeState) => listener(state);
        ipcRenderer.on("launcher:runtime", wrapped);
        return () => ipcRenderer.removeListener("launcher:runtime", wrapped);
    }
};

contextBridge.exposeInMainWorld("launcher", api);
