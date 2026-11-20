import "./styles.css";
import { createPreviewApi } from "./preview-api";
import { LauncherUi } from "./ui/controller";

if (!window.launcher && import.meta.env.MODE === "preview") {
    window.launcher = createPreviewApi();
}

const root = document.querySelector<HTMLElement>("#app");
if (!root) throw new Error("Application root is missing.");
if (!window.launcher) throw new Error("Launcher API is unavailable.");

void new LauncherUi(root, window.launcher).start();
