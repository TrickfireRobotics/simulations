import type { ProcessOutput } from "../types";

export const escapeHtml = (value: string | number): string =>
    String(value)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;")
        .replaceAll("'", "&#039;");

export const renderLogEntry = (entry: ProcessOutput): string => {
    const time = new Date(entry.timestamp).toLocaleTimeString([], { hour12: false });
    return `<div class="log-line log-${entry.stream}"><time>${time}</time><b>${escapeHtml(entry.source)}</b><span>${escapeHtml(entry.line)}</span></div>`;
};
