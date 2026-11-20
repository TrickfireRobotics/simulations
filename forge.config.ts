import type { ForgeConfig } from "@electron-forge/shared-types";
import { MakerSquirrel } from "@electron-forge/maker-squirrel";
import { MakerZIP } from "@electron-forge/maker-zip";
import { MakerRpm } from "@electron-forge/maker-rpm";
import { MakerDeb } from "@electron-forge/maker-deb";
import { AutoUnpackNativesPlugin } from "@electron-forge/plugin-auto-unpack-natives";
import { FusesPlugin } from "@electron-forge/plugin-fuses";
import { VitePlugin } from "@electron-forge/plugin-vite";
import { FuseV1Options, FuseVersion } from "@electron/fuses";

const config: ForgeConfig = {
    packagerConfig: {
        asar: true
    },
    rebuildConfig: {},
    makers: [
        new MakerSquirrel({}),
        new MakerZIP({}, ["darwin"]),
        new MakerRpm({}),
        new MakerDeb({})
    ],
    plugins: [
        new AutoUnpackNativesPlugin({}),
        new VitePlugin({
            build: [
                {
                    entry: "launcher/src/main.ts",
                    config: "vite.main.config.ts",
                    target: "main"
                },
                {
                    entry: "launcher/src/preload.ts",
                    config: "vite.preload.config.ts",
                    target: "preload"
                }
            ],
            renderer: [
                {
                    name: "main_window",
                    config: "vite.renderer.config.ts"
                }
            ]
        }),
        new FusesPlugin({
            version: FuseVersion.V1,
            [FuseV1Options.RunAsNode]: false,
            [FuseV1Options.EnableCookieEncryption]: true,
            [FuseV1Options.EnableNodeOptionsEnvironmentVariable]: false,
            [FuseV1Options.EnableNodeCliInspectArguments]: false,
            [FuseV1Options.EnableEmbeddedAsarIntegrityValidation]: true,
            [FuseV1Options.OnlyLoadAppFromAsar]: true
        })
    ]
};

export default config;
