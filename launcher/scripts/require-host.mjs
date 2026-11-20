if (process.env.TRICKFIRE_DEVCONTAINER === "1") {
    console.error(
        `The simulations dashboard must run on the host not inside the simulations devcontainer!`
    );
    process.exit(1);
}
