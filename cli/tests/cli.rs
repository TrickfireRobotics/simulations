use std::process::Command;

fn sim() -> Command {
    Command::new(env!("CARGO_BIN_EXE_sim"))
}

#[test]
fn runs_successfully() {
    let output = sim().output().expect("failed to run the sim binary");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}
