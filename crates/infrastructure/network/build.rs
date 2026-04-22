use std::path::PathBuf;

fn main() {
    // Load the repo-root `.env` so `DEVICE_ID` reaches this crate's compiler
    // env. Cargo runs build scripts with CARGO_MANIFEST_DIR = the crate dir,
    // so we walk three levels up to the workspace root.
    let repo_root: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("..");
    let env_path = repo_root.join(".env");
    if env_path.exists() {
        let _ = dotenv::from_path(&env_path);
        println!("cargo:rerun-if-changed={}", env_path.display());
    }

    // `mqtt.rs` builds the cert/key filenames as `tower_{DEVICE_ID}-...`, so
    // the compiler needs DEVICE_ID visible to `env!()`. Re-export whatever
    // `.env` (or the shell) set, defaulting to match build.rs's default.
    let device_id = std::env::var("DEVICE_ID").unwrap_or_else(|_| "1A".to_string());
    println!("cargo:rustc-env=DEVICE_ID={}", device_id);

    // If the operator swaps cert files without touching .env, still rebuild.
    let cert = format!("tower_{}-certificate.pem.crt", device_id);
    let key = format!("tower_{}-private.pem.key", device_id);
    println!("cargo:rerun-if-changed={}", cert);
    println!("cargo:rerun-if-changed={}", key);
    println!("cargo:rerun-if-changed=AmazonRootCA1.pem");
}
