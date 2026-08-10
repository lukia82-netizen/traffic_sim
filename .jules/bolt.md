# Bolt's Performance Journal - Critical Learnings Only

## 2023-10-27 - Backend logic validation on headless Linux environments
**Learning:** When developing Tauri-based Rust applications on a headless Linux environment without root privileges, system dependencies like GTK and WebKit2Gtk are missing, causing `cargo check` and `cargo test` to fail.
**Action:** Isolate the core simulation logic and compile/test it using a temporary Cargo project in `/tmp` containing only the pure-Rust dependencies (`glam`, `rand`, `serde`, `toml`). This validates compilation and execution correctness cleanly without polluting the git index.
