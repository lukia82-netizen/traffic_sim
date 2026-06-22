## 2025-05-15 - [O(N²) to O(N log N) Simulation Complexity]
**Learning:** The traffic simulation's IDM calculation was a hidden O(N²) bottleneck because every vehicle scanned every other vehicle to find its leader and check for oncoming traffic. By grouping vehicles by lane and sorting them by distance along the travel direction, these lookups become O(1) after an O(N log N) sort.
**Action:** Always check for nested loops over entities in simulation `tick` functions. Prefer spatial partitioning or grouping by state (like `LaneId`) to reduce search space.

## 2025-05-15 - [Linux-incompatible Rust Toolchain]
**Learning:** Repository configuration files (`rust-toolchain.toml`, `.cargo/config.toml`) hardcoded to Windows-specific toolchains or paths (like `C:\`) will break `cargo` on Linux with cryptic "failed to join paths" or target triple errors.
**Action:** Temporarily move or rename these files during local development/verification if they cannot be modified permanently. Always restore them before submission.
