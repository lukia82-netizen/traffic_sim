# Bolt's Performance Journal

## 2026-07-17 - O(N) Spatial/Path Grouping & O(log M) Leader Lookup
**Learning:** In traffic simulations where vehicles are updated in a tick loop, pairwise checks for oncoming vehicles, conflict points, and lane leaders can degrade performance to O(N²) when matching list items flatly. Grouping vehicle snapshots into lane-indexed and path-indexed arrays of vectors at the start of each step avoids full linear scans, optimizing complexity. Furthermore, sorting lane groups by distance allows using binary search (`partition_point`) instead of linear scans to locate leaders in O(log M) time.
**Action:** Always snapshot and group simulation entities by topological categories (e.g., lane, path) at the start of a step to convert expensive O(N²) cross-scans into fast O(N) grouped scans or O(log M) lookups.

## 2026-07-17 - Windows Cargo-Target Escaping in Linux Git Index
**Learning:** Standard shell-escaping mismatch errors on Linux environments can cause `git rm` or `git reset` commands to fail when attempting to remove staged Windows-style target paths (e.g., `'C:\cargo-target'`). This is because the backslashes are interpreted as escapes or paths.
**Action:** Use Python with null-byte terminated `git ls-files -z` to safely locate and physically delete directories, followed by precise `git rm --cached` using raw string byte representations to cleanly clear them.
