# Bolt's Performance Journal

## 2025-02-18 - Precalculated Arc Lengths & Grouped Path Snapshots in Hot Loop
**Learning:** In continuous trajectory path-following simulations, computing Bezier arc distances via numerical quadrature loops inside pairwise vehicle conflict evaluations is an O(N) operation per candidate pair. Precalculating a 64-sample cumulative arc-length lookup table on curve path creation converts arc distance calculations to O(1) linear interpolation. Additionally, categorizing vehicle snapshots into lane-indexed and path-indexed arrays in a single O(N) pass removes full linear scans in leader, oncoming, and conflict evaluations.
**Action:** Always precalculate immutable curve properties (e.g. cumulative arc-length tables) during path catalogue initialization, and group snapshot arrays by path/lane ID prior to evaluating pairwise trajectory interactions.
