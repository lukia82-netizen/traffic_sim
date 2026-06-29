## 2026-06-29 - [Spatial Optimization via Lane Sorting]
**Learning:** The traffic simulation's O(N²) bottleneck was caused by global car-following and oncoming checks. By grouping vehicles into lanes and sorting them by their projection along the travel axis, we can achieve O(1) leader lookups and O(N_lane) oncoming checks.
**Action:** When handling multi-agent simulations with fixed paths (like lanes), always prefer lane-local sorting for proximity and collision checks to avoid global O(N²) scans. Use `position.dot(direction)` for stable 1D projections.
