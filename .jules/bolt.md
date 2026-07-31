# Bolt's Performance Journal

## 2023-11-20 - Grouping Snapshots for Simulation Tick Optimization
**Learning:** Performing O(N) linear scans over all vehicle snapshots to find lane leaders and oncoming vehicles on every tick introduces a major O(N²) bottleneck as the vehicle count increases. Grouping snapshots into lane-indexed and path-indexed arrays of vectors at the start of each tick avoids full linear scans in oncoming checks, leader lookups, and conflict point evaluations, optimizing complexity to O(N) per tick.
**Action:** Always group entity states by spatial/routing keys (like lane_id or path_id) at the start of the simulation tick rather than querying the entire population on each entity's update.
