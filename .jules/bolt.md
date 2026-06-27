## 2025-05-15 - [O(N^2) to O(N log N) Simulation Optimization]
**Learning:** In traffic simulations, global O(N^2) searches for leaders or oncoming traffic are major bottlenecks. Grouping vehicles by `LaneId` and sorting them along their travel axis allows for O(1) leader lookups and significantly smaller search spaces for oncoming traffic. This reduced tick time from ~2.4s to ~112ms for 1000 vehicles (~22x speedup).
**Action:** Always check if global O(N^2) spatial queries can be partitioned into domain-specific groups (lanes, sectors, grids) to linearize or log-linearize search complexity.
