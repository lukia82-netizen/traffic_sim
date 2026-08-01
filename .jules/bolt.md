# Bolt's Performance Journal

## 2024-11-20 - Initial Setup
**Learning:** Found that the simulation code has O(N^2) complexity in `idm_step` and `left_turn_step` because of full flat scans over all vehicles when looking up leaders, oncoming vehicles, and conflict points.
**Action:** Plan to implement spatial/lane-based partitioning or grouping to reduce the lookup scope from O(N) to O(1) or O(M) where M is the average number of vehicles per lane/path.
