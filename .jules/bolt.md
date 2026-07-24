# Bolt's Performance Journal

## 2026-03-02 - Optimizing Traffic Simulation from O(N²) to O(N log N) with Spatial/Lane Partitioning
**Learning:** Grouping vehicle snapshots by lane and path ID avoids O(N) full linear scans for every vehicle during leader searches, oncoming vehicle checks, and conflict point evaluations. Caching vehicle projections `v.proj` along lane directions at snapshot creation and sorting the lane-grouped arrays enables extremely fast O(log M) binary search (`partition_point`) lookups for leaders.
**Action:** Apply spatial indexing and type-based partitioning (arrays of vectors indexed by enums) to transform O(N²) collision/leadership lookups into O(N) or O(N log N) operations.
