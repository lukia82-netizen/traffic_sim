## 2026-02-20 - O(N²) Neighbor Search Bottleneck
**Learning:** The traffic simulation used global linear scans (O(N²)) for car-following (leader search) and oncoming vehicle detection. This becomes a severe bottleneck as the vehicle count increases. Grouping vehicles by lane and sorting them by their projection along the lane's axis allows for O(log N) lookups via binary search.
**Action:** Use spatial partitioning or lane-based sorting for all proximity-based logic in simulations to maintain performance at scale.
