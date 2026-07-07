## 2025-05-15 - [O(N^2) to O(N log N) Traffic Simulation]
**Learning:** In traffic simulations where interactions are primarily lane-based, grouping vehicles by lane and sorting them by their 1D projection along the lane's axis allows for extremely efficient car-following lookups (using binary search) and oncoming traffic checks. This eliminates the O(N^2) bottleneck of checking all-pairs interactions.
**Action:** Always prefer spatial partitioning (like lane-based grouping) over O(N^2) global loops when simulating many interacting entities with predictable spatial constraints.
