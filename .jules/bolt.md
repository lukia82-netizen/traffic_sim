## 2024-05-22 - [Algorithmic Optimization of IDM and Left Turn Logic]
**Learning:** The traffic simulation was bottlenecked by O(N²) loops in `idm_step` and `left_turn_step` where every vehicle iterated over all other vehicles to find leaders or oncoming traffic.
**Action:** Grouping vehicles by lane and sorting them by progress along the travel direction reduces the complexity to O(N log N). Finding the immediate leader becomes an O(1) operation after sorting.
