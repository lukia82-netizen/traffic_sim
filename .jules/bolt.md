## 2025-05-18 - Precalculating Cumulative Arc Length Tables for Parametric Trajectories

**Learning:** Calculating trajectory arc distance dynamically via multi-step numerical integration loops inside nested simulation conflict checks is a major bottleneck. Precalculating a cumulative arc length lookup table (`[f32; 33]`) on curve construction reduces `arc_distance` and `path_length` to O(1) linear interpolation lookups, reducing frame tick computation by ~55%. Avoiding unnecessary `Vec` allocations in helper functions (`maneuver_path_ids`) and borrowing struct sections (`&cfg.idm`) further reduces tick latency down to 52.5 us/tick (3.35x speedup overall).

**Action:** Always precalculate arc length lookup tables on parametric curves when arc-distance queries are performed frequently in simulation loops, and avoid heap allocations in hot per-vehicle loops.
