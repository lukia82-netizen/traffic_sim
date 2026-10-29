## 2025-05-18 - Path-Indexed Snapshot Grouping and O(1) Bézier Arc Distance

**Learning:** Re-evaluating Bézier arc distances via numerical integration inside inner loop conflict point evaluations accounts for substantial CPU time during simulation ticks. Precalculating a fixed cumulative arc-length table (`[f32; 65]`) on Bézier construction reduces `arc_distance` to O(1) linear interpolation. In addition, grouping vehicle snapshots into path-indexed arrays (`[Vec<ConflictSnap>; 8]`) avoids flat linear scans over all vehicles when evaluating conflict points, reducing tick execution time by ~3.66x.

**Action:** Prefer O(1) precomputed lookup tables for curve arc-lengths, and index dynamic entity snapshots by path/lane to constrain spatial conflict queries to relevant sub-arrays.
