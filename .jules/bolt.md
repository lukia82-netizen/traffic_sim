## 2026-03-02 - Path and Lane Grouping for Vehicle Conflict Evaluation

**Learning:** In multi-lane traffic simulation loops, flat linear iteration over all vehicle snapshots to evaluate conflict points and leader headway causes $O(N^2)$ inner loop overhead per tick. Grouping snapshots into fixed-size stack arrays indexed by lane (`[Vec<ConflictSnap>; 4]`) and path (`[Vec<ConflictSnap>; 8]`) in a single $O(N)$ pass reduces snapshot lookup complexity to $O(1)$ bucket access, giving an ~8x speedup.

**Action:** Always group vehicle snapshots into lane-based and path-based fixed arrays at the start of simulation steps before running spatial interaction, conflict evaluation, or leader queries.
