## 2026-07-17 - Lane and Path Vehicle Partitioning in Simulation Loop

**Learning:** In the per-tick traffic simulation loop, performing linear scans across all vehicles for leader detection, oncoming lane yielding, and path conflict point evaluation results in O(N²) quadratic scaling and repeated redundant snapshot allocations. Partitioning snapshots once per frame into stack-allocated arrays of vectors indexed by lane (`[Vec<ConflictSnap>; 4]`) and path (`[Vec<ConflictSnap>; 8]`) prunes the search space by 4x to 8x without hashing overhead.
**Action:** When evaluating multi-agent interactions across discrete lanes or spatial paths, group snapshot records into fixed-size stack arrays of vectors upfront during the step to turn global scans into targeted sub-group lookups.
