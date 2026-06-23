## 2025-05-15 - Initial Assessment
**Learning:** The traffic simulation currently uses O(N²) lookups for car-following (leaders) and oncoming traffic checks. With 16 vehicles it's fine, but it scales poorly. There are also redundant allocations and clones (e.g., `SimulationConfig`, multiple `snap` vectors).
**Action:** Optimize leader lookup by grouping vehicles by lane and sorting them once per tick. This should move the complexity towards O(N log N).

## 2025-05-15 - Baseline Measurement
**Learning:** O(N²) leader lookup for 1000 vehicles over 100 ticks takes ~3.07s on this environment.
**Action:** Implement O(N log N) lane-based sorting and lookup to reduce this significantly.

## 2025-05-15 - Optimization Results
**Learning:** Moving from O(N²) to O(N log N) by grouping vehicles by lane and sorting them once per tick yielded a ~160x speedup (from 3.07s to 19ms for 1000 vehicles / 100 ticks). Using `#[repr(usize)]` on the `LaneId` enum allowed for efficient array indexing.
**Action:** Always consider lane-based grouping for traffic simulations to avoid quadratic complexity.
