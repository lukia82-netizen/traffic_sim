## 2025-05-15 - [O(N^2) Bottleneck in Simulation Updates]
**Learning:** The traffic simulation performed full $O(N^2)$ iterations over all vehicles for every tick to find leaders and oncoming traffic. This caused significant performance degradation as the number of vehicles increased.

**Action:** Group vehicles by lane and sort them by distance to the intersection at the start of each tick. Use binary search ($O(\log M)$) for leader lookups and lane-specific filtering for yield checks, reducing overall complexity to $O(N \log N)$.

### Baseline Benchmark (1000 vehicles, 100 ticks)
- O(N^2) Implementation: ~145ms
- O(N log N) Implementation: ~10.7ms (approx. 13.5x faster)
