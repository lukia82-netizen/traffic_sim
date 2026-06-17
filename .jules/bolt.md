## 2026-06-17 - [O(N^2) Simulation Loop]
**Learning:** The simulation core uses nested loops to find leaders and oncoming vehicles, leading to O(N^2) complexity. This becomes a bottleneck as the number of vehicles increases.
**Action:** Use spatial partitioning or per-lane sorted lists to reduce lookup complexity to O(1) or O(log N) after an O(N) pre-processing step.
