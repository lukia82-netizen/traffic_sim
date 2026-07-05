## 2026-07-05 - [O(N log N) Traffic Simulation]
**Learning:** Grouping entities by spatial properties (like lanes) and sorting them allows replacing (N)$ searches with (\log N)$ binary searches or (1)$ lookups. This is significantly more efficient than global (N^2)$ checks in a simulation tick.
**Action:** Always consider spatial partitioning or property-based grouping when implementing multi-agent interactions to maintain high performance as the number of agents grows.
