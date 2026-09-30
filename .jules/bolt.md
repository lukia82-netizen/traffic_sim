## 2025-05-20 - Precalculate cumulative arc length lookup tables for Bezier curve distance lookups

**Learning:** Evaluated arc distance calculations on Bezier curves inside multi-vehicle conflict point evaluation loops represented a significant bottleneck (~68–71% of simulation tick time). Precalculating a cumulative arc length lookup table (`[f32; 65]`) on path construction reduces `arc_distance` and `path_length` calculations from multi-step numerical integration loops to O(1) linear interpolation lookups, resulting in a ~3.2x–3.5x simulation tick speedup.

**Action:** Whenever parametric curves or trajectories are evaluated repeatedly in hot simulation or collision loops, precompute cumulative distance lookup tables on path instantiation instead of numerically integrating on the fly.
