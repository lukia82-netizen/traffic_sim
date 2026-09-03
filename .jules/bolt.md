## 2025-05-18 - Precomputing Bezier Arc Lengths

**Learning:** Re-evaluating numerical integration loops (10-20 samples with `sqrt()`) in `arc_distance` during high-frequency simulation tick collision evaluations creates significant overhead. Precalculating a 64-division cumulative length table on `BezierCurvePath` construction reduces `arc_distance` and `path_length` calculations to O(1) linear interpolation while increasing resolution.

**Action:** Prefer storing O(1) cumulative length tables when instantiating static geometric paths instead of computing numerical integration loops on cold/hot execution paths.
