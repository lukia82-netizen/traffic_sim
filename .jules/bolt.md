## 2025-02-21 - O(1) Precalculated Arc Lengths on Bezier Paths

**Learning:** `BezierCurvePath::arc_distance` and `CubicBezierCurvePath::arc_distance` were executing multi-step numerical integration loops (evaluating 10-20 polynomial points with square root distance calculations per call) during conflict checks every frame. Precalculated cumulative lookup tables (`[f32; 65]`) constructed once per path turn queries into O(1) linear interpolations with negligible memory overhead and <0.003 world units deviation.

**Action:** Look for immutable geometry paths or parametric curves evaluated repeatedly in simulation step loops, and precalculate cumulative arc-length lookup tables on construction.
