//! Path geometry for trajectory-following: position + tangent in [0, 1] progress.

use glam::Vec2;

/// Normalised trajectory: `t` ∈ [0, 1] maps along the curve; speed in world units/s
/// advances progress by `velocity * dt / path_length`.
pub trait Path: Send + Sync {
    fn point(&self, t: f32) -> Vec2;
    /// Unit tangent (direction of increasing `t`).
    fn tangent(&self, t: f32) -> Vec2;
    /// Total arc length.
    fn path_length(&self) -> f32;
    /// Arc length from `t0` to `t1` along the curve (clamped segment).
    fn arc_distance(&self, t0: f32, t1: f32) -> f32;
    /// `t` in [0,1] that minimises distance to `p` (reserved for projection / tooling).
    #[allow(dead_code)]
    fn closest_t(&self, p: Vec2) -> f32;
}

// ─── Straight line ────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct StraightLinePath {
    pub start: Vec2,
    pub end: Vec2,
}

impl StraightLinePath {
    pub fn new(start: Vec2, end: Vec2) -> Self {
        Self { start, end }
    }

    fn chord_length(&self) -> f32 {
        (self.end - self.start).length().max(1e-6)
    }
}

impl Path for StraightLinePath {
    fn point(&self, t: f32) -> Vec2 {
        let t = t.clamp(0.0, 1.0);
        self.start.lerp(self.end, t)
    }

    fn tangent(&self, _t: f32) -> Vec2 {
        let d = self.end - self.start;
        if d.length_squared() > 1e-12 {
            d.normalize()
        } else {
            Vec2::Y
        }
    }

    fn path_length(&self) -> f32 {
        self.chord_length()
    }

    fn arc_distance(&self, t0: f32, t1: f32) -> f32 {
        (t1 - t0).abs() * self.chord_length()
    }

    fn closest_t(&self, p: Vec2) -> f32 {
        let ab = self.end - self.start;
        let ap = p - self.start;
        let t = ap.dot(ab) / ab.length_squared().max(1e-8);
        t.clamp(0.0, 1.0)
    }
}

// ─── Quadratic Bezier ─────────────────────────────────────────────────────────

const SAMPLES: usize = 64;

/// Quadratic Bezier curve path with precomputed cumulative arc lengths
/// to allow O(1) arc_distance and path_length queries.
#[derive(Debug, Clone)]
pub struct BezierCurvePath {
    start: Vec2,
    ctrl: Vec2,
    end: Vec2,
    cum_len: [f32; SAMPLES + 1],
}

impl BezierCurvePath {
    pub fn new(start: Vec2, ctrl: Vec2, end: Vec2) -> Self {
        let mut cum_len = [0.0; SAMPLES + 1];
        let mut prev = start;
        let mut len = 0.0_f32;
        for i in 1..=SAMPLES {
            let t = i as f32 / SAMPLES as f32;
            let p = Self::pos(start, ctrl, end, t);
            len += (p - prev).length();
            cum_len[i] = len;
            prev = p;
        }
        Self {
            start,
            ctrl,
            end,
            cum_len,
        }
    }

    pub fn start(&self) -> Vec2 {
        self.start
    }

    pub fn ctrl(&self) -> Vec2 {
        self.ctrl
    }

    pub fn end(&self) -> Vec2 {
        self.end
    }

    pub fn pos(s: Vec2, c: Vec2, e: Vec2, t: f32) -> Vec2 {
        let u = 1.0 - t;
        u * u * s + 2.0 * u * t * c + t * t * e
    }

    pub fn tan(s: Vec2, c: Vec2, e: Vec2, t: f32) -> Vec2 {
        let u = 1.0 - t;
        let d = 2.0 * u * (c - s) + 2.0 * t * (e - c);
        if d.length_squared() > 1e-12 {
            d.normalize()
        } else {
            (e - s).normalize()
        }
    }

    fn cum_arc_len(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        let idx_f = t * SAMPLES as f32;
        let idx = (idx_f as usize).min(SAMPLES - 1);
        let frac = idx_f - idx as f32;
        self.cum_len[idx] + frac * (self.cum_len[idx + 1] - self.cum_len[idx])
    }
}

impl Path for BezierCurvePath {
    fn point(&self, t: f32) -> Vec2 {
        let t = t.clamp(0.0, 1.0);
        Self::pos(self.start, self.ctrl, self.end, t)
    }

    fn tangent(&self, t: f32) -> Vec2 {
        let t = t.clamp(0.0, 1.0);
        Self::tan(self.start, self.ctrl, self.end, t)
    }

    fn path_length(&self) -> f32 {
        self.cum_len[SAMPLES].max(1e-3)
    }

    fn arc_distance(&self, t0: f32, t1: f32) -> f32 {
        (self.cum_arc_len(t1) - self.cum_arc_len(t0)).abs()
    }

    fn closest_t(&self, p: Vec2) -> f32 {
        let mut best_t = 0.0_f32;
        let mut best_d = f32::MAX;
        for i in 0..=64 {
            let t = i as f32 / 64.0;
            let q = self.point(t);
            let d = q.distance_squared(p);
            if d < best_d {
                best_d = d;
                best_t = t;
            }
        }
        best_t
    }
}

// ─── Cubic Bézier (two interior control points) ───────────────────────────────

/// Cubic Bezier curve path with precomputed cumulative arc lengths
/// to allow O(1) arc_distance and path_length queries.
#[derive(Debug, Clone)]
pub struct CubicBezierCurvePath {
    p0: Vec2,
    p1: Vec2,
    p2: Vec2,
    p3: Vec2,
    cum_len: [f32; SAMPLES + 1],
}

impl CubicBezierCurvePath {
    pub fn new(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2) -> Self {
        let mut cum_len = [0.0; SAMPLES + 1];
        let mut prev = p0;
        let mut len = 0.0_f32;
        for i in 1..=SAMPLES {
            let t = i as f32 / SAMPLES as f32;
            let p = Self::pos(p0, p1, p2, p3, t);
            len += (p - prev).length();
            cum_len[i] = len;
            prev = p;
        }
        Self {
            p0,
            p1,
            p2,
            p3,
            cum_len,
        }
    }

    pub fn p0(&self) -> Vec2 {
        self.p0
    }

    pub fn p1(&self) -> Vec2 {
        self.p1
    }

    pub fn p2(&self) -> Vec2 {
        self.p2
    }

    pub fn p3(&self) -> Vec2 {
        self.p3
    }

    pub fn pos(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f32) -> Vec2 {
        let t = t.clamp(0.0, 1.0);
        let u = 1.0 - t;
        u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3
    }

    pub fn tan(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f32) -> Vec2 {
        let t = t.clamp(0.0, 1.0);
        let u = 1.0 - t;
        let d = 3.0 * u * u * (p1 - p0) + 6.0 * u * t * (p2 - p1) + 3.0 * t * t * (p3 - p2);
        if d.length_squared() > 1e-12 {
            d.normalize()
        } else {
            (p3 - p0).normalize()
        }
    }

    fn cum_arc_len(&self, t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        let idx_f = t * SAMPLES as f32;
        let idx = (idx_f as usize).min(SAMPLES - 1);
        let frac = idx_f - idx as f32;
        self.cum_len[idx] + frac * (self.cum_len[idx + 1] - self.cum_len[idx])
    }
}

impl Path for CubicBezierCurvePath {
    fn point(&self, t: f32) -> Vec2 {
        Self::pos(self.p0, self.p1, self.p2, self.p3, t)
    }

    fn tangent(&self, t: f32) -> Vec2 {
        Self::tan(self.p0, self.p1, self.p2, self.p3, t)
    }

    fn path_length(&self) -> f32 {
        self.cum_len[SAMPLES].max(1e-3)
    }

    fn arc_distance(&self, t0: f32, t1: f32) -> f32 {
        (self.cum_arc_len(t1) - self.cum_arc_len(t0)).abs()
    }

    fn closest_t(&self, p: Vec2) -> f32 {
        let mut best_t = 0.0_f32;
        let mut best_d = f32::MAX;
        for i in 0..=64 {
            let t = i as f32 / 64.0;
            let q = self.point(t);
            let d = q.distance_squared(p);
            if d < best_d {
                best_d = d;
                best_t = t;
            }
        }
        best_t
    }
}

/// Left-turn control point for 90° CCW turn (same construction as legacy sim).
pub fn left_turn_ctrl(start: Vec2, end: Vec2, old_dir: Vec2) -> Vec2 {
    let c1 = (end - start).dot(old_dir);
    start + c1 * old_dir
}
