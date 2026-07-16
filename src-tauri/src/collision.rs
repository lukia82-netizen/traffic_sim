//! Geometric conflict points where two paths meet (for TTC-based yielding).

use glam::Vec2;
use std::sync::Arc;

use crate::path::Path;

/// A crossing between path A at `t_a` and path B at `t_b` in world space `pos`.
#[derive(Debug, Clone)]
pub struct ConflictPoint {
    pub path_id_a: u8,
    pub t_a: f32,
    pub path_id_b: u8,
    pub t_b: f32,
    pub pos: Vec2,
}

#[derive(Debug, Clone, Default)]
pub struct ConflictRegistry {
    pub points: Vec<ConflictPoint>,
}

impl ConflictRegistry {
    /// Pairwise scan of path catalogue; crossings closer than `epsilon` become conflict points.
    #[allow(dead_code)]
    pub fn from_catalog(catalog: &[Arc<dyn Path>], epsilon: f32) -> Self {
        let mut points = Vec::new();
        let n = catalog.len();
        for i in 0..n {
            for j in (i + 1)..n {
                push_crossing_if_new(&mut points, catalog, i, j, epsilon);
            }
        }
        Self { points }
    }

    /// Incremental conflict detection: only pairs where one path index is in `new_indices`
    /// and the other is in `neighbor_indices`, plus all unordered pairs **within** `new_indices`.
    /// Used when a new road segment is added so we do not rescan the entire catalogue.
    pub fn generate_conflict_map_incremental(
        &mut self,
        catalog: &[Arc<dyn Path>],
        new_indices: &[usize],
        neighbor_indices: &[usize],
        epsilon: f32,
    ) {
        for &i in new_indices {
            for &j in neighbor_indices {
                if i == j {
                    continue;
                }
                let (a, b) = if i < j { (i, j) } else { (j, i) };
                push_crossing_if_new(&mut self.points, catalog, a, b, epsilon);
            }
        }
        for ii in 0..new_indices.len() {
            for jj in (ii + 1)..new_indices.len() {
                let i = new_indices[ii];
                let j = new_indices[jj];
                let (a, b) = if i < j { (i, j) } else { (j, i) };
                push_crossing_if_new(&mut self.points, catalog, a, b, epsilon);
            }
        }
    }
}

fn crossing_near_duplicate(points: &[ConflictPoint], a: u8, b: u8, w: Vec2, tol: f32) -> bool {
    points.iter().any(|p| {
        let same = (p.path_id_a == a && p.path_id_b == b) || (p.path_id_a == b && p.path_id_b == a);
        same && p.pos.distance(w) < tol
    })
}

fn push_crossing_if_new(
    points: &mut Vec<ConflictPoint>,
    catalog: &[Arc<dyn Path>],
    a: usize,
    b: usize,
    epsilon: f32,
) {
    if let Some((ti, tj, w)) = find_best_crossing(&*catalog[a], &*catalog[b], epsilon) {
        let pa = a as u8;
        let pb = b as u8;
        if !crossing_near_duplicate(points, pa, pb, w, epsilon * 3.0) {
            points.push(ConflictPoint {
                path_id_a: pa,
                t_a: ti,
                path_id_b: pb,
                t_b: tj,
                pos: w,
            });
        }
    }
}

pub fn find_best_crossing(a: &dyn Path, b: &dyn Path, epsilon: f32) -> Option<(f32, f32, Vec2)> {
    let mut best_d = f32::MAX;
    let mut best: Option<(f32, f32, Vec2)> = None;
    const GRID: i32 = 48;
    for ia in 0..=GRID {
        let ta = ia as f32 / GRID as f32;
        let pa = a.point(ta);
        for ib in 0..=GRID {
            let tb = ib as f32 / GRID as f32;
            let pb = b.point(tb);
            let d = pa.distance(pb);
            if d < best_d {
                best_d = d;
                best = Some((ta, tb, (pa + pb) * 0.5));
            }
        }
    }
    if best_d > epsilon {
        return None;
    }
    best
}
