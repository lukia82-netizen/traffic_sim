//! Node–segment road graph: cubic spine, offset quadratic lane splines, incremental junction conflicts.

use glam::Vec2;
use serde::Serialize;
use std::sync::Arc;

use crate::collision::ConflictRegistry;
use crate::path::{BezierCurvePath, CubicBezierCurvePath, Path, left_turn_ctrl};

pub const P_STRAIGHT_N: u8 = 0;
pub const P_STRAIGHT_S: u8 = 1;
pub const P_STRAIGHT_E: u8 = 2;
pub const P_STRAIGHT_W: u8 = 3;
pub const P_BEZIER_N: u8 = 4;
pub const P_BEZIER_S: u8 = 5;
pub const P_BEZIER_E: u8 = 6;
pub const P_BEZIER_W: u8 = 7;

#[derive(Debug, Clone)]
pub struct RoadNode {
    pub id: u32,
    pub position: Vec2,
}

#[derive(Debug, Clone)]
pub struct RoadSegment {
    pub id: u32,
    pub node_a: u32,
    pub node_b: u32,
    pub ctrl1: Vec2,
    pub ctrl2: Vec2,
    pub lane_count: u8,
    pub lane_spacing: f32,
    /// Catalogue paths from segments sharing this id are incrementally crossed against new paths.
    pub junction_group: u32,
}

fn lerp_vec(a: Vec2, b: Vec2, t: f32) -> Vec2 {
    a * (1.0 - t) + b * t
}

impl RoadSegment {
    fn node_pos(nodes: &[RoadNode], id: u32) -> Option<Vec2> {
        nodes.iter().find(|n| n.id == id).map(|n| n.position)
    }

    pub fn central_spline(&self, nodes: &[RoadNode]) -> Option<CubicBezierCurvePath> {
        let p0 = Self::node_pos(nodes, self.node_a)?;
        let p3 = Self::node_pos(nodes, self.node_b)?;
        Some(CubicBezierCurvePath::new(p0, self.ctrl1, self.ctrl2, p3))
    }

    pub fn generate_lanes(&self, nodes: &[RoadNode]) -> Vec<BezierCurvePath> {
        let Some(central) = self.central_spline(nodes) else {
            return Vec::new();
        };
        let n_lanes = self.lane_count.max(1);
        let mid = (n_lanes.saturating_sub(1)) as f32 * 0.5;
        (0..n_lanes)
            .map(|i| {
                let off = (i as f32 - mid) * self.lane_spacing;
                offset_lane_quadratic(&central, off)
            })
            .collect()
    }

    pub fn visual_strip(
        &self,
        nodes: &[RoadNode],
        road_half_width: f32,
        samples: usize,
    ) -> Option<RoadVisualStrip> {
        let c = self.central_spline(nodes)?;
        let steps = samples.max(8);
        let mut left_edge = Vec::with_capacity(steps + 1);
        let mut right_edge = Vec::with_capacity(steps + 1);
        for i in 0..=steps {
            let t = i as f32 / steps as f32;
            let p = c.point(t);
            let n = lateral_normal(&c, t);
            left_edge.push([(p + n * road_half_width).x, (p + n * road_half_width).y]);
            right_edge.push([(p - n * road_half_width).x, (p - n * road_half_width).y]);
        }

        let lanes = self.generate_lanes(nodes);
        let lane_samples = 24usize;
        let lane_center_polylines: Vec<Vec<[f32; 2]>> = lanes
            .iter()
            .map(|lane| {
                (0..=lane_samples)
                    .map(|j| {
                        let t = j as f32 / lane_samples as f32;
                        let q = lane.point(t);
                        [q.x, q.y]
                    })
                    .collect()
            })
            .collect();

        let lane_dividers = midlines_between_polylines(&lane_center_polylines);

        Some(RoadVisualStrip {
            segment_id: self.id,
            left_edge,
            right_edge,
            lane_center_polylines,
            lane_dividers,
        })
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct RoadVisualStrip {
    pub segment_id: u32,
    pub left_edge: Vec<[f32; 2]>,
    pub right_edge: Vec<[f32; 2]>,
    pub lane_center_polylines: Vec<Vec<[f32; 2]>>,
    pub lane_dividers: Vec<Vec<[f32; 2]>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PathDebugWire {
    pub id: u8,
    pub kind: u8,
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}

pub struct BuiltRoad {
    pub paths: Vec<Arc<dyn Path>>,
    pub path_debug: Vec<PathDebugWire>,
    pub conflicts: ConflictRegistry,
    pub road_visuals: Vec<RoadVisualStrip>,
}

fn lateral_normal(c: &CubicBezierCurvePath, t: f32) -> Vec2 {
    let tg = CubicBezierCurvePath::tan(c.p0, c.p1, c.p2, c.p3, t);
    let n = Vec2::new(-tg.y, tg.x);
    if n.length_squared() > 1e-12 {
        n.normalize()
    } else {
        Vec2::X
    }
}

fn offset_lane_quadratic(c: &CubicBezierCurvePath, offset: f32) -> BezierCurvePath {
    let s0 = c.point(0.0) + lateral_normal(c, 0.0) * offset;
    let sm = c.point(0.5) + lateral_normal(c, 0.5) * offset;
    let s1 = c.point(1.0) + lateral_normal(c, 1.0) * offset;
    let ctrl = 2.0 * sm - 0.5 * (s0 + s1);
    BezierCurvePath::new(s0, ctrl, s1)
}

fn midlines_between_polylines(polys: &[Vec<[f32; 2]>]) -> Vec<Vec<[f32; 2]>> {
    let mut out = Vec::new();
    for w in polys.windows(2) {
        let a = &w[0];
        let b = &w[1];
        let n = a.len().min(b.len());
        let mid: Vec<[f32; 2]> = (0..n)
            .map(|i| {
                [(a[i][0] + b[i][0]) * 0.5, (a[i][1] + b[i][1]) * 0.5]
            })
            .collect();
        out.push(mid);
    }
    out
}

fn straight_debug_id(segment_id: u32) -> u8 {
    match segment_id {
        0 => P_STRAIGHT_N,
        1 => P_STRAIGHT_S,
        2 => P_STRAIGHT_E,
        3 => P_STRAIGHT_W,
        _ => 255,
    }
}

fn wire_bezier(id: u8, b: &BezierCurvePath) -> PathDebugWire {
    PathDebugWire {
        id,
        kind: 1,
        x0: b.start.x,
        y0: b.start.y,
        x1: b.ctrl.x,
        y1: b.ctrl.y,
        x2: b.end.x,
        y2: b.end.y,
    }
}

/// Holds topology and extends the path catalogue + conflict registry incrementally per segment.
pub struct RoadGraph {
    pub nodes: Vec<RoadNode>,
    pub segments: Vec<RoadSegment>,
    pub catalog: Vec<Arc<dyn Path>>,
    pub path_debug: Vec<PathDebugWire>,
    pub conflicts: ConflictRegistry,
    path_junction_group: Vec<u32>,
    epsilon: f32,
    pub road_visuals: Vec<RoadVisualStrip>,
}

impl RoadGraph {
    pub fn new(epsilon: f32) -> Self {
        Self {
            nodes: Vec::new(),
            segments: Vec::new(),
            catalog: Vec::new(),
            path_debug: Vec::new(),
            conflicts: ConflictRegistry::default(),
            path_junction_group: Vec::new(),
            epsilon,
            road_visuals: Vec::new(),
        }
    }

    pub fn add_segment(&mut self, seg: RoadSegment, road_half_width: f32) {
        let lanes = seg.generate_lanes(&self.nodes);
        let base = self.catalog.len();
        let new_indices: Vec<usize> = (0..lanes.len()).map(|k| base + k).collect();

        let mut lane_idx = 0usize;
        for lane in lanes {
            let id = straight_debug_id(seg.id);
            self.path_debug.push(wire_bezier(id, &lane));
            self.path_junction_group.push(seg.junction_group);
            self.catalog.push(Arc::new(lane) as Arc<dyn Path>);
            lane_idx += 1;
        }
        let _ = lane_idx;

        let neighbor_indices: Vec<usize> = (0..base)
            .filter(|&i| {
                self.path_junction_group
                    .get(i)
                    .copied()
                    .unwrap_or(u32::MAX)
                    == seg.junction_group
            })
            .collect();

        self.conflicts.generate_conflict_map_incremental(
            &self.catalog,
            &new_indices,
            &neighbor_indices,
            self.epsilon,
        );

        if let Some(strip) = seg.visual_strip(&self.nodes, road_half_width, 32) {
            self.road_visuals.push(strip);
        }

        self.segments.push(seg);
    }

    /// Left-turn template arcs (same geometry as legacy intersection).
    pub fn add_bezier_arc(&mut self, lane: BezierCurvePath, debug_id: u8, junction_group: u32) {
        let base = self.catalog.len();
        let new_idx = vec![base];
        self.path_debug.push(wire_bezier(debug_id, &lane));
        self.path_junction_group.push(junction_group);
        self.catalog.push(Arc::new(lane) as Arc<dyn Path>);

        let neighbor_indices: Vec<usize> = (0..base)
            .filter(|&i| self.path_junction_group[i] == junction_group)
            .collect();

        self.conflicts.generate_conflict_map_incremental(
            &self.catalog,
            &new_idx,
            &neighbor_indices,
            self.epsilon,
        );
    }

    pub fn into_built(self) -> BuiltRoad {
        BuiltRoad {
            paths: self.catalog,
            path_debug: self.path_debug,
            conflicts: self.conflicts,
            road_visuals: self.road_visuals,
        }
    }
}

/// Builds the default four-arm intersection using `RoadGraph` + incremental `generate_conflict_map_incremental`.
pub fn build_intersection_catalog(
    lane_offset: f32,
    viewport_half: f32,
    stop_line_offset: f32,
    bezier_exit: f32,
    conflict_epsilon: f32,
) -> BuiltRoad {
    let lo = lane_offset;
    let vh = viewport_half;
    let stop = stop_line_offset;
    let ext = bezier_exit;
    let jn = 0u32;
    let road_half = 0.52_f32;

    let nodes = vec![
        RoadNode {
            id: 0,
            position: Vec2::new(lo, vh),
        },
        RoadNode {
            id: 1,
            position: Vec2::new(lo, -vh),
        },
        RoadNode {
            id: 2,
            position: Vec2::new(-lo, -vh),
        },
        RoadNode {
            id: 3,
            position: Vec2::new(-lo, vh),
        },
        RoadNode {
            id: 4,
            position: Vec2::new(vh, -lo),
        },
        RoadNode {
            id: 5,
            position: Vec2::new(-vh, -lo),
        },
        RoadNode {
            id: 6,
            position: Vec2::new(-vh, lo),
        },
        RoadNode {
            id: 7,
            position: Vec2::new(vh, lo),
        },
    ];

    let mut g = RoadGraph::new(conflict_epsilon);
    g.nodes = nodes;

    let seg_n = RoadSegment {
        id: 0,
        node_a: 0,
        node_b: 1,
        ctrl1: lerp_vec(g.nodes[0].position, g.nodes[1].position, 1.0 / 3.0),
        ctrl2: lerp_vec(g.nodes[0].position, g.nodes[1].position, 2.0 / 3.0),
        lane_count: 1,
        lane_spacing: 0.4,
        junction_group: jn,
    };
    let seg_s = RoadSegment {
        id: 1,
        node_a: 2,
        node_b: 3,
        ctrl1: lerp_vec(g.nodes[2].position, g.nodes[3].position, 1.0 / 3.0),
        ctrl2: lerp_vec(g.nodes[2].position, g.nodes[3].position, 2.0 / 3.0),
        lane_count: 1,
        lane_spacing: 0.4,
        junction_group: jn,
    };
    let seg_e = RoadSegment {
        id: 2,
        node_a: 4,
        node_b: 5,
        ctrl1: lerp_vec(g.nodes[4].position, g.nodes[5].position, 1.0 / 3.0),
        ctrl2: lerp_vec(g.nodes[4].position, g.nodes[5].position, 2.0 / 3.0),
        lane_count: 1,
        lane_spacing: 0.4,
        junction_group: jn,
    };
    let seg_w = RoadSegment {
        id: 3,
        node_a: 6,
        node_b: 7,
        ctrl1: lerp_vec(g.nodes[6].position, g.nodes[7].position, 1.0 / 3.0),
        ctrl2: lerp_vec(g.nodes[6].position, g.nodes[7].position, 2.0 / 3.0),
        lane_count: 1,
        lane_spacing: 0.4,
        junction_group: jn,
    };

    g.add_segment(seg_n, road_half);
    g.add_segment(seg_s, road_half);
    g.add_segment(seg_e, road_half);
    g.add_segment(seg_w, road_half);

    let templates: [(u8, Vec2, Vec2); 4] = [
        (
            P_BEZIER_N,
            Vec2::new(0.0, -1.0),
            Vec2::new(lo, stop),
        ),
        (
            P_BEZIER_S,
            Vec2::new(0.0, 1.0),
            Vec2::new(-lo, -stop),
        ),
        (
            P_BEZIER_E,
            Vec2::new(-1.0, 0.0),
            Vec2::new(stop, -lo),
        ),
        (
            P_BEZIER_W,
            Vec2::new(1.0, 0.0),
            Vec2::new(-stop, lo),
        ),
    ];

    for (id, old_dir, start) in templates {
        let new_dir = Vec2::new(-old_dir.y, old_dir.x);
        let end = new_dir * ext + (-old_dir) * lo;
        let ctrl = left_turn_ctrl(start, end, old_dir);
        let b = BezierCurvePath::new(start, ctrl, end);
        g.add_bezier_arc(b, id, jn);
    }

    g.into_built()
}
