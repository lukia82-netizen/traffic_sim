// Prevents an additional console window on Windows in release builds.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod collision;
mod path;
mod road;

use collision::{ConflictPoint, ConflictRegistry};
use glam::Vec2;
use path::{Path, StraightLinePath};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path as StdPath;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tauri::{AppHandle, Emitter};

// ─── Constants ────────────────────────────────────────────────────────────────

const TARGET_FPS: f64 = 60.0;
const FRAME_DURATION: Duration = Duration::from_nanos((1_000_000_000.0 / TARGET_FPS) as u64);
const LANE_OFFSET: f32 = 0.2;

/// World half-extent where `|x|` or `|y|` hits the canvas border (`sx = 400 + x*100` in `main.js`).
/// Must match `src/main.js`: `(CANVAS_SIZE / 2) / VISUAL_SCALE` (800 / 2 / 100).
const VIEWPORT_HALF_WORLD: f32 = 4.0;
const BEZIER_EXIT_DIST: f32 = 1.5;

/// Path catalogue indices (must match `path_catalog`).
const P_STRAIGHT_N: u8 = 0;
const P_STRAIGHT_S: u8 = 1;
const P_STRAIGHT_E: u8 = 2;
const P_STRAIGHT_W: u8 = 3;
const P_BEZIER_N: u8 = 4;
const P_BEZIER_S: u8 = 5;
const P_BEZIER_E: u8 = 6;
const P_BEZIER_W: u8 = 7;

// ─── Enums ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
enum VehicleStatus {
    Cruising,
    Stopped,
    Yielding,
    Crossing,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum VehicleIntent {
    Straight,
    Left,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum LaneId {
    North,
    South,
    East,
    West,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum LightPhase {
    NSGreen,
    AllRed1,
    EWGreen,
    AllRed2,
}

impl LightPhase {
    fn next(self) -> Self {
        match self {
            LightPhase::NSGreen => LightPhase::AllRed1,
            LightPhase::AllRed1 => LightPhase::EWGreen,
            LightPhase::EWGreen => LightPhase::AllRed2,
            LightPhase::AllRed2 => LightPhase::NSGreen,
        }
    }
    fn ns_green(self) -> bool {
        matches!(self, LightPhase::NSGreen)
    }
    fn ew_green(self) -> bool {
        matches!(self, LightPhase::EWGreen)
    }
}

const ALL_LANES: [LaneId; 4] = [LaneId::North, LaneId::South, LaneId::East, LaneId::West];

fn lane_direction(lane: LaneId) -> Vec2 {
    match lane {
        LaneId::North => Vec2::new(0.0, -1.0),
        LaneId::South => Vec2::new(0.0, 1.0),
        LaneId::East => Vec2::new(-1.0, 0.0),
        LaneId::West => Vec2::new(1.0, 0.0),
    }
}

fn lane_spawn_pos(lane: LaneId, dist: f32) -> Vec2 {
    match lane {
        LaneId::North => Vec2::new(LANE_OFFSET, dist),
        LaneId::South => Vec2::new(-LANE_OFFSET, -dist),
        LaneId::East => Vec2::new(dist, -LANE_OFFSET),
        LaneId::West => Vec2::new(-dist, LANE_OFFSET),
    }
}

/// Full straight leg through the viewport (used for catalogue + spawn progress).
fn straight_lane_world_path(lane: LaneId, lo: f32, half: f32) -> StraightLinePath {
    match lane {
        LaneId::North => StraightLinePath::new(Vec2::new(lo, half), Vec2::new(lo, -half)),
        LaneId::South => StraightLinePath::new(Vec2::new(-lo, -half), Vec2::new(-lo, half)),
        LaneId::East => StraightLinePath::new(Vec2::new(half, -lo), Vec2::new(-half, -lo)),
        LaneId::West => StraightLinePath::new(Vec2::new(-half, lo), Vec2::new(half, lo)),
    }
}

/// Minimum |coordinate| along the lane axis so that `‖lane_spawn_pos(lane, coord)‖ > approach_radius`
/// (vehicles are not spawned exactly on the yellow approach ring).
fn min_lane_axis_outside_approach_ring(approach_radius: f32) -> f32 {
    const MARGIN: f32 = 0.3;
    let r = (approach_radius + MARGIN).max(LANE_OFFSET + 1e-4);
    (r * r - LANE_OFFSET * LANE_OFFSET).max(0.0).sqrt()
}

fn oncoming_lane(lane: LaneId) -> LaneId {
    match lane {
        LaneId::North => LaneId::South,
        LaneId::South => LaneId::North,
        LaneId::East => LaneId::West,
        LaneId::West => LaneId::East,
    }
}

fn lane_for_direction(dir: Vec2) -> LaneId {
    if dir.x > 0.5 {
        LaneId::West
    } else if dir.x < -0.5 {
        LaneId::East
    } else if dir.y > 0.5 {
        LaneId::South
    } else {
        LaneId::North
    }
}

fn straight_path_id(lane: LaneId) -> u8 {
    match lane {
        LaneId::North => P_STRAIGHT_N,
        LaneId::South => P_STRAIGHT_S,
        LaneId::East => P_STRAIGHT_E,
        LaneId::West => P_STRAIGHT_W,
    }
}

fn bezier_path_id(lane: LaneId) -> u8 {
    match lane {
        LaneId::North => P_BEZIER_N,
        LaneId::South => P_BEZIER_S,
        LaneId::East => P_BEZIER_E,
        LaneId::West => P_BEZIER_W,
    }
}

/// Build path catalogue + conflict registry from the node–segment road graph (incremental junction mapping).
fn path_catalog_with_debug(
    cfg: &SimulationConfig,
) -> (
    Vec<Arc<dyn Path>>,
    Vec<PathDebug>,
    ConflictRegistry,
    Vec<road::RoadVisualStrip>,
) {
    let built = road::build_intersection_catalog(
        LANE_OFFSET,
        VIEWPORT_HALF_WORLD,
        cfg.idm.stop_line_offset,
        BEZIER_EXIT_DIST,
        0.12,
    );
    let dbg = built
        .path_debug
        .into_iter()
        .map(|w| PathDebug {
            id: w.id,
            kind: w.kind,
            x0: w.x0,
            y0: w.y0,
            x1: w.x1,
            y1: w.y1,
            x2: w.x2,
            y2: w.y2,
        })
        .collect();
    (built.paths, dbg, built.conflicts, built.road_visuals)
}

fn straight_path_for_lane(catalog: &[Arc<dyn Path>], lane: LaneId) -> Arc<dyn Path> {
    catalog[straight_path_id(lane) as usize].clone()
}

fn template_bezier_for_lane(catalog: &[Arc<dyn Path>], lane: LaneId) -> Arc<dyn Path> {
    catalog[bezier_path_id(lane) as usize].clone()
}

/// Project world position onto a straight segment; returns progress t.
fn progress_on_straight(path: &StraightLinePath, pos: Vec2) -> f32 {
    let ab = path.end - path.start;
    let ap = pos - path.start;
    let t = ap.dot(ab) / ab.length_squared().max(1e-8);
    t.clamp(0.0, 1.0)
}

/// Initial `path_progress` on the lane straight for `pos` (same geometry as `path_catalog`).
fn spawn_straight_progress(lane: LaneId, pos: Vec2) -> f32 {
    let seg = straight_lane_world_path(lane, LANE_OFFSET, VIEWPORT_HALF_WORLD);
    progress_on_straight(&seg, pos)
}

// ─── Vehicle ─────────────────────────────────────────────────────────────────

struct Vehicle {
    id: usize,
    position: Vec2,
    approach_dir: Vec2,
    lane_id: LaneId,
    conflict_point: Vec2,
    max_speed: f32,
    current_speed: f32,
    acceleration: f32,
    status: VehicleStatus,
    intent: VehicleIntent,
    leader_id: Option<usize>,
    oncoming_yield_id: Option<usize>,
    /// Kinematic path (straight segment or Bezier); progress 0..1 along curve length.
    current_path: Arc<dyn Path>,
    path_id: u8,
    path_progress: f32,
    path_length_cached: f32,
    /// True while following a left-turn Bezier (path_id 4–7).
    on_bezier_turn: bool,
}

// ─── Traffic lights ───────────────────────────────────────────────────────────

struct TrafficLightManager {
    phase: LightPhase,
    elapsed: f32,
    green_duration: f32,
    allred_duration: f32,
}

impl TrafficLightManager {
    fn new(green_duration: f32, allred_duration: f32) -> Self {
        Self {
            phase: LightPhase::NSGreen,
            elapsed: 0.0,
            green_duration,
            allred_duration,
        }
    }

    fn tick(&mut self, dt: f32) {
        self.elapsed += dt;
        let duration = match self.phase {
            LightPhase::NSGreen | LightPhase::EWGreen => self.green_duration,
            LightPhase::AllRed1 | LightPhase::AllRed2 => self.allred_duration,
        };
        if self.elapsed >= duration {
            self.elapsed -= duration;
            self.phase = self.phase.next();
        }
    }

    fn is_green(&self, lane: LaneId) -> bool {
        match lane {
            LaneId::North | LaneId::South => self.phase.ns_green(),
            LaneId::East | LaneId::West => self.phase.ew_green(),
        }
    }
}

// ─── Config (unchanged schema) ────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationSection {
    fixed_dt: f32,
    max_frames: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IntersectionSection {
    approach_radius: f32,
    clear_radius: f32,
    #[serde(default = "default_oncoming_clear_dist")]
    oncoming_clear_dist: f32,
    /// If |TTC_self − TTC_other| is below this [s], treat as ambiguous and apply virtual leader.
    #[serde(default = "default_conflict_danger_window")]
    conflict_danger_window_s: f32,
}
fn default_oncoming_clear_dist() -> f32 {
    3.0
}
fn default_conflict_danger_window() -> f32 {
    0.35
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TrafficLightSection {
    #[serde(default = "default_green_duration")]
    green_duration: f32,
    #[serde(default = "default_allred_duration")]
    allred_duration: f32,
}
fn default_green_duration() -> f32 {
    8.0
}
fn default_allred_duration() -> f32 {
    2.0
}

impl Default for TrafficLightSection {
    fn default() -> Self {
        Self {
            green_duration: default_green_duration(),
            allred_duration: default_allred_duration(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IdmSection {
    a_max: f32,
    b_comf: f32,
    delta: f32,
    s0: f32,
    t_headway: f32,
    stop_line_offset: f32,
    #[serde(default = "default_vehicle_length")]
    vehicle_length: f32,
}
fn default_vehicle_length() -> f32 {
    0.2
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SpawnSection {
    #[serde(default = "default_spawn_distance")]
    spawn_distance: f32,
    #[serde(default = "default_offmap_distance")]
    offmap_distance: f32,
    #[serde(default = "default_num_vehicles")]
    num_vehicles: usize,
    #[serde(default = "default_left_turn_probability")]
    left_turn_probability: f32,
}
fn default_spawn_distance() -> f32 {
    1.5
}
fn default_offmap_distance() -> f32 {
    4.5
}
fn default_num_vehicles() -> usize {
    4
}
fn default_left_turn_probability() -> f32 {
    0.20
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct VehicleSection {
    max_speed: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationConfig {
    simulation: SimulationSection,
    intersection: IntersectionSection,
    #[serde(default)]
    traffic_lights: TrafficLightSection,
    idm: IdmSection,
    spawn: SpawnSection,
    vehicle: VehicleSection,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            simulation: SimulationSection {
                fixed_dt: 0.016,
                max_frames: 0,
            },
            intersection: IntersectionSection {
                approach_radius: 4.0,
                clear_radius: 0.8,
                oncoming_clear_dist: 3.0,
                conflict_danger_window_s: default_conflict_danger_window(),
            },
            traffic_lights: TrafficLightSection::default(),
            idm: IdmSection {
                a_max: 1.5,
                b_comf: 3.0,
                delta: 4.0,
                s0: 0.6,
                t_headway: 0.8,
                stop_line_offset: 1.2,
                vehicle_length: 0.4,
            },
            spawn: SpawnSection {
                spawn_distance: 1.5,
                offmap_distance: 6.0,
                num_vehicles: 16,
                left_turn_probability: 0.20,
            },
            vehicle: VehicleSection { max_speed: 1.0 },
        }
    }
}

impl SimulationConfig {
    const FILE_PATH: &'static str = "sim_config.toml";

    fn load_or_create() -> Self {
        let path = StdPath::new(Self::FILE_PATH);
        if !path.exists() {
            let defaults = Self::default();
            let _ = fs::write(path, defaults.to_toml_text());
            return defaults;
        }
        match fs::read_to_string(path) {
            Ok(content) => toml::from_str(&content).unwrap_or_else(|e| {
                eprintln!("Config parse error: {e}. Using defaults.");
                Self::default()
            }),
            Err(e) => {
                eprintln!("Cannot read config: {e}. Using defaults.");
                Self::default()
            }
        }
    }

    fn to_toml_text(&self) -> String {
        format!(
            "[simulation]\n\
             fixed_dt   = {}    # wall-clock dt used at runtime\n\
             max_frames = {}    # 0 = run indefinitely\n\n\
             [intersection]\n\
             approach_radius     = {}   # approach zone circle radius\n\
             clear_radius        = {}   # clear zone circle radius\n\
             oncoming_clear_dist = {}   # oncoming vehicle within this dist blocks a left-turn [units]\n\
             conflict_danger_window_s = {}   # TTC ambiguity window for conflict yielding [s]\n\n\
             [traffic_lights]\n\
             green_duration  = {}   # green phase duration [s]\n\
             allred_duration = {}   # all-red safety gap [s]\n\n\
             [idm]\n\
             a_max            = {}   # max acceleration [m/s²]\n\
             b_comf           = {}   # comfortable braking [m/s²]\n\
             delta            = {}   # acceleration exponent\n\
             s0               = {}   # minimum jam gap [units]\n\
             t_headway        = {}   # safe time headway [s]\n\
             stop_line_offset = {}   # stop position before conflict point [units]\n\
             vehicle_length   = {}   # bumper-to-bumper correction [units]\n\n\
             [spawn]\n\
             num_vehicles           = {}   # total vehicles to spawn\n\
             spawn_distance         = {}   # spacing along lane toward intersection [world units]; first car at canvas edge (see VIEWPORT_HALF_WORLD in main.rs)\n\
             offmap_distance        = {}   # removal distance past centre [units]\n\
             left_turn_probability  = {}   # fraction of vehicles with left-turn intent [0–1]\n\n\
             [vehicle]\n\
             max_speed = {}   # max speed [units/s]\n",
            self.simulation.fixed_dt,
            self.simulation.max_frames,
            self.intersection.approach_radius,
            self.intersection.clear_radius,
            self.intersection.oncoming_clear_dist,
            self.intersection.conflict_danger_window_s,
            self.traffic_lights.green_duration,
            self.traffic_lights.allred_duration,
            self.idm.a_max,
            self.idm.b_comf,
            self.idm.delta,
            self.idm.s0,
            self.idm.t_headway,
            self.idm.stop_line_offset,
            self.idm.vehicle_length,
            self.spawn.num_vehicles,
            self.spawn.spawn_distance,
            self.spawn.offmap_distance,
            self.spawn.left_turn_probability,
            self.vehicle.max_speed,
        )
    }
}

// ─── Simulation ───────────────────────────────────────────────────────────────

struct SimulationState {
    vehicles: Vec<Vehicle>,
    lights: TrafficLightManager,
    cfg: SimulationConfig,
    path_catalog: Vec<Arc<dyn Path>>,
    path_debug: Vec<PathDebug>,
    conflict_registry: ConflictRegistry,
    /// Populated each `idm_step`: TTC-based yields (virtual leader at conflict).
    active_conflicts: Vec<ActiveConflictDto>,
    road_visuals: Vec<road::RoadVisualStrip>,
}

#[derive(Serialize, Clone, Debug)]
struct ActiveConflictDto {
    x: f32,
    y: f32,
    yielder_id: usize,
    other_id: usize,
}

/// Within this TTC band [s], lower vehicle ID has priority (higher ID must yield / target ~0 speed).
const CONFLICT_TTC_TIE_S: f32 = 0.2;

fn braking_distance_horizon(speed: f32, idm: &IdmSection) -> f32 {
    let v = speed.max(0.0);
    v * v / (2.0 * idm.b_comf.max(0.1)) + idm.s0
}

fn conflict_params_on_path(cp: &ConflictPoint, path_pid: u8) -> Option<(f32, u8, f32)> {
    if cp.path_id_a == path_pid {
        Some((cp.t_a, cp.path_id_b, cp.t_b))
    } else if cp.path_id_b == path_pid {
        Some((cp.t_b, cp.path_id_a, cp.t_a))
    } else {
        None
    }
}

/// Paths that must be conflict-free for atomic entry (straight + future Bezier for left intent).
fn maneuver_path_ids(v: &Vehicle) -> Vec<u8> {
    if v.on_bezier_turn {
        return vec![v.path_id];
    }
    if v.intent == VehicleIntent::Left {
        vec![straight_path_id(v.lane_id), bezier_path_id(v.lane_id)]
    } else {
        vec![v.path_id]
    }
}

fn effective_progress_on_path(v: &Vehicle, path_pid: u8) -> Option<f32> {
    if v.path_id == path_pid {
        return Some(v.path_progress);
    }
    if v.intent == VehicleIntent::Left && !v.on_bezier_turn && path_pid == bezier_path_id(v.lane_id) {
        return Some(0.0);
    }
    None
}

fn past_stop_line_geom(v: &Vehicle, stop_offset: f32) -> bool {
    let to_c = v.position - v.conflict_point;
    to_c.length() - stop_offset <= 0.0
}

fn idm_virtual_leader_term(dist: f32, speed: f32, v_lead: f32, idm: &IdmSection) -> f32 {
    let s = dist.max(1e-4);
    let dv = speed - v_lead;
    let sqrt_ab = (idm.a_max * idm.b_comf).sqrt();
    let s_star = (idm.s0 + speed * idm.t_headway + speed * dv / (2.0 * sqrt_ab)).max(0.0);
    (s_star / s).powi(2)
}

#[derive(Clone, Copy)]
struct ConflictSnap {
    id: usize,
    position: Vec2,
    approach_dir: Vec2,
    current_speed: f32,
    lane_id: LaneId,
    path_id: u8,
    path_progress: f32,
    on_bezier_turn: bool,
    proj: f32,
}

fn snapshot_conflict(vehicles: &[Vehicle]) -> Vec<ConflictSnap> {
    vehicles
        .iter()
        .map(|v| {
            let proj = v.position.dot(lane_direction(v.lane_id));
            ConflictSnap {
                id: v.id,
                position: v.position,
                approach_dir: v.approach_dir,
                current_speed: v.current_speed,
                lane_id: v.lane_id,
                path_id: v.path_id,
                path_progress: v.path_progress,
                on_bezier_turn: v.on_bezier_turn,
                proj,
            }
        })
        .collect()
}

fn lane_to_index(lane: LaneId) -> usize {
    match lane {
        LaneId::North => 0,
        LaneId::South => 1,
        LaneId::East => 2,
        LaneId::West => 3,
    }
}

fn get_grouped_snaps(vehicles: &[Vehicle]) -> ([Vec<ConflictSnap>; 4], [Vec<ConflictSnap>; 8]) {
    let mut lane_snaps: [Vec<ConflictSnap>; 4] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut path_snaps: [Vec<ConflictSnap>; 8] = [
        Vec::new(), Vec::new(), Vec::new(), Vec::new(),
        Vec::new(), Vec::new(), Vec::new(), Vec::new(),
    ];

    for v in vehicles {
        let proj = v.position.dot(lane_direction(v.lane_id));
        let snap = ConflictSnap {
            id: v.id,
            position: v.position,
            approach_dir: v.approach_dir,
            current_speed: v.current_speed,
            lane_id: v.lane_id,
            path_id: v.path_id,
            path_progress: v.path_progress,
            on_bezier_turn: v.on_bezier_turn,
            proj,
        };
        lane_snaps[lane_to_index(v.lane_id)].push(snap);
        if (v.path_id as usize) < 8 {
            path_snaps[v.path_id as usize].push(snap);
        }
    }

    for lane_group in &mut lane_snaps {
        lane_group.sort_by(|a, b| a.proj.partial_cmp(&b.proj).unwrap_or(std::cmp::Ordering::Equal));
    }

    (lane_snaps, path_snaps)
}

/// One conflict geometry: `path_pid` is the side we evaluate for vehicle `v` at `eff_prog`.
fn evaluate_conflict_point(
    v: &Vehicle,
    cp: &ConflictPoint,
    path_pid: u8,
    my_t_cp: f32,
    other_pid: u8,
    other_t_cp: f32,
    eff_prog: f32,
    path_snaps: &[Vec<ConflictSnap>; 8],
    cat: &[Arc<dyn Path>],
    idm: &IdmSection,
    danger_window: f32,
    horizon: Option<f32>,
    record_active: Option<&mut Vec<ActiveConflictDto>>,
) -> f32 {
    if my_t_cp <= eff_prog + 1e-5 {
        return 0.0;
    }

    let geom = &cat[path_pid as usize];
    let dist_v = geom.arc_distance(eff_prog, my_t_cp).max(0.0);
    if let Some(h) = horizon {
        if dist_v > h + 1e-3 {
            return 0.0;
        }
    }

    let speed = v.current_speed.max(0.0);
    let ttc_v = dist_v / speed.max(0.08);

    let path_b = &cat[other_pid as usize];
    // Best conflicting vehicle on `other_pid`: (id, TTC to cp, scalar speed along path).
    let mut best_o: Option<(usize, f32, f32)> = None;

    for snap in &path_snaps[other_pid as usize] {
        if snap.id == v.id {
            continue;
        }
        if snap.path_progress >= other_t_cp - 1e-4 {
            continue;
        }
        let dist_o = path_b.arc_distance(snap.path_progress, other_t_cp).max(0.0);
        let o_spd = snap.current_speed.max(0.0);
        let ttc_o = dist_o / o_spd.max(0.08);
        let pick = match best_o {
            None => true,
            Some((_, best_ttc, _)) => ttc_o < best_ttc,
        };
        if pick {
            best_o = Some((snap.id, ttc_o, o_spd));
        }
    }

    let Some((oid, ttc_o, v_lead)) = best_o else {
        return 0.0;
    };

    let dt = (ttc_v - ttc_o).abs();
    let mut term = 0.0_f32;

    if dt < CONFLICT_TTC_TIE_S {
        if v.id > oid {
            term = idm_virtual_leader_term(dist_v, speed, v_lead, idm).max(4.0);
            if let Some(out) = record_active {
                out.push(ActiveConflictDto {
                    x: cp.pos.x,
                    y: cp.pos.y,
                    yielder_id: v.id,
                    other_id: oid,
                });
            }
        }
    } else if ttc_o < ttc_v {
        term = idm_virtual_leader_term(dist_v, speed, v_lead, idm);
        if let Some(out) = record_active {
            out.push(ActiveConflictDto {
                x: cp.pos.x,
                y: cp.pos.y,
                yielder_id: v.id,
                other_id: oid,
            });
        }
    } else if dt < danger_window {
        term = idm_virtual_leader_term(dist_v, speed, v_lead, idm);
        if let Some(out) = record_active {
            out.push(ActiveConflictDto {
                x: cp.pos.x,
                y: cp.pos.y,
                yielder_id: v.id,
                other_id: oid,
            });
        }
    }

    term
}

fn conflict_virtual_leader_interaction(
    registry: &ConflictRegistry,
    idm: &IdmSection,
    danger_window: f32,
    v: &Vehicle,
    path_snaps: &[Vec<ConflictSnap>; 8],
    cat: &[Arc<dyn Path>],
    out_active: &mut Vec<ActiveConflictDto>,
) -> f32 {
    let horizon = braking_distance_horizon(v.current_speed, idm);
    let pid = v.path_id;
    let Some(eff) = effective_progress_on_path(v, pid) else {
        return 0.0;
    };
    let mut worst = 0.0_f32;
    for cp in &registry.points {
        let Some((my_t, o_pid, o_t)) = conflict_params_on_path(cp, pid) else {
            continue;
        };
        worst = worst.max(evaluate_conflict_point(
            v,
            cp,
            pid,
            my_t,
            o_pid,
            o_t,
            eff,
            path_snaps,
            cat,
            idm,
            danger_window,
            Some(horizon),
            Some(out_active),
        ));
    }
    worst
}

/// True iff the vehicle may cross the stop line / commit to the full maneuver: every future
/// conflict on the maneuver path is clearable. Already past stop or mid-curve → always true.
fn should_enter_intersection(
    v: &Vehicle,
    path_snaps: &[Vec<ConflictSnap>; 8],
    cat: &[Arc<dyn Path>],
    registry: &ConflictRegistry,
    idm: &IdmSection,
    danger_window: f32,
) -> bool {
    if v.on_bezier_turn && v.path_progress > 1e-4 {
        return true;
    }
    let to_c = v.position - v.conflict_point;
    let past_point = to_c.dot(v.approach_dir) > 0.0;
    if past_stop_line_geom(v, idm.stop_line_offset) || past_point {
        return true;
    }

    for path_pid in maneuver_path_ids(v) {
        let Some(eff) = effective_progress_on_path(v, path_pid) else {
            continue;
        };
        for cp in &registry.points {
            let Some((my_t, o_pid, o_t)) = conflict_params_on_path(cp, path_pid) else {
                continue;
            };
            let t = evaluate_conflict_point(
                v,
                cp,
                path_pid,
                my_t,
                o_pid,
                o_t,
                eff,
                path_snaps,
                cat,
                idm,
                danger_window,
                None,
                None,
            );
            if t > 1e-4 {
                return false;
            }
        }
    }
    true
}

impl SimulationState {
    fn new(cfg: SimulationConfig) -> Self {
        use rand::Rng;
        let ms = cfg.vehicle.max_speed;
        let ar = cfg.intersection.approach_radius;
        let spacing = cfg.spawn.spawn_distance.max(0.2);
        let edge = VIEWPORT_HALF_WORLD;
        let min_axis = min_lane_axis_outside_approach_ring(ar);
        let n = cfg.spawn.num_vehicles.max(1);
        let ltp = cfg.spawn.left_turn_probability;
        let tl = &cfg.traffic_lights;

        let (path_catalog, path_debug, conflict_registry, road_visuals) = path_catalog_with_debug(&cfg);

        let mut rng = rand::thread_rng();
        let mut lane_counts = [0usize; 4];

        let vehicles = (0..n)
            .map(|id| {
                let lane_idx = rng.gen_range(0..4usize);
                let lane = ALL_LANES[lane_idx];
                let rank = lane_counts[lane_idx];
                lane_counts[lane_idx] += 1;
                let dist = (edge - rank as f32 * spacing).max(min_axis);
                let pos = lane_spawn_pos(lane, dist);
                let intent = if rng.gen::<f32>() < ltp {
                    VehicleIntent::Left
                } else {
                    VehicleIntent::Straight
                };

                let pid = straight_path_id(lane);
                let sp = path_catalog[pid as usize].clone();
                let plen = sp.path_length();
                let t0 = spawn_straight_progress(lane, pos);

                let dir = lane_direction(lane);
                Vehicle {
                    id,
                    position: pos,
                    approach_dir: dir,
                    lane_id: lane,
                    conflict_point: Vec2::ZERO,
                    max_speed: ms,
                    current_speed: 0.0,
                    acceleration: 0.0,
                    status: VehicleStatus::Cruising,
                    intent,
                    leader_id: None,
                    oncoming_yield_id: None,
                    current_path: sp,
                    path_id: pid,
                    path_progress: t0,
                    path_length_cached: plen,
                    on_bezier_turn: false,
                }
            })
            .collect();

        Self {
            vehicles,
            lights: TrafficLightManager::new(tl.green_duration, tl.allred_duration),
            cfg,
            path_catalog,
            path_debug,
            conflict_registry,
            active_conflicts: Vec::new(),
            road_visuals,
        }
    }

    fn tick(&mut self, dt: f32) {
        self.lights.tick(dt);
        self.idm_step();
        self.movement_step(dt);
        self.left_turn_step();
        self.remove_finished();
    }

    fn is_finished(&self) -> bool {
        self.vehicles.is_empty()
    }

    fn idm_step(&mut self) {
        self.active_conflicts.clear();
        let idm = self.cfg.idm.clone();
        let ocd = self.cfg.intersection.oncoming_clear_dist;
        let danger = self.cfg.intersection.conflict_danger_window_s;
        let cat = &self.path_catalog;

        let (lane_snaps, path_snaps) = get_grouped_snaps(&self.vehicles);

        for v in &mut self.vehicles {
            let speed = v.current_speed;
            let v0 = v.max_speed;

            // ── Bezier left-turn: free acceleration + oncoming gap first half ───
            if v.on_bezier_turn {
                let p = v.path_progress;
                let free_road = 1.0 - (speed / v0).powf(idm.delta);

                let (oi, ocy_id) = if p < 0.5 {
                    let opp = oncoming_lane(v.lane_id);
                    let closest = lane_snaps[lane_to_index(opp)]
                        .iter()
                        .filter(|snap| snap.id != v.id && snap.position.length() < ocd)
                        .min_by(|a, b| {
                            a.position.length().partial_cmp(&b.position.length()).unwrap_or(std::cmp::Ordering::Equal)
                        });

                    if let Some(snap) = closest {
                        let s = v.position.length().max(f32::EPSILON);
                        let s_star = idm.s0 + speed * idm.t_headway;
                        ((s_star / s).powi(2), Some(snap.id))
                    } else {
                        (0.0, None)
                    }
                } else {
                    (0.0, None)
                };

                let mut interaction = oi;

                let cix = conflict_virtual_leader_interaction(
                    &self.conflict_registry,
                    &self.cfg.idm,
                    danger,
                    v,
                    &path_snaps,
                    cat,
                    &mut self.active_conflicts,
                );
                interaction = interaction.max(cix);

                v.oncoming_yield_id = ocy_id;
                v.leader_id = None;
                v.acceleration = (idm.a_max * (free_road - interaction)).max(-idm.b_comf);
                v.status = if ocy_id.is_some() || cix > 0.02 {
                    VehicleStatus::Yielding
                } else {
                    VehicleStatus::Crossing
                };
                continue;
            }

            let free_road = 1.0 - (speed / v0).powf(idm.delta);

            // Binary search for leader in the sorted lane group
            let lane_idx = lane_to_index(v.lane_id);
            let lane_group = &lane_snaps[lane_idx];
            let v_proj = v.position.dot(lane_direction(v.lane_id));
            let idx = lane_group.partition_point(|snap| snap.proj <= v_proj + 1e-4);
            let leader = lane_group[idx..].iter().find(|snap| snap.id != v.id);

            v.leader_id = leader.map(|snap| snap.id);

            let car_following = if let Some(snap) = leader {
                let gap = snap.proj - v_proj;
                let s = (gap - idm.vehicle_length).max(f32::EPSILON);
                let dv = speed - snap.current_speed;
                let s_star = idm.s0
                    + (speed * idm.t_headway
                        + speed * dv / (2.0 * (idm.a_max * idm.b_comf).sqrt()))
                    .max(0.0);
                (s_star / s).powi(2)
            } else {
                0.0
            };

            let to_conflict = v.position - v.conflict_point;
            let past_point = to_conflict.dot(v.approach_dir) > 0.0;
            let lane_green = self.lights.is_green(v.lane_id);
            let dist_to_stop = to_conflict.length() - idm.stop_line_offset;
            let past_stop_line = dist_to_stop <= 0.0;
            let atomic_ok = should_enter_intersection(
                v,
                &path_snaps,
                cat,
                &self.conflict_registry,
                &self.cfg.idm,
                danger,
            );
            let blocked_at_stop =
                !past_point && !past_stop_line && (!lane_green || (lane_green && !atomic_ok));

            let tl_interaction = if blocked_at_stop {
                let s = dist_to_stop.max(f32::EPSILON);
                let s_star = idm.s0 + speed * idm.t_headway;
                (s_star / s).powi(2)
            } else {
                0.0
            };

            let is_left = v.intent == VehicleIntent::Left;
            let (oncoming_interaction, ocy_id) = if is_left && lane_green && !past_point {
                let opp = oncoming_lane(v.lane_id);
                let closest = lane_snaps[lane_to_index(opp)]
                    .iter()
                    .filter(|snap| snap.id != v.id && snap.position.length() < ocd)
                    .min_by(|a, b| {
                        a.position.length().partial_cmp(&b.position.length()).unwrap_or(std::cmp::Ordering::Equal)
                    });

                if let Some(snap) = closest {
                    let s = v.position.length().max(f32::EPSILON);
                    let s_star = idm.s0 + speed * idm.t_headway;
                    ((s_star / s).powi(2), Some(snap.id))
                } else {
                    (0.0, None)
                }
            } else {
                (0.0, None)
            };

            v.oncoming_yield_id = ocy_id;

            let ix_term = conflict_virtual_leader_interaction(
                &self.conflict_registry,
                &self.cfg.idm,
                danger,
                v,
                &path_snaps,
                cat,
                &mut self.active_conflicts,
            );

            let interaction =
                car_following.max(tl_interaction).max(oncoming_interaction).max(ix_term);
            v.acceleration = (idm.a_max * (free_road - interaction)).max(-idm.b_comf);

            let dist_to_center = v.position.length();
            v.status = if !lane_green && !past_point && !past_stop_line {
                VehicleStatus::Stopped
            } else if ocy_id.is_some() || ix_term > 0.02 {
                VehicleStatus::Yielding
            } else if (past_point || past_stop_line)
                && dist_to_center < self.cfg.intersection.clear_radius * 3.0
            {
                VehicleStatus::Crossing
            } else {
                VehicleStatus::Cruising
            };
        }
    }

    fn movement_step(&mut self, dt: f32) {
        for v in &mut self.vehicles {
            v.current_speed = (v.current_speed + v.acceleration * dt).clamp(0.0, v.max_speed);
            let len = v.path_length_cached.max(1e-6);
            v.path_progress += v.current_speed * dt / len;
            let t_vis = v.path_progress.min(1.0);
            v.position = v.current_path.point(t_vis);
            v.approach_dir = v.current_path.tangent(t_vis);
        }
    }

    fn left_turn_step(&mut self) {
        let stop_line = self.cfg.idm.stop_line_offset;
        let ocd = self.cfg.intersection.oncoming_clear_dist;
        let cat = &self.path_catalog;
        let danger = self.cfg.intersection.conflict_danger_window_s;
        let idm = &self.cfg.idm;

        let (lane_snaps, path_snaps) = get_grouped_snaps(&self.vehicles);

        for v in &mut self.vehicles {
            // ── Finish Bezier: switch to exit-lane straight ───────────────────
            if v.on_bezier_turn && v.path_progress >= 1.0 {
                let new_lane = lane_for_direction(v.approach_dir);
                let new_pid = straight_path_id(new_lane);
                v.current_path = straight_path_for_lane(cat, new_lane);
                v.path_id = new_pid;
                v.path_length_cached = v.current_path.path_length();
                v.path_progress = spawn_straight_progress(new_lane, v.position);
                v.lane_id = new_lane;
                v.intent = VehicleIntent::Straight;
                v.on_bezier_turn = false;
                v.leader_id = None;
                v.oncoming_yield_id = None;
                continue;
            }

            if v.on_bezier_turn {
                continue;
            }

            if v.intent != VehicleIntent::Left {
                continue;
            }

            let dist_from_centre = v.position.length();
            if dist_from_centre > stop_line + 0.05 {
                continue;
            }
            if v.position.dot(v.approach_dir) > 0.1 {
                continue;
            }
            if !self.lights.is_green(v.lane_id) {
                continue;
            }

            let opp = oncoming_lane(v.lane_id);
            let oncoming_blocks = lane_snaps[lane_to_index(opp)]
                .iter()
                .any(|snap| snap.id != v.id && snap.position.length() < ocd);
            if oncoming_blocks {
                continue;
            }

            if !should_enter_intersection(
                v,
                &path_snaps,
                cat,
                &self.conflict_registry,
                idm,
                danger,
            ) {
                continue;
            }

            let lane = v.lane_id;
            v.current_path = template_bezier_for_lane(cat, lane);
            v.path_id = bezier_path_id(lane);
            v.path_length_cached = v.current_path.path_length();
            v.path_progress = 0.0;
            v.position = v.current_path.point(0.0);
            v.approach_dir = v.current_path.tangent(0.0);
            v.on_bezier_turn = true;
        }
    }

    fn remove_finished(&mut self) {
        let offmap = self.cfg.spawn.offmap_distance;
        self.vehicles.retain(|v| {
            if v.on_bezier_turn {
                return true;
            }
            let to_conflict = v.position - v.conflict_point;
            let past_point = to_conflict.dot(v.approach_dir) > 0.0;
            !(past_point && to_conflict.length() > offmap)
        });
    }

    fn to_frame(&self) -> SimFrame {
        SimFrame {
            vehicles: self
                .vehicles
                .iter()
                .map(|v| VehicleFrame {
                    id: v.id,
                    x: v.position.x,
                    y: v.position.y,
                    dir_x: v.approach_dir.x,
                    dir_y: v.approach_dir.y,
                    status: v.status.clone(),
                    intent: v.intent,
                    speed: v.current_speed,
                    accel: v.acceleration,
                    leader_id: v.leader_id,
                    oncoming_yield_id: v.oncoming_yield_id,
                    path_id: v.path_id,
                    path_progress: v.path_progress,
                    on_bezier_turn: v.on_bezier_turn,
                })
                .collect(),
            approach_radius: self.cfg.intersection.approach_radius,
            clear_radius: self.cfg.intersection.clear_radius,
            stop_line_offset: self.cfg.idm.stop_line_offset,
            light_ns: self.lights.phase.ns_green(),
            light_ew: self.lights.phase.ew_green(),
            paths: self.path_debug.clone(),
            conflict_points: self
                .conflict_registry
                .points
                .iter()
                .map(|p| [p.pos.x, p.pos.y])
                .collect(),
            active_conflicts: self.active_conflicts.clone(),
            road_visuals: self.road_visuals.clone(),
        }
    }
}

#[derive(Serialize, Clone)]
struct PathDebug {
    id: u8,
    kind: u8,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
}

#[derive(Serialize, Clone)]
struct VehicleFrame {
    id: usize,
    x: f32,
    y: f32,
    dir_x: f32,
    dir_y: f32,
    status: VehicleStatus,
    intent: VehicleIntent,
    speed: f32,
    accel: f32,
    leader_id: Option<usize>,
    oncoming_yield_id: Option<usize>,
    path_id: u8,
    path_progress: f32,
    on_bezier_turn: bool,
}

#[derive(Serialize, Clone)]
struct SimFrame {
    vehicles: Vec<VehicleFrame>,
    approach_radius: f32,
    clear_radius: f32,
    stop_line_offset: f32,
    light_ns: bool,
    light_ew: bool,
    paths: Vec<PathDebug>,
    conflict_points: Vec<[f32; 2]>,
    active_conflicts: Vec<ActiveConflictDto>,
    road_visuals: Vec<road::RoadVisualStrip>,
}

type SharedState = Arc<Mutex<SimulationState>>;

#[tauri::command]
fn get_simulation_frame(state: tauri::State<SharedState>) -> SimFrame {
    state.lock().unwrap().to_frame()
}

fn start_simulation_loop(app_handle: AppHandle, state: SharedState) {
    thread::spawn(move || {
        let mut last = Instant::now();
        loop {
            let now = Instant::now();
            let dt = now.duration_since(last).as_secs_f32();
            last = now;

            let (frame, finished) = {
                let mut sim = state.lock().unwrap();
                sim.tick(dt);
                let finished = sim.is_finished();
                (sim.to_frame(), finished)
            };

            if let Err(e) = app_handle.emit("sim-frame", &frame) {
                eprintln!("emit error: {e}");
            }

            if finished {
                let _ = app_handle.emit("sim-done", ());
                break;
            }

            let elapsed = last.elapsed();
            if elapsed < FRAME_DURATION {
                thread::sleep(FRAME_DURATION - elapsed);
            }
        }
    });
}

fn run() {
    let cfg = SimulationConfig::load_or_create();
    let sim_state: SharedState = Arc::new(Mutex::new(SimulationState::new(cfg)));

    tauri::Builder::default()
        .manage(sim_state.clone())
        .invoke_handler(tauri::generate_handler![get_simulation_frame])
        .setup(move |app| {
            let handle = app.handle().clone();
            start_simulation_loop(handle, sim_state.clone());
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running Tauri application");
}

fn main() {
    run();
}
