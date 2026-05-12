// Prevents an additional console window on Windows in release builds.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use glam::Vec2;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path as StdPath;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use tauri::{AppHandle, Emitter};

// ─── Constants ────────────────────────────────────────────────────────────────

const TARGET_FPS:       f64      = 60.0;
const FRAME_DURATION:   Duration = Duration::from_nanos((1_000_000_000.0 / TARGET_FPS) as u64);
const LANE_OFFSET:      f32      = 0.2;  // half-width of lane from road centre-line [world units]
const TURN_THRESHOLD:   f32      = 0.15; // distance from centre that triggers left-turn transform

// ─── Enums ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
enum VehicleStatus {
    Cruising,
    Stopped,  // held at red light
    Yielding, // left-turner waiting for oncoming gap
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

/// Traffic-light phase — cycles NS-green → all-red → EW-green → all-red → …
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
    fn ns_green(self) -> bool { matches!(self, LightPhase::NSGreen) }
    fn ew_green(self) -> bool { matches!(self, LightPhase::EWGreen) }
}

// ─── Lane helpers ─────────────────────────────────────────────────────────────

const ALL_LANES: [LaneId; 4] = [LaneId::North, LaneId::South, LaneId::East, LaneId::West];

fn lane_direction(lane: LaneId) -> Vec2 {
    match lane {
        LaneId::North => Vec2::new( 0.0, -1.0), // southbound
        LaneId::South => Vec2::new( 0.0,  1.0), // northbound
        LaneId::East  => Vec2::new(-1.0,  0.0), // westbound
        LaneId::West  => Vec2::new( 1.0,  0.0), // eastbound
    }
}

fn lane_spawn_pos(lane: LaneId, dist: f32) -> Vec2 {
    match lane {
        LaneId::North => Vec2::new( LANE_OFFSET,  dist),
        LaneId::South => Vec2::new(-LANE_OFFSET, -dist),
        LaneId::East  => Vec2::new( dist, -LANE_OFFSET),
        LaneId::West  => Vec2::new(-dist,  LANE_OFFSET),
    }
}

/// The lane coming from the opposite direction on the same road (oncoming traffic).
fn oncoming_lane(lane: LaneId) -> LaneId {
    match lane {
        LaneId::North => LaneId::South,
        LaneId::South => LaneId::North,
        LaneId::East  => LaneId::West,
        LaneId::West  => LaneId::East,
    }
}

/// Map a normalised direction vector back to a LaneId.
fn lane_for_direction(dir: Vec2) -> LaneId {
    if      dir.x >  0.5 { LaneId::West  }
    else if dir.x < -0.5 { LaneId::East  }
    else if dir.y >  0.5 { LaneId::South }
    else                 { LaneId::North }
}

// ─── Simulation data ──────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct Vehicle {
    id:                usize,
    position:          Vec2,
    approach_dir:      Vec2,
    lane_id:           LaneId,
    conflict_point:    Vec2,
    max_speed:         f32,
    current_speed:     f32,
    acceleration:      f32,
    status:            VehicleStatus,
    intent:            VehicleIntent,
    leader_id:         Option<usize>,
    oncoming_yield_id: Option<usize>, // set when left-turner is waiting for an oncoming gap
}

// ─── Traffic Light Manager ────────────────────────────────────────────────────

struct TrafficLightManager {
    phase:           LightPhase,
    elapsed:         f32,
    green_duration:  f32,
    allred_duration: f32,
}

impl TrafficLightManager {
    fn new(green_duration: f32, allred_duration: f32) -> Self {
        Self { phase: LightPhase::NSGreen, elapsed: 0.0, green_duration, allred_duration }
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
            LaneId::East  | LaneId::West  => self.phase.ew_green(),
        }
    }
}

// ─── Config types ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationSection {
    fixed_dt:   f32,
    max_frames: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IntersectionSection {
    approach_radius: f32,
    clear_radius:    f32,
    /// Oncoming vehicle within this distance from centre blocks a left-turn.
    #[serde(default = "default_oncoming_clear_dist")]
    oncoming_clear_dist: f32,
}
fn default_oncoming_clear_dist() -> f32 { 3.0 }

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TrafficLightSection {
    #[serde(default = "default_green_duration")]
    green_duration: f32,
    #[serde(default = "default_allred_duration")]
    allred_duration: f32,
}
fn default_green_duration()  -> f32 { 8.0 }
fn default_allred_duration() -> f32 { 2.0 }

impl Default for TrafficLightSection {
    fn default() -> Self {
        Self { green_duration: default_green_duration(), allred_duration: default_allred_duration() }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IdmSection {
    a_max:            f32,
    b_comf:           f32,
    delta:            f32,
    s0:               f32,
    t_headway:        f32,
    stop_line_offset: f32,
    #[serde(default = "default_vehicle_length")]
    vehicle_length: f32,
}
fn default_vehicle_length() -> f32 { 0.2 }

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SpawnSection {
    #[serde(default = "default_spawn_distance")]
    spawn_distance: f32,
    #[serde(default = "default_offmap_distance")]
    offmap_distance: f32,
    #[serde(default = "default_num_vehicles")]
    num_vehicles: usize,
    /// Fraction of spawned vehicles assigned a left-turn intent [0.0 – 1.0].
    #[serde(default = "default_left_turn_probability")]
    left_turn_probability: f32,
}
fn default_spawn_distance()        -> f32   { 4.0 }
fn default_offmap_distance()       -> f32   { 4.5 }
fn default_num_vehicles()          -> usize { 4 }
fn default_left_turn_probability() -> f32   { 0.20 }

#[derive(Debug, Clone, Serialize, Deserialize)]
struct VehicleSection {
    max_speed: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationConfig {
    simulation:     SimulationSection,
    intersection:   IntersectionSection,
    #[serde(default)]
    traffic_lights: TrafficLightSection,
    idm:            IdmSection,
    spawn:          SpawnSection,
    vehicle:        VehicleSection,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            simulation:   SimulationSection { fixed_dt: 0.016, max_frames: 0 },
            intersection: IntersectionSection {
                approach_radius:     4.0,
                clear_radius:        0.8,
                oncoming_clear_dist: 3.0,
            },
            traffic_lights: TrafficLightSection::default(),
            idm: IdmSection {
                a_max: 1.5, b_comf: 3.0, delta: 4.0,
                s0: 0.6, t_headway: 0.8,
                stop_line_offset: 1.2, vehicle_length: 0.4,
            },
            spawn: SpawnSection {
                spawn_distance: 5.0, offmap_distance: 6.0,
                num_vehicles: 16, left_turn_probability: 0.20,
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
             oncoming_clear_dist = {}   # oncoming vehicle within this dist blocks a left-turn [units]\n\n\
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
             spawn_distance         = {}   # spawn distance from centre [units]\n\
             offmap_distance        = {}   # removal distance past centre [units]\n\
             left_turn_probability  = {}   # fraction of vehicles with left-turn intent [0–1]\n\n\
             [vehicle]\n\
             max_speed = {}   # max speed [units/s]\n",
            self.simulation.fixed_dt, self.simulation.max_frames,
            self.intersection.approach_radius, self.intersection.clear_radius,
            self.intersection.oncoming_clear_dist,
            self.traffic_lights.green_duration, self.traffic_lights.allred_duration,
            self.idm.a_max, self.idm.b_comf, self.idm.delta,
            self.idm.s0, self.idm.t_headway,
            self.idm.stop_line_offset, self.idm.vehicle_length,
            self.spawn.num_vehicles, self.spawn.spawn_distance,
            self.spawn.offmap_distance, self.spawn.left_turn_probability,
            self.vehicle.max_speed,
        )
    }
}

// ─── Simulation state ─────────────────────────────────────────────────────────

struct SimulationState {
    vehicles: Vec<Vehicle>,
    lights:   TrafficLightManager,
    cfg:      SimulationConfig,
}

impl SimulationState {
    fn new(cfg: SimulationConfig) -> Self {
        use rand::Rng;
        let ms  = cfg.vehicle.max_speed;
        let d   = cfg.spawn.spawn_distance;
        let n   = cfg.spawn.num_vehicles.max(1);
        let ltp = cfg.spawn.left_turn_probability;
        let tl  = &cfg.traffic_lights;

        let mut rng         = rand::thread_rng();
        let mut lane_counts = [0usize; 4];

        let vehicles = (0..n)
            .map(|id| {
                let lane_idx = rng.gen_range(0..4usize);
                let lane     = ALL_LANES[lane_idx];
                let rank     = lane_counts[lane_idx];
                lane_counts[lane_idx] += 1;
                let dir    = lane_direction(lane);
                let pos    = lane_spawn_pos(lane, d * (rank as f32 + 1.0));
                let intent = if rng.gen::<f32>() < ltp {
                    VehicleIntent::Left
                } else {
                    VehicleIntent::Straight
                };
                Vehicle {
                    id,
                    position:          pos,
                    approach_dir:      dir,
                    lane_id:           lane,
                    conflict_point:    Vec2::ZERO,
                    max_speed:         ms,
                    current_speed:     0.0,
                    acceleration:      0.0,
                    status:            VehicleStatus::Cruising,
                    intent,
                    leader_id:         None,
                    oncoming_yield_id: None,
                }
            })
            .collect();

        Self {
            vehicles,
            lights: TrafficLightManager::new(tl.green_duration, tl.allred_duration),
            cfg,
        }
    }

    fn tick(&mut self, dt: f32) {
        self.lights.tick(dt);
        self.idm_step();
        self.movement_step(dt);
        self.left_turn_step();
        self.remove_finished();
    }

    fn is_finished(&self) -> bool { self.vehicles.is_empty() }

    // ── IDM acceleration (traffic lights + car-following + left-turn oncoming) ─

    fn idm_step(&mut self) {
        let idm  = self.cfg.idm.clone();
        let ocd  = self.cfg.intersection.oncoming_clear_dist;

        // Snapshot: (id, position, approach_dir, speed, lane_id).
        let snap: Vec<(usize, Vec2, Vec2, f32, LaneId)> = self
            .vehicles
            .iter()
            .map(|v| (v.id, v.position, v.approach_dir, v.current_speed, v.lane_id))
            .collect();

        for v in &mut self.vehicles {
            let speed = v.current_speed;
            let v0    = v.max_speed;

            // ── Free-road term ────────────────────────────────────────────────
            let free_road = 1.0 - (speed / v0).powf(idm.delta);

            // ── Car-following: same lane only ─────────────────────────────────
            let leader = snap
                .iter()
                .filter(|(id, _, _, _, lane)| *id != v.id && *lane == v.lane_id)
                .filter_map(|(lid, pos, _, ls, _)| {
                    let proj = (*pos - v.position).dot(v.approach_dir);
                    if proj > 0.0 { Some((*lid, proj, *ls)) } else { None }
                })
                .min_by(|(_, a, _), (_, b, _)| a.partial_cmp(b).unwrap());

            v.leader_id = leader.map(|(lid, _, _)| lid);

            let car_following = if let Some((_, gap, ls)) = leader {
                let s      = (gap - idm.vehicle_length).max(f32::EPSILON);
                let dv     = speed - ls;
                let s_star = idm.s0
                    + (speed * idm.t_headway
                        + speed * dv / (2.0 * (idm.a_max * idm.b_comf).sqrt()))
                    .max(0.0);
                (s_star / s).powi(2)
            } else {
                0.0
            };

            // ── Traffic-light stop-line term ──────────────────────────────────
            let to_conflict    = v.position - v.conflict_point;
            let past_point     = to_conflict.dot(v.approach_dir) > 0.0;
            let lane_green     = self.lights.is_green(v.lane_id);
            let dist_to_stop   = to_conflict.length() - idm.stop_line_offset;
            let past_stop_line = dist_to_stop <= 0.0;

            // Only brake at stop line if approaching on red and not yet past the line.
            let tl_interaction = if !lane_green && !past_point && !past_stop_line {
                let s      = dist_to_stop.max(f32::EPSILON);
                let s_star = idm.s0 + speed * idm.t_headway;
                (s_star / s).powi(2)
            } else {
                0.0
            };

            // ── Left-turn oncoming yield ──────────────────────────────────────
            // A left-turner on GREEN yields to any oncoming vehicle within
            // `oncoming_clear_dist` of centre by braking toward the centre.
            // Once past centre the turn is already committed → no more check.
            let is_left = v.intent == VehicleIntent::Left;
            let (oncoming_interaction, ocy_id) = if is_left && lane_green && !past_point {
                let opp     = oncoming_lane(v.lane_id);
                let closest = snap
                    .iter()
                    .filter(|(id, pos, _, _, lane)| {
                        *id != v.id && *lane == opp && pos.length() < ocd
                    })
                    .min_by(|(_, a, ..), (_, b, ..)| {
                        a.length().partial_cmp(&b.length()).unwrap()
                    });

                if let Some((oc_id, _, ..)) = closest {
                    let s      = v.position.length().max(f32::EPSILON);
                    let s_star = idm.s0 + speed * idm.t_headway;
                    ((s_star / s).powi(2), Some(*oc_id))
                } else {
                    (0.0, None)
                }
            } else {
                (0.0, None)
            };

            v.oncoming_yield_id = ocy_id;

            // Worst-case interaction wins.
            let interaction = car_following.max(tl_interaction).max(oncoming_interaction);
            v.acceleration  = (idm.a_max * (free_road - interaction)).max(-idm.b_comf);

            // ── Status ────────────────────────────────────────────────────────
            let dist_to_center = v.position.length();
            v.status = if !lane_green && !past_point && !past_stop_line {
                VehicleStatus::Stopped
            } else if ocy_id.is_some() {
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

    // ── Euler integration ─────────────────────────────────────────────────────

    fn movement_step(&mut self, dt: f32) {
        for v in &mut self.vehicles {
            v.current_speed = (v.current_speed + v.acceleration * dt).clamp(0.0, v.max_speed);
            v.position += v.approach_dir * v.current_speed * dt;
        }
    }

    // ── Left-turn path transformation ─────────────────────────────────────────
    // Once a left-turning vehicle reaches the centre (within TURN_THRESHOLD),
    // rotate its direction 90° CCW, snap to the new lane axis, update lane_id.

    fn left_turn_step(&mut self) {
        for v in &mut self.vehicles {
            if v.intent != VehicleIntent::Left { continue; }
            if v.position.length() > TURN_THRESHOLD { continue; }

            let old_dir = v.approach_dir;
            // CCW 90°: (x,y) → (-y, x)
            let new_dir = Vec2::new(-old_dir.y, old_dir.x);
            // Progress the vehicle has made along the NEW direction.
            let along_new = v.position.dot(new_dir) * new_dir;
            // Lane centre offset for the new road = -old_dir * LANE_OFFSET
            // (verified for all four turning cases in the design notes).
            v.position    = along_new + (-old_dir) * LANE_OFFSET;
            v.approach_dir = new_dir;
            v.lane_id      = lane_for_direction(new_dir);
            v.intent       = VehicleIntent::Straight;
            v.leader_id    = None;
            v.oncoming_yield_id = None;
        }
    }

    // ── Remove vehicles that have driven off the map ──────────────────────────

    fn remove_finished(&mut self) {
        let offmap = self.cfg.spawn.offmap_distance;
        self.vehicles.retain(|v| {
            let to_conflict = v.position - v.conflict_point;
            let past_point  = to_conflict.dot(v.approach_dir) > 0.0;
            !(past_point && to_conflict.length() > offmap)
        });
    }

    fn to_frame(&self) -> SimFrame {
        SimFrame {
            vehicles: self
                .vehicles
                .iter()
                .map(|v| VehicleFrame {
                    id:                v.id,
                    x:                 v.position.x,
                    y:                 v.position.y,
                    dir_x:             v.approach_dir.x,
                    dir_y:             v.approach_dir.y,
                    status:            v.status.clone(),
                    intent:            v.intent,
                    speed:             v.current_speed,
                    accel:             v.acceleration,
                    leader_id:         v.leader_id,
                    oncoming_yield_id: v.oncoming_yield_id,
                })
                .collect(),
            approach_radius:  self.cfg.intersection.approach_radius,
            clear_radius:     self.cfg.intersection.clear_radius,
            stop_line_offset: self.cfg.idm.stop_line_offset,
            light_ns:         self.lights.phase.ns_green(),
            light_ew:         self.lights.phase.ew_green(),
        }
    }
}

// ─── Tauri payload types ──────────────────────────────────────────────────────

#[derive(Serialize, Clone)]
struct VehicleFrame {
    id:                usize,
    x:                 f32,
    y:                 f32,
    dir_x:             f32,
    dir_y:             f32,
    status:            VehicleStatus,
    intent:            VehicleIntent,
    speed:             f32,
    accel:             f32,
    leader_id:         Option<usize>,
    oncoming_yield_id: Option<usize>,
}

#[derive(Serialize, Clone)]
struct SimFrame {
    vehicles:         Vec<VehicleFrame>,
    approach_radius:  f32,
    clear_radius:     f32,
    stop_line_offset: f32,
    light_ns:         bool,
    light_ew:         bool,
}

// ─── Tauri commands ───────────────────────────────────────────────────────────

type SharedState = Arc<Mutex<SimulationState>>;

#[tauri::command]
fn get_simulation_frame(state: tauri::State<SharedState>) -> SimFrame {
    state.lock().unwrap().to_frame()
}

// ─── Simulation thread ────────────────────────────────────────────────────────

fn start_simulation_loop(app_handle: AppHandle, state: SharedState) {
    thread::spawn(move || {
        let mut last = Instant::now();
        loop {
            let now = Instant::now();
            let dt  = now.duration_since(last).as_secs_f32();
            last    = now;

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

// ─── Entry point ──────────────────────────────────────────────────────────────

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
