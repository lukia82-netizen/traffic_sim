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

const TARGET_FPS: f64 = 60.0;
const FRAME_DURATION: Duration = Duration::from_nanos((1_000_000_000.0 / TARGET_FPS) as u64);

// ─── Enums ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
enum VehicleStatus {
    Cruising,
    Waiting,
    Crossing,
}

// ─── Simulation data ──────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct Vehicle {
    id: usize,
    position: Vec2,
    approach_dir: Vec2,
    conflict_point: Vec2,
    max_speed: f32,
    current_speed: f32,
    acceleration: f32,
    status: VehicleStatus,
    leader_id: Option<usize>,
}

#[derive(Debug, Clone)]
struct ConflictManager {
    granted: Option<usize>,
}

// ─── Config types ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationSection {
    fixed_dt: f32,
    max_frames: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IntersectionSection {
    approach_radius: f32,
    clear_radius: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct IdmSection {
    a_max: f32,
    b_comf: f32,
    delta: f32,
    s0: f32,
    t_headway: f32,
    stop_line_offset: f32,
    /// Bumper-to-bumper correction: subtracted from center-to-center distance.
    #[serde(default = "default_vehicle_length")]
    vehicle_length: f32,
}

fn default_vehicle_length() -> f32 { 0.2 }

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SpawnSection {
    /// Distance from the conflict point where each vehicle is placed at start.
    #[serde(default = "default_spawn_distance")]
    spawn_distance: f32,
    /// How far past the conflict point before a vehicle is removed.
    #[serde(default = "default_offmap_distance")]
    offmap_distance: f32,
    /// How many vehicles to spawn (1–4), placed clockwise starting from North.
    #[serde(default = "default_num_vehicles")]
    num_vehicles: usize,
}

fn default_spawn_distance() -> f32 { 4.0 }
fn default_offmap_distance() -> f32 { 4.5 }
fn default_num_vehicles() -> usize { 4 }

#[derive(Debug, Clone, Serialize, Deserialize)]
struct VehicleSection {
    max_speed: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SimulationConfig {
    simulation: SimulationSection,
    intersection: IntersectionSection,
    idm: IdmSection,
    spawn: SpawnSection,
    vehicle: VehicleSection,
}

impl Default for SimulationConfig {
    /// Fallback used ONLY when sim_config.toml is missing or unparseable.
    /// Values here must stay in sync with sim_config.toml.
    fn default() -> Self {
        Self {
            simulation: SimulationSection {
                fixed_dt: 0.016,
                max_frames: 0,
            },
            intersection: IntersectionSection {
                approach_radius: 0.5,
                clear_radius: 0.2,
            },
            idm: IdmSection {
                a_max: 1.0,
                b_comf: 1.5,
                delta: 4.0,
                s0: 0.3,
                t_headway: 0.8,
                stop_line_offset: 0.2,
                vehicle_length: 0.2,
            },
            spawn: SpawnSection {
                spawn_distance: 4.0,
                offmap_distance: 4.5,
                num_vehicles: 4,
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
             fixed_dt   = {}    # simulation step in seconds (ignored at runtime — wall-clock used)\n\
             max_frames = {}    # 0 = run indefinitely\n\n\
             [intersection]\n\
             approach_radius = {}   # distance from conflict point where a vehicle requests access\n\
             clear_radius    = {}   # distance past conflict point required to release the grant\n\n\
             [idm]\n\
             a_max            = {}   # max acceleration [m/s²]\n\
             b_comf           = {}   # comfortable braking [m/s²]\n\
             delta            = {}   # acceleration exponent\n\
             s0               = {}   # minimum jam gap at standstill [units]\n\
             t_headway        = {}   # safe time headway [s]\n\
             stop_line_offset = {}   # distance before conflict point where Waiting vehicle targets to stop\n\
             vehicle_length   = {}   # bumper-to-bumper correction subtracted from center-to-center gap [units]\n\n\
             [spawn]\n\
             num_vehicles    = {}   # number of vehicles to spawn: 1-4, clockwise from North\n\
             spawn_distance  = {}   # distance from conflict point where each vehicle spawns [units]\n\
             offmap_distance = {}   # distance past conflict point before vehicle is removed [units]\n\n\
             [vehicle]\n\
             max_speed = {}   # max vehicle speed [units/s]\n",
            self.simulation.fixed_dt,
            self.simulation.max_frames,
            self.intersection.approach_radius,
            self.intersection.clear_radius,
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
            self.vehicle.max_speed,
        )
    }
}

// ─── Simulation state ─────────────────────────────────────────────────────────

struct SimulationState {
    vehicles: Vec<Vehicle>,
    manager: ConflictManager,
    cfg: SimulationConfig,
}

impl SimulationState {
    fn new(cfg: SimulationConfig) -> Self {
        let ms = cfg.vehicle.max_speed;
        let d = cfg.spawn.spawn_distance;
        let n = cfg.spawn.num_vehicles.max(1);

        // Clockwise lane directions: N → E → S → W.
        // approach_dir points toward the conflict point (into the intersection).
        let lane_dirs: [Vec2; 4] = [
            Vec2::new( 0.0, -1.0), // North lane → moves south
            Vec2::new(-1.0,  0.0), // East  lane → moves west
            Vec2::new( 0.0,  1.0), // South lane → moves north
            Vec2::new( 1.0,  0.0), // West  lane → moves east
        ];

        // Vehicle id=k → lane k%4, rank k/4.
        // Rank 0 spawns at spawn_distance, rank 1 at 2*spawn_distance, etc.
        // Spawn position = -approach_dir * spawn_distance * (rank + 1).
        let vehicles = (0..n)
            .map(|id| {
                let dir = lane_dirs[id % 4];
                let rank = (id / 4) as f32;
                let pos = -dir * d * (rank + 1.0);
                Vehicle {
                    id,
                    position: pos,
                    approach_dir: dir,
                    conflict_point: Vec2::ZERO,
                    max_speed: ms,
                    current_speed: 0.0,
                    acceleration: 0.0,
                    status: VehicleStatus::Cruising,
                    leader_id: None,
                }
            })
            .collect();
        Self {
            vehicles,
            manager: ConflictManager { granted: None },
            cfg,
        }
    }

    fn tick(&mut self, dt: f32) {
        self.intersection_step();
        self.idm_step();
        self.movement_step(dt);
        self.remove_finished();
    }

    /// Returns true when all vehicles have left the map.
    fn is_finished(&self) -> bool {
        self.vehicles.is_empty()
    }

    // ── Conflict arbiter ──────────────────────────────────────────────────────

    fn intersection_step(&mut self) {
        // Step A: release grant if crossing vehicle has cleared the zone.
        if let Some(granted_id) = self.manager.granted {
            if let Some(v) = self.vehicles.iter_mut().find(|v| v.id == granted_id) {
                let to_conflict = v.position - v.conflict_point;
                let past_point = to_conflict.dot(v.approach_dir) > 0.0;
                let far_enough = to_conflict.length() > self.cfg.intersection.clear_radius;
                if past_point && far_enough {
                    v.status = VehicleStatus::Cruising;
                    self.manager.granted = None;
                }
            } else {
                self.manager.granted = None;
            }
        }

        // Step B: collect approaching (not-yet-past) vehicles sorted by id.
        let approach_radius = self.cfg.intersection.approach_radius;
        let mut approaching: Vec<usize> = self
            .vehicles
            .iter()
            .filter_map(|v| {
                let to_conflict = v.position - v.conflict_point;
                let past_point = to_conflict.dot(v.approach_dir) > 0.0;
                let dist = to_conflict.length();
                if dist < approach_radius && !past_point {
                    Some(v.id)
                } else {
                    None
                }
            })
            .collect();
        approaching.sort_unstable();

        // Step C: lowest id wins the grant.
        if self.manager.granted.is_none() {
            if let Some(&winner_id) = approaching.first() {
                self.manager.granted = Some(winner_id);
                if let Some(v) = self.vehicles.iter_mut().find(|v| v.id == winner_id) {
                    v.status = VehicleStatus::Crossing;
                }
            }
        }

        // Step D: all other approaching vehicles wait.
        let granted = self.manager.granted;
        for v in &mut self.vehicles {
            if approaching.contains(&v.id) && Some(v.id) != granted {
                v.status = VehicleStatus::Waiting;
            }
        }
    }

    // ── IDM acceleration ──────────────────────────────────────────────────────

    fn idm_step(&mut self) {
        let idm = self.cfg.idm.clone();

        // Snapshot: (id, position, approach_dir, current_speed).
        let snapshot: Vec<(usize, Vec2, Vec2, f32)> = self
            .vehicles
            .iter()
            .map(|v| (v.id, v.position, v.approach_dir, v.current_speed))
            .collect();

        for v in &mut self.vehicles {
            let speed = v.current_speed;
            let v0 = v.max_speed;

            // ── Free-road term ────────────────────────────────────────────────
            let free_road = 1.0 - (speed / v0).powf(idm.delta);

            // ── Leader search: same lane, closest vehicle ahead ───────────────
            let leader = snapshot
                .iter()
                .filter(|(id, _, dir, _)| {
                    *id != v.id && dir.dot(v.approach_dir) > 0.99
                })
                .filter_map(|(lid, pos, _, leader_speed)| {
                    let to_leader = *pos - v.position;
                    let proj = to_leader.dot(v.approach_dir);
                    if proj > 0.0 {
                        Some((*lid, proj, *leader_speed))
                    } else {
                        None
                    }
                })
                .min_by(|(_, a, _), (_, b, _)| a.partial_cmp(b).unwrap());

            // Store the leader id for debug rendering.
            v.leader_id = leader.map(|(lid, _, _)| lid);

            // ── Car-following interaction term ────────────────────────────────
            let car_following = if let Some((_, center_gap, leader_speed)) = leader {
                let s = (center_gap - idm.vehicle_length).max(f32::EPSILON);
                let delta_v = speed - leader_speed;
                let s_star = idm.s0
                    + (speed * idm.t_headway
                        + speed * delta_v / (2.0 * (idm.a_max * idm.b_comf).sqrt()))
                    .max(0.0);
                (s_star / s).powi(2)
            } else {
                0.0
            };

            // ── Intersection interaction term ─────────────────────────────────
            // Treat the stop line as a virtual stationary leader for Waiting
            // vehicles that have not yet crossed the conflict point.
            let to_conflict = v.position - v.conflict_point;
            let past_point = to_conflict.dot(v.approach_dir) > 0.0;

            let intersection = if v.status == VehicleStatus::Waiting && !past_point {
                let s = (to_conflict.length() - idm.stop_line_offset).max(f32::EPSILON);
                let s_star = idm.s0 + speed * idm.t_headway;
                (s_star / s).powi(2)
            } else {
                0.0
            };

            // Worst-case (most braking) wins.
            let interaction = car_following.max(intersection);
            v.acceleration = (idm.a_max * (free_road - interaction)).max(-idm.b_comf);
        }
    }

    // ── Euler integration ─────────────────────────────────────────────────────

    fn movement_step(&mut self, dt: f32) {
        for v in &mut self.vehicles {
            v.current_speed = (v.current_speed + v.acceleration * dt).clamp(0.0, v.max_speed);
            v.position += v.approach_dir * v.current_speed * dt;
        }
    }

    // ── Remove vehicles that have driven off the map ──────────────────────────

    fn remove_finished(&mut self) {
        let offmap = self.cfg.spawn.offmap_distance;
        self.vehicles.retain(|v| {
            let to_conflict = v.position - v.conflict_point;
            let past_point = to_conflict.dot(v.approach_dir) > 0.0;
            !(past_point && to_conflict.length() > offmap)
        });
        // If the removed vehicle held the grant, clear it.
        if let Some(id) = self.manager.granted {
            if !self.vehicles.iter().any(|v| v.id == id) {
                self.manager.granted = None;
            }
        }
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
                    speed: v.current_speed,
                    accel: v.acceleration,
                    leader_id: v.leader_id,
                })
                .collect(),
            approach_radius: self.cfg.intersection.approach_radius,
            clear_radius: self.cfg.intersection.clear_radius,
        }
    }
}

// ─── Tauri payload types ──────────────────────────────────────────────────────

#[derive(Serialize, Clone)]
struct VehicleFrame {
    id: usize,
    x: f32,
    y: f32,
    dir_x: f32,
    dir_y: f32,
    status: VehicleStatus,
    speed: f32,
    accel: f32,
    leader_id: Option<usize>,
}

#[derive(Serialize, Clone)]
struct SimFrame {
    vehicles: Vec<VehicleFrame>,
    approach_radius: f32,
    clear_radius: f32,
}

// ─── Tauri commands ───────────────────────────────────────────────────────────

type SharedState = Arc<Mutex<SimulationState>>;

/// Returns the current simulation frame on demand (optional polling path).
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

            // Sleep for the remainder of the frame budget.
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
