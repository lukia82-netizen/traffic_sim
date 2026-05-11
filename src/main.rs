use bevy::app::AppExit;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path as StdPath;

// ─── Enums ────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq)]
enum VehicleStatus {
    Cruising,
    Waiting,
    Crossing,
}

// ─── Components ───────────────────────────────────────────────────────────────

#[derive(Component)]
struct Position(Vec2);

#[derive(Component)]
struct Vehicle {
    max_speed: f32,
    status: VehicleStatus,
    current_speed: f32,
    acceleration: f32,
}

/// Describes a vehicle's relationship to a single conflict point.
#[derive(Component)]
struct Path {
    conflict_point: Vec2,
    /// Normalized direction of travel (spawn → conflict point and beyond).
    approach_dir: Vec2,
}

#[derive(Component)]
struct Intersection {
    conflict_point: Vec2,
}

// ─── Resources ────────────────────────────────────────────────────────────────

/// The central arbiter: at most one entity may occupy the conflict point at a time.
#[derive(Resource)]
struct ConflictManager {
    granted: Option<Entity>,
}

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
struct SimulationConfig {
    simulation: SimulationSection,
    intersection: IntersectionSection,
    spawn: SpawnSection,
    vehicle: VehicleSection,
    idm: IdmSection,
}

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
    /// Max acceleration (m/s²).
    a_max: f32,
    /// Comfortable braking deceleration — clamps minimum acceleration (m/s²).
    b_comf: f32,
    /// Acceleration exponent.
    delta: f32,
    /// Minimum jam gap (distance kept at standstill).
    s0: f32,
    /// Safe time headway (s).
    t_headway: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SpawnSection {
    north_spawn_y: f32,
    east_spawn_x: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct VehicleSection {
    max_speed: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            simulation: SimulationSection {
                fixed_dt: 0.016,
                max_frames: 300,
            },
            intersection: IntersectionSection {
                approach_radius: 2.0,
                clear_radius: 1.0,
            },
            spawn: SpawnSection {
                north_spawn_y: 4.0,
                east_spawn_x: 4.0,
            },
            vehicle: VehicleSection { max_speed: 1.0 },
            idm: IdmSection {
                a_max: 1.5,
                b_comf: 2.0,
                delta: 4.0,
                s0: 1.0,
                t_headway: 1.5,
            },
        }
    }
}

impl SimulationConfig {
    const FILE_PATH: &'static str = "sim_config.toml";

    fn load_or_create() -> Self {
        let path = StdPath::new(Self::FILE_PATH);
        if !path.exists() {
            let defaults = Self::default();
            if let Err(err) = fs::write(path, defaults.to_toml_text()) {
                eprintln!(
                    "Could not create {} ({}). Using in-memory defaults.",
                    Self::FILE_PATH,
                    err
                );
            }
            return defaults;
        }

        match fs::read_to_string(path) {
            Ok(content) => match Self::from_toml_text(&content) {
                Ok(cfg) => cfg,
                Err(err) => {
                    eprintln!(
                        "Config parse error in {}: {}. Falling back to defaults.",
                        Self::FILE_PATH,
                        err
                    );
                    Self::default()
                }
            },
            Err(err) => {
                eprintln!(
                    "Could not read {} ({}). Falling back to defaults.",
                    Self::FILE_PATH,
                    err
                );
                Self::default()
            }
        }
    }

    fn from_toml_text(input: &str) -> Result<Self, String> {
        let cfg: Self = toml::from_str(input).map_err(|err| err.to_string())?;

        if cfg.simulation.fixed_dt <= 0.0 {
            return Err("fixed_dt must be > 0".to_string());
        }
        if cfg.intersection.approach_radius <= 0.0 {
            return Err("approach_radius must be > 0".to_string());
        }
        if cfg.intersection.clear_radius < 0.0 {
            return Err("clear_radius must be >= 0".to_string());
        }
        if cfg.vehicle.max_speed <= 0.0 {
            return Err("vehicle_max_speed must be > 0".to_string());
        }
        if cfg.idm.a_max <= 0.0 {
            return Err("idm.a_max must be > 0".to_string());
        }
        if cfg.idm.b_comf <= 0.0 {
            return Err("idm.b_comf must be > 0".to_string());
        }
        if cfg.idm.delta <= 0.0 {
            return Err("idm.delta must be > 0".to_string());
        }
        if cfg.idm.s0 <= 0.0 {
            return Err("idm.s0 must be > 0".to_string());
        }
        if cfg.idm.t_headway <= 0.0 {
            return Err("idm.t_headway must be > 0".to_string());
        }

        Ok(cfg)
    }

    fn to_toml_text(&self) -> String {
        format!(
            "# Runtime config for traffic_sim\n\
# Edit values and rerun the app (no rebuild needed).\n\n\
[simulation]\n\
# Fixed simulation step in seconds.\n\
fixed_dt = {}\n\
# Number of frames before graceful shutdown.\n\
max_frames = {}\n\n\
[intersection]\n\
# Distance from conflict point where a vehicle requests access.\n\
approach_radius = {}\n\
# Distance after crossing required to release intersection grant.\n\
clear_radius = {}\n\n\
[spawn]\n\
# Initial Y for north vehicle (moves toward negative Y).\n\
north_spawn_y = {}\n\
# Initial X for east vehicle (moves toward negative X).\n\
east_spawn_x = {}\n\n\
[vehicle]\n\
# Max vehicle speed units per second.\n\
max_speed = {}\n\n\
[idm]\n\
# Maximum acceleration (m/s²).\n\
a_max = {}\n\
# Comfortable braking deceleration — clamps minimum acceleration (m/s²).\n\
b_comf = {}\n\
# Acceleration exponent.\n\
delta = {}\n\
# Minimum jam gap kept at standstill.\n\
s0 = {}\n\
# Safe time headway (s).\n\
t_headway = {}\n",
            self.simulation.fixed_dt,
            self.simulation.max_frames,
            self.intersection.approach_radius,
            self.intersection.clear_radius,
            self.spawn.north_spawn_y,
            self.spawn.east_spawn_x,
            self.vehicle.max_speed,
            self.idm.a_max,
            self.idm.b_comf,
            self.idm.delta,
            self.idm.s0,
            self.idm.t_headway,
        )
    }
}

// ─── Startup System ───────────────────────────────────────────────────────────

fn startup(mut commands: Commands, cfg: Res<SimulationConfig>) {
    // Intersection anchor at (0, 0).
    commands.spawn(Intersection {
        conflict_point: Vec2::ZERO,
    });

    // North vehicle: travelling south along the Y axis.
    commands.spawn((
        Position(Vec2::new(0.0, cfg.spawn.north_spawn_y)),
        Vehicle {
            max_speed: cfg.vehicle.max_speed,
            status: VehicleStatus::Cruising,
            current_speed: 0.0,
            acceleration: 0.0,
        },
        Path {
            conflict_point: Vec2::ZERO,
            approach_dir: Vec2::new(0.0, -1.0),
        },
    ));

    // East vehicle: travelling west along the X axis.
    commands.spawn((
        Position(Vec2::new(cfg.spawn.east_spawn_x, 0.0)),
        Vehicle {
            max_speed: cfg.vehicle.max_speed,
            status: VehicleStatus::Cruising,
            current_speed: 0.0,
            acceleration: 0.0,
        },
        Path {
            conflict_point: Vec2::ZERO,
            approach_dir: Vec2::new(-1.0, 0.0),
        },
    ));
}

// ─── Intersection System (The Arbiter) ────────────────────────────────────────

fn intersection_system(
    mut mgr: ResMut<ConflictManager>,
    cfg: Res<SimulationConfig>,
    mut q: Query<(Entity, &Position, &mut Vehicle, &Path)>,
) {
    // ── Step A: release grant if the crossing vehicle has cleared the zone ──
    if let Some(granted_entity) = mgr.granted {
        if let Ok((_, pos, mut vehicle, path)) = q.get_mut(granted_entity) {
            let to_conflict = pos.0 - path.conflict_point;
            let past_point = to_conflict.dot(path.approach_dir) > 0.0;
            let far_enough = to_conflict.length() > cfg.intersection.clear_radius;
            if past_point && far_enough {
                vehicle.status = VehicleStatus::Cruising;
                mgr.granted = None;
            }
        } else {
            // Entity was despawned; release unconditionally.
            mgr.granted = None;
        }
    }

    // ── Step B: collect entities approaching the conflict point ──
    let mut approaching: Vec<(Entity, u32)> = q
        .iter()
        .filter_map(|(entity, pos, _, path)| {
            let to_conflict = pos.0 - path.conflict_point;
            let past_point = to_conflict.dot(path.approach_dir) > 0.0;
            let dist = to_conflict.length();
            if dist < cfg.intersection.approach_radius && !past_point {
                Some((entity, entity.index_u32()))
            } else {
                None
            }
        })
        .collect();

    // ── Step C: grant access to the vehicle with the lowest entity index ──
    if mgr.granted.is_none() && !approaching.is_empty() {
        // Deterministic tie-breaker: lowest raw entity index wins.
        approaching.sort_by_key(|&(_, idx)| idx);
        let (winner, _) = approaching[0];
        mgr.granted = Some(winner);
        if let Ok((_, _, mut vehicle, _)) = q.get_mut(winner) {
            vehicle.status = VehicleStatus::Crossing;
        }
    }

    // ── Step D: all other approaching vehicles must wait ──
    for (entity, _) in &approaching {
        if Some(*entity) != mgr.granted {
            if let Ok((_, _, mut vehicle, _)) = q.get_mut(*entity) {
                vehicle.status = VehicleStatus::Waiting;
            }
        }
    }
}

// ─── IDM System ───────────────────────────────────────────────────────────────

fn idm_system(cfg: Res<SimulationConfig>, mut q: Query<(&Position, &mut Vehicle, &Path)>) {
    for (pos, mut vehicle, path) in &mut q {
        let v = vehicle.current_speed;
        let v0 = vehicle.max_speed;
        let idm = &cfg.idm;

        // Dot-product guard: a vehicle past the conflict point must not apply
        // the interaction term (it has already crossed and should accelerate freely).
        let to_conflict = pos.0 - path.conflict_point;
        let past_point = to_conflict.dot(path.approach_dir) > 0.0;

        // Free-road term: accelerate toward v0.
        let free_road = 1.0 - (v / v0).powf(idm.delta);

        // Interaction term: only for Waiting vehicles still approaching.
        let interaction = if vehicle.status == VehicleStatus::Waiting && !past_point {
            let s = to_conflict.length().max(f32::EPSILON);
            let s_star = idm.s0 + v * idm.t_headway;
            (s_star / s).powi(2)
        } else {
            0.0
        };

        // Clamp deceleration floor to -b_comf to avoid numerical blow-up.
        vehicle.acceleration = (idm.a_max * (free_road - interaction)).max(-idm.b_comf);
    }
}

// ─── Movement System ──────────────────────────────────────────────────────────

fn movement_system(time: Res<Time>, mut q: Query<(&mut Position, &mut Vehicle, &Path)>) {
    let dt = time.delta_secs();
    for (mut pos, mut vehicle, path) in &mut q {
        vehicle.current_speed = (vehicle.current_speed + vehicle.acceleration * dt)
            .clamp(0.0, vehicle.max_speed);
        pos.0 += path.approach_dir * vehicle.current_speed * dt;
    }
}

// ─── Log + Termination System ─────────────────────────────────────────────────

fn log_system(
    cfg: Res<SimulationConfig>,
    frame: Res<FrameCount>,
    q: Query<(Entity, &Position, &Vehicle)>,
    mut app_exit: MessageWriter<AppExit>,
) {
    println!("── Frame {:>4} ──────────────────────────────", frame.0);
    for (entity, pos, vehicle) in q.iter() {
        println!(
            "  Entity {:?}  pos ({:6.3}, {:6.3})  speed {:5.3}  status {:?}",
            entity, pos.0.x, pos.0.y, vehicle.current_speed, vehicle.status
        );
    }

    if frame.0 >= cfg.simulation.max_frames {
        println!(
            "Simulation complete after {} frames. Exiting.",
            cfg.simulation.max_frames
        );
        app_exit.write(AppExit::Success);
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    let sim_cfg = SimulationConfig::load_or_create();
    println!(
        "Loaded runtime config from {}: {:?}",
        SimulationConfig::FILE_PATH,
        sim_cfg
    );

    App::new()
        .add_plugins(MinimalPlugins)
        .insert_resource(ConflictManager { granted: None })
        .insert_resource(sim_cfg)
        .add_systems(Startup, startup)
        .add_systems(
            Update,
            (intersection_system, idm_system, movement_system, log_system).chain(),
        )
        .run();
}
