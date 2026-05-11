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
    /// Vehicles in Waiting state stop at this distance from the conflict point.
    stop_distance: f32,
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
                stop_distance: 0.15,
            },
            spawn: SpawnSection {
                north_spawn_y: 4.0,
                east_spawn_x: 4.0,
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
        if cfg.intersection.stop_distance < 0.0 {
            return Err("stop_distance must be >= 0".to_string());
        }
        if cfg.vehicle.max_speed <= 0.0 {
            return Err("vehicle_max_speed must be > 0".to_string());
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
clear_radius = {}\n\
# Waiting vehicles coast forward until this distance from the conflict point.\n\
stop_distance = {}\n\n\
[spawn]\n\
# Initial Y for north vehicle (moves toward negative Y).\n\
north_spawn_y = {}\n\
# Initial X for east vehicle (moves toward negative X).\n\
east_spawn_x = {}\n\n\
[vehicle]\n\
# Max vehicle speed units per second.\n\
max_speed = {}\n",
            self.simulation.fixed_dt,
            self.simulation.max_frames,
            self.intersection.approach_radius,
            self.intersection.clear_radius,
            self.intersection.stop_distance,
            self.spawn.north_spawn_y,
            self.spawn.east_spawn_x,
            self.vehicle.max_speed
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
            let dist = pos.0.distance(path.conflict_point);
            if dist < cfg.intersection.approach_radius {
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

// ─── Movement System ──────────────────────────────────────────────────────────

fn movement_system(cfg: Res<SimulationConfig>, mut q: Query<(&mut Position, &Vehicle, &Path)>) {
    for (mut pos, vehicle, path) in &mut q {
        match vehicle.status {
            VehicleStatus::Cruising | VehicleStatus::Crossing => {
                pos.0 += path.approach_dir * vehicle.max_speed * cfg.simulation.fixed_dt;
            }
            VehicleStatus::Waiting => {
                // Coast forward until stop_distance from the conflict point.
                let dist = pos.0.distance(path.conflict_point);
                if dist > cfg.intersection.stop_distance {
                    pos.0 += path.approach_dir * vehicle.max_speed * cfg.simulation.fixed_dt;
                }
            }
        }
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
            "  Entity {:?}  pos ({:6.3}, {:6.3})  status {:?}",
            entity, pos.0.x, pos.0.y, vehicle.status
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
            (intersection_system, movement_system, log_system).chain(),
        )
        .run();
}
