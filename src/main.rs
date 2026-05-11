use bevy::app::AppExit;
use bevy::color::palettes::css;
use bevy::diagnostic::FrameCount;
use bevy::math::Isometry2d;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path as StdPath;

/// Simulation-unit → pixel multiplier for all gizmo drawing.
const VISUAL_SCALE: f32 = 100.0;
/// Radius (pixels) of a vehicle circle in the viewport.
const VEHICLE_RADIUS_PX: f32 = 10.0;

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
            return Err("max_speed must be > 0".to_string());
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
fixed_dt   = {}    # simulation step in seconds\n\
max_frames = {}    # frames before graceful shutdown\n\n\
[intersection]\n\
approach_radius = {}   # distance from conflict point where a vehicle requests access\n\
clear_radius    = {}   # distance past conflict point required to release the grant\n\n\
[spawn]\n\
north_spawn_y = {}   # initial Y for north vehicle (moves toward negative Y)\n\
east_spawn_x  = {}   # initial X for east vehicle (moves toward negative X)\n\n\
[vehicle]\n\
max_speed = {}   # max vehicle speed [units/s]\n\n\
[idm]\n\
a_max     = {}   # max acceleration [m/s²]\n\
b_comf    = {}   # comfortable braking — clamps minimum deceleration [m/s²]\n\
delta     = {}   # acceleration exponent (IDM curve shape)\n\
s0        = {}   # minimum jam gap at standstill [units]\n\
t_headway = {}   # safe time headway [s]\n",
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

// ─── Camera Setup ─────────────────────────────────────────────────────────────

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2d);
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

// ─── Termination System ───────────────────────────────────────────────────────

fn termination_system(
    cfg: Res<SimulationConfig>,
    frame: Res<FrameCount>,
    mut app_exit: MessageWriter<AppExit>,
) {
    if frame.0 >= cfg.simulation.max_frames {
        app_exit.write(AppExit::Success);
    }
}

// ─── Gizmo Draw System ────────────────────────────────────────────────────────

fn draw_gizmos_system(
    mut gizmos: Gizmos,
    cfg: Res<SimulationConfig>,
    q: Query<(&Position, &Vehicle)>,
) {
    // ── Intersection marker: white cross at (0, 0) ──
    let arm = 16.0_f32;
    gizmos.line_2d(Vec2::new(-arm, 0.0), Vec2::new(arm, 0.0), css::WHITE);
    gizmos.line_2d(Vec2::new(0.0, -arm), Vec2::new(0.0, arm), css::WHITE);

    // ── Approach-radius ring (yellow) ──
    let approach_px = cfg.intersection.approach_radius * VISUAL_SCALE;
    gizmos.circle_2d(Isometry2d::IDENTITY, approach_px, css::YELLOW);

    // ── Clear-radius ring (dim orange) ──
    let clear_px = cfg.intersection.clear_radius * VISUAL_SCALE;
    gizmos.circle_2d(Isometry2d::IDENTITY, clear_px, Color::srgb(1.0, 0.5, 0.0));

    // ── Vehicles ──
    for (pos, vehicle) in q.iter() {
        let color: Color = match vehicle.status {
            VehicleStatus::Waiting => css::RED.into(),
            VehicleStatus::Cruising | VehicleStatus::Crossing => css::GREEN.into(),
        };
        let screen = pos.0 * VISUAL_SCALE;
        gizmos.circle_2d(Isometry2d::from_translation(screen), VEHICLE_RADIUS_PX, color);
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
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Traffic Sim – IDM".to_string(),
                resolution: (800u32, 800u32).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ConflictManager { granted: None })
        .insert_resource(sim_cfg)
        .add_systems(Startup, (startup, setup_camera))
        .add_systems(
            Update,
            (
                intersection_system,
                idm_system,
                movement_system,
                draw_gizmos_system,
                termination_system,
            )
                .chain(),
        )
        .run();
}
