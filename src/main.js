// ─── Constants ────────────────────────────────────────────────────────────────

const VISUAL_SCALE = 100;
const VEHICLE_RADIUS = 10;
const CANVAS_SIZE = 800;
const CENTER = CANVAS_SIZE / 2;

const COLOR = {
  cruising: 0x00cc44,
  crossing: 0x00cc44,
  yielding: 0xffcc00, // yellow: stopped for right-hand vehicle
  waiting:  0xff2222,
  approach: 0xffee00,
  clear:    0xff8800,
  cross:    0xffffff,
  label:    0xffff00,
};

const statsEl = document.getElementById("stats");

function showError(msg) {
  statsEl.style.color = "#ff4444";
  statsEl.textContent = "ERROR: " + msg;
  console.error(msg);
}

// ─── Coordinate transform ─────────────────────────────────────────────────────

function toScreen(x, y) {
  return {
    sx: CENTER + x * VISUAL_SCALE,
    sy: CENTER - y * VISUAL_SCALE,
  };
}

// ─── Main init (wrapped so errors surface visibly) ────────────────────────────

async function main() {
  // 1. Verify Tauri API is injected
  if (!window.__TAURI__) {
    showError("window.__TAURI__ not found. Make sure withGlobalTauri=true in tauri.conf.json");
    return;
  }

  // 2. Verify Pixi.js loaded from CDN
  if (!window.PIXI) {
    showError("Pixi.js not loaded. Check CDN URL in index.html");
    return;
  }

  statsEl.textContent = "Initializing renderer...";

  // 3. Init Pixi application
  const app = new PIXI.Application();
  await app.init({
    canvas: document.getElementById("canvas"),
    width: CANVAS_SIZE,
    height: CANVAS_SIZE,
    backgroundColor: 0x111111,
    antialias: true,
    autoDensity: true,
  });

  // 4. Static scene layer (intersection rings + cross)
  const sceneLayer = new PIXI.Graphics();
  app.stage.addChild(sceneLayer);

  function drawStaticScene(approachRadius, clearRadius) {
    sceneLayer.clear();

    const ROAD_W   = 80;  // total road width in canvas pixels (each lane = 40 px)
    const DASH_LEN = 15;
    const GAP_LEN  = 10;

    // ── Road bodies ───────────────────────────────────────────────────────────
    // N-S road (vertical strip centred on x = CENTER)
    sceneLayer
      .rect(CENTER - ROAD_W / 2, 0, ROAD_W, CANVAS_SIZE)
      .fill({ color: 0x2a2a2a });
    // E-W road (horizontal strip centred on y = CENTER)
    sceneLayer
      .rect(0, CENTER - ROAD_W / 2, CANVAS_SIZE, ROAD_W)
      .fill({ color: 0x2a2a2a });
    // Intersection box (slightly lighter so the cross reads clearly)
    sceneLayer
      .rect(CENTER - ROAD_W / 2, CENTER - ROAD_W / 2, ROAD_W, ROAD_W)
      .fill({ color: 0x383838 });

    // ── Dashed yellow centre-lines ────────────────────────────────────────────
    // N-S centreline (vertical dashes at x = CENTER, separating the two lanes)
    for (let y = 0; y < CANVAS_SIZE; y += DASH_LEN + GAP_LEN) {
      sceneLayer
        .moveTo(CENTER, y)
        .lineTo(CENTER, Math.min(y + DASH_LEN, CANVAS_SIZE))
        .stroke({ width: 2, color: 0xffee00, alpha: 0.85 });
    }
    // E-W centreline (horizontal dashes at y = CENTER)
    for (let x = 0; x < CANVAS_SIZE; x += DASH_LEN + GAP_LEN) {
      sceneLayer
        .moveTo(x, CENTER)
        .lineTo(Math.min(x + DASH_LEN, CANVAS_SIZE), CENTER)
        .stroke({ width: 2, color: 0xffee00, alpha: 0.85 });
    }

    // ── Approach / clear-zone circles (reference overlay) ─────────────────────
    const ap = approachRadius * VISUAL_SCALE;
    const cr = clearRadius   * VISUAL_SCALE;
    sceneLayer
      .circle(CENTER, CENTER, ap)
      .stroke({ width: 1.5, color: COLOR.approach, alpha: 0.5 });
    sceneLayer
      .circle(CENTER, CENTER, cr)
      .stroke({ width: 1.5, color: COLOR.clear, alpha: 0.5 });
  }

  // 5. Debug layer (laser lines — rendered below vehicles)
  const debugLayer = new PIXI.Graphics();
  app.stage.addChild(debugLayer);

  // 6. Dynamic vehicle layer
  const dynamicLayer = new PIXI.Container();
  app.stage.addChild(dynamicLayer);

  const vehicleGraphics = new Map();
  const vehicleLabels   = new Map();
  const leaderMarkers   = new Map(); // "L" text above leaders
  const yieldMarkers    = new Map(); // "!" text above yielding vehicles

  function getOrCreateVehicle(id) {
    if (!vehicleGraphics.has(id)) {
      const g = new PIXI.Graphics();
      dynamicLayer.addChild(g);
      vehicleGraphics.set(id, g);

      const label = new PIXI.Text({
        text: "",
        style: { fontSize: 12, fill: COLOR.label, fontFamily: "monospace" },
      });
      label.anchor.set(0.5, 1);
      dynamicLayer.addChild(label);
      vehicleLabels.set(id, label);

      const ym = new PIXI.Text({
        text: "!",
        style: { fontSize: 16, fill: COLOR.yielding, fontFamily: "monospace", fontWeight: "bold" },
      });
      ym.anchor.set(0.5, 0.5);
      ym.visible = false;
      dynamicLayer.addChild(ym);
      yieldMarkers.set(id, ym);
    }
    return { g: vehicleGraphics.get(id), label: vehicleLabels.get(id), ym: yieldMarkers.get(id) };
  }

  function updateVehicle(v) {
    const { g, label, ym } = getOrCreateVehicle(v.id);
    const { sx, sy } = toScreen(v.x, v.y);
    const color = v.status === "yielding" ? COLOR.yielding
                : v.status === "waiting"  ? COLOR.waiting
                : COLOR.cruising;

    g.clear();
    g.circle(sx, sy, VEHICLE_RADIUS).fill({ color, alpha: 0.9 });
    g.circle(sx, sy, VEHICLE_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    // Direction arrow — Rust Y-up → canvas Y-down, so flip dir_y
    const arrowLen = VEHICLE_RADIUS * 1.8;
    const dx =  v.dir_x * arrowLen;
    const dy = -v.dir_y * arrowLen;
    g.moveTo(sx, sy).lineTo(sx + dx, sy + dy).stroke({ width: 2, color: 0xffffff, alpha: 0.6 });

    label.text =
      "#" + v.id + " " + v.status.toUpperCase() + "\n" +
      "V:" + v.speed.toFixed(2) + " A:" + v.accel.toFixed(2);
    label.x = sx;
    label.y = sy - VEHICLE_RADIUS - 4;

    // "!" yield marker
    ym.visible = v.status === "yielding";
    ym.x = sx + VEHICLE_RADIUS + 8;
    ym.y = sy - VEHICLE_RADIUS;
  }

  function getOrCreateLeaderMarker(id) {
    if (!leaderMarkers.has(id)) {
      const m = new PIXI.Text({
        text: "L",
        style: { fontSize: 11, fill: 0xffffff, fontFamily: "monospace", fontWeight: "bold" },
      });
      m.anchor.set(0.5, 0.5);
      m.alpha = 0.75;
      dynamicLayer.addChild(m);
      leaderMarkers.set(id, m);
    }
    return leaderMarkers.get(id);
  }

  // 7. FPS counter
  let frameCount = 0;
  let lastFpsTime = performance.now();

  function updateStats(frame) {
    frameCount++;
    const now = performance.now();
    if (now - lastFpsTime >= 1000) {
      const fps = Math.round((frameCount * 1000) / (now - lastFpsTime));
      frameCount = 0;
      lastFpsTime = now;
      const parts = frame.vehicles.map(
        (v) => "#" + v.id + "[" + v.status + "] V=" + v.speed.toFixed(2)
      );
      statsEl.style.color = "#888";
      statsEl.textContent = fps + " fps   |   " + parts.join("   ");
    }
  }

  // 8. Subscribe to Tauri events
  statsEl.textContent = "Connecting to simulation...";

  let staticSceneDrawn = false;

  try {
    const { listen } = window.__TAURI__.event;

    await listen("sim-frame", (event) => {
      const frame = event.payload;

      if (!staticSceneDrawn) {
        drawStaticScene(frame.approach_radius, frame.clear_radius);
        staticSceneDrawn = true;
      }

      // Build a quick id→vehicle lookup for the debug pass.
      const byId = new Map(frame.vehicles.map((v) => [v.id, v]));

      // ── Debug: lines between followers→leaders and yielders→right-hand ─────
      debugLayer.clear();
      // Collect which vehicles ARE leaders so we can draw the 'L' badge.
      const leaderIds = new Set();
      for (const v of frame.vehicles) {
        // Leader-following line (cyan)
        if (v.leader_id != null) {
          const leader = byId.get(v.leader_id);
          if (leader) {
            leaderIds.add(v.leader_id);
            const { sx: fx, sy: fy } = toScreen(v.x, v.y);
            const { sx: lx, sy: ly } = toScreen(leader.x, leader.y);
            debugLayer
              .moveTo(fx, fy)
              .lineTo(lx, ly)
              .stroke({ width: 2, color: 0x00ccff, alpha: 0.7 });
          }
        }
        // Yield line (yellow dashed look — drawn as a thicker yellow line)
        if (v.yielding_to_id != null) {
          const target = byId.get(v.yielding_to_id);
          if (target) {
            const { sx: fx, sy: fy } = toScreen(v.x, v.y);
            const { sx: tx, sy: ty } = toScreen(target.x, target.y);
            debugLayer
              .moveTo(fx, fy)
              .lineTo(tx, ty)
              .stroke({ width: 2.5, color: 0xffcc00, alpha: 0.85 });
          }
        }
      }

      // Show / hide 'L' markers.
      const activeIds = new Set(frame.vehicles.map((v) => v.id));
      for (const v of frame.vehicles) {
        if (leaderIds.has(v.id)) {
          const m = getOrCreateLeaderMarker(v.id);
          const { sx, sy } = toScreen(v.x, v.y);
          m.x = sx + VEHICLE_RADIUS + 6;
          m.y = sy - VEHICLE_RADIUS - 2;
          m.visible = true;
        } else if (leaderMarkers.has(v.id)) {
          leaderMarkers.get(v.id).visible = false;
        }
      }

      // Remove graphics for vehicles that have left the scene.
      for (const [id, g] of vehicleGraphics) {
        if (!activeIds.has(id)) {
          dynamicLayer.removeChild(g);
          vehicleGraphics.delete(id);
          const label = vehicleLabels.get(id);
          if (label) { dynamicLayer.removeChild(label); vehicleLabels.delete(id); }
          const marker = leaderMarkers.get(id);
          if (marker) { dynamicLayer.removeChild(marker); leaderMarkers.delete(id); }
          const ym = yieldMarkers.get(id);
          if (ym) { dynamicLayer.removeChild(ym); yieldMarkers.delete(id); }
        }
      }

      for (const vehicle of frame.vehicles) {
        updateVehicle(vehicle);
      }

      updateStats(frame);
    });

    await listen("sim-done", () => {
      debugLayer.clear();
      for (const g of vehicleGraphics.values()) dynamicLayer.removeChild(g);
      for (const l of vehicleLabels.values()) dynamicLayer.removeChild(l);
      for (const m of leaderMarkers.values()) dynamicLayer.removeChild(m);
      for (const y of yieldMarkers.values()) dynamicLayer.removeChild(y);
      vehicleGraphics.clear();
      vehicleLabels.clear();
      leaderMarkers.clear();
      yieldMarkers.clear();

      statsEl.style.color = "#44ff88";
      statsEl.textContent = "Simulation complete — all vehicles crossed.";
    });

    console.log("Listening for sim-frame / sim-done events...");
  } catch (err) {
    showError("listen() failed: " + err);
  }
}

main();
