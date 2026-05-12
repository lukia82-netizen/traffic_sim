// ─── Constants ────────────────────────────────────────────────────────────────

const VISUAL_SCALE  = 100;
const VEHICLE_RADIUS = 10;
const CANVAS_SIZE   = 800;
const CENTER        = CANVAS_SIZE / 2;

// Road half-width in screen pixels (LANE_OFFSET = 0.2 world units)
const ROAD_HALF_PX  = 0.5 * VISUAL_SCALE; // draw stop lines spanning ±0.5 world units

const COLOR = {
  cruising:   0x00cc44,
  crossing:   0x44aaff,
  stopped:    0xff2222,
  yielding:   0xff9900,  // left-turner waiting for oncoming gap
  approach:   0xffee00,
  clear:      0xff8800,
  cross:      0xffffff,
  label:      0xffff00,
  stopLine:   0xffffff,
  blinker:    0xffdd00,  // left-turn indicator dot
  oncomingLine: 0xff6600,
  lightGreen: 0x00ee44,
  lightRed:   0xee2222,
  lightOff:   0x333333,
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

// ─── Main init ────────────────────────────────────────────────────────────────

async function main() {
  if (!window.__TAURI__) {
    showError("window.__TAURI__ not found. Make sure withGlobalTauri=true in tauri.conf.json");
    return;
  }
  if (!window.PIXI) {
    showError("Pixi.js not loaded. Check CDN URL in index.html");
    return;
  }

  statsEl.textContent = "Initializing renderer...";

  const app = new PIXI.Application();
  await app.init({
    canvas: document.getElementById("canvas"),
    width:  CANVAS_SIZE,
    height: CANVAS_SIZE,
    backgroundColor: 0x111111,
    antialias: true,
    autoDensity: true,
  });

  // ── Layer order: static → debug → vehicles ─────────────────────────────────
  const sceneLayer  = new PIXI.Graphics();
  const tlLayer     = new PIXI.Graphics(); // traffic light indicators (redrawn each frame)
  const debugLayer  = new PIXI.Graphics(); // laser lines
  const dynamicLayer = new PIXI.Container();
  app.stage.addChild(sceneLayer);
  app.stage.addChild(tlLayer);
  app.stage.addChild(debugLayer);
  app.stage.addChild(dynamicLayer);

  // ── Static scene (drawn once) ──────────────────────────────────────────────

  function drawStaticScene(approachRadius, clearRadius, stopLineOffset) {
    sceneLayer.clear();
    const ap  = approachRadius  * VISUAL_SCALE;
    const cr  = clearRadius     * VISUAL_SCALE;
    const slo = stopLineOffset  * VISUAL_SCALE;
    const rw  = ROAD_HALF_PX;   // road half-width in px

    // Approach zone ring
    sceneLayer
      .circle(CENTER, CENTER, ap)
      .stroke({ width: 1.5, color: COLOR.approach, alpha: 0.5 });

    // Clear zone ring
    sceneLayer
      .circle(CENTER, CENTER, cr)
      .stroke({ width: 1.5, color: COLOR.clear, alpha: 0.5 });

    // Intersection cross-hair
    sceneLayer
      .moveTo(CENTER - 20, CENTER).lineTo(CENTER + 20, CENTER)
      .stroke({ width: 2, color: COLOR.cross });
    sceneLayer
      .moveTo(CENTER, CENTER - 20).lineTo(CENTER, CENTER + 20)
      .stroke({ width: 2, color: COLOR.cross });

    // ── Stop lines ────────────────────────────────────────────────────────────
    // North approach: horizontal line above centre at y = +slo (screen y is flipped)
    const stopN_y = CENTER - slo;
    const stopS_y = CENTER + slo;
    const stopE_x = CENTER + slo;
    const stopW_x = CENTER - slo;

    // North & South: horizontal dashed bars
    for (const sy of [stopN_y, stopS_y]) {
      sceneLayer
        .moveTo(CENTER - rw, sy).lineTo(CENTER + rw, sy)
        .stroke({ width: 3, color: COLOR.stopLine, alpha: 0.85 });
    }
    // East & West: vertical dashed bars
    for (const sx of [stopE_x, stopW_x]) {
      sceneLayer
        .moveTo(sx, CENTER - rw).lineTo(sx, CENTER + rw)
        .stroke({ width: 3, color: COLOR.stopLine, alpha: 0.85 });
    }
  }

  // ── Traffic light indicators (redrawn each frame) ──────────────────────────
  // 4 circles: one per approach, positioned just outside the stop line

  const TL_RADIUS = 8;
  const TL_OFFSET = ROAD_HALF_PX + 18; // side offset from road edge

  function drawTrafficLights(lightNS, lightEW, stopLineOffset) {
    tlLayer.clear();
    const slo   = stopLineOffset * VISUAL_SCALE;
    const nsCol = lightNS ? COLOR.lightGreen : COLOR.lightRed;
    const ewCol = lightEW ? COLOR.lightGreen : COLOR.lightRed;

    // North approach light  (above centre, right side of the road)
    tlLayer.circle(CENTER + TL_OFFSET, CENTER - slo, TL_RADIUS).fill({ color: nsCol, alpha: 0.95 });
    tlLayer.circle(CENTER + TL_OFFSET, CENTER - slo, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    // South approach light  (below centre, left side of the road)
    tlLayer.circle(CENTER - TL_OFFSET, CENTER + slo, TL_RADIUS).fill({ color: nsCol, alpha: 0.95 });
    tlLayer.circle(CENTER - TL_OFFSET, CENTER + slo, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    // East approach light   (right of centre, bottom side of the road)
    tlLayer.circle(CENTER + slo, CENTER + TL_OFFSET, TL_RADIUS).fill({ color: ewCol, alpha: 0.95 });
    tlLayer.circle(CENTER + slo, CENTER + TL_OFFSET, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    // West approach light   (left of centre, top side of the road)
    tlLayer.circle(CENTER - slo, CENTER - TL_OFFSET, TL_RADIUS).fill({ color: ewCol, alpha: 0.95 });
    tlLayer.circle(CENTER - slo, CENTER - TL_OFFSET, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });
  }

  // ── Vehicle graphics ───────────────────────────────────────────────────────

  const vehicleGraphics = new Map();
  const vehicleLabels   = new Map();
  const leaderMarkers   = new Map();

  function getOrCreateVehicle(id) {
    if (!vehicleGraphics.has(id)) {
      const g = new PIXI.Graphics();
      dynamicLayer.addChild(g);
      vehicleGraphics.set(id, g);

      const label = new PIXI.Text({
        text: "",
        style: { fontSize: 11, fill: COLOR.label, fontFamily: "monospace" },
      });
      label.anchor.set(0.5, 1);
      dynamicLayer.addChild(label);
      vehicleLabels.set(id, label);
    }
    return { g: vehicleGraphics.get(id), label: vehicleLabels.get(id) };
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

  function vehicleColor(status) {
    if (status === "stopped")  return COLOR.stopped;
    if (status === "crossing") return COLOR.crossing;
    if (status === "yielding") return COLOR.yielding;
    return COLOR.cruising;
  }

  function updateVehicle(v) {
    const { g, label } = getOrCreateVehicle(v.id);
    const { sx, sy }   = toScreen(v.x, v.y);
    const color        = vehicleColor(v.status);

    g.clear();
    g.circle(sx, sy, VEHICLE_RADIUS).fill({ color, alpha: 0.9 });
    g.circle(sx, sy, VEHICLE_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    // Direction arrow (flip dir_y because canvas Y is downward)
    const arrowLen = VEHICLE_RADIUS * 1.8;
    const dx =  v.dir_x * arrowLen;
    const dy = -v.dir_y * arrowLen;
    g.moveTo(sx, sy).lineTo(sx + dx, sy + dy).stroke({ width: 2, color: 0xffffff, alpha: 0.6 });

    // Left-turn blinker: yellow dot on the LEFT side of the vehicle.
    // Left side in screen space: offset = (-dir_y, -dir_x) * blinker_dist
    if (v.intent === "left") {
      const bd  = VEHICLE_RADIUS * 1.1;
      const blx = sx - v.dir_y * bd;
      const bly = sy - v.dir_x * bd;
      g.circle(blx, bly, 4).fill({ color: COLOR.blinker, alpha: 0.95 });
    }

    const intentTag = v.intent === "left" ? " ↰" : "";
    label.text =
      "#" + v.id + " " + v.status.toUpperCase() + intentTag + "\n" +
      "V:" + v.speed.toFixed(2) + " A:" + v.accel.toFixed(2);
    label.x = sx;
    label.y = sy - VEHICLE_RADIUS - 4;
  }

  // ── FPS / stats ────────────────────────────────────────────────────────────

  let frameCount  = 0;
  let lastFpsTime = performance.now();

  function updateStats(frame) {
    frameCount++;
    const now = performance.now();
    if (now - lastFpsTime >= 1000) {
      const fps = Math.round((frameCount * 1000) / (now - lastFpsTime));
      frameCount  = 0;
      lastFpsTime = now;
      const parts = frame.vehicles.map(
        (v) => "#" + v.id + "[" + v.status + "] V=" + v.speed.toFixed(2)
      );
      statsEl.style.color = "#888";
      statsEl.textContent = fps + " fps   |   " + parts.join("   ");
    }
  }

  // ── Tauri event subscription ───────────────────────────────────────────────

  statsEl.textContent = "Connecting to simulation...";
  let staticSceneDrawn = false;

  try {
    const { listen } = window.__TAURI__.event;

    await listen("sim-frame", (event) => {
      const frame = event.payload;

      if (!staticSceneDrawn) {
        drawStaticScene(frame.approach_radius, frame.clear_radius, frame.stop_line_offset);
        staticSceneDrawn = true;
      }

      // Traffic light indicators (updated every frame for phase transitions)
      drawTrafficLights(frame.light_ns, frame.light_ew, frame.stop_line_offset);

      // Build id→vehicle map for debug pass
      const byId = new Map(frame.vehicles.map((v) => [v.id, v]));

      // ── Debug laser lines ──────────────────────────────────────────────────
      debugLayer.clear();

      // Car-following leader lines (white, thin)
      const leaderIds = new Set();
      for (const v of frame.vehicles) {
        if (v.leader_id == null) continue;
        const leader = byId.get(v.leader_id);
        if (!leader) continue;
        leaderIds.add(v.leader_id);
        const { sx: fx, sy: fy } = toScreen(v.x, v.y);
        const { sx: lx, sy: ly } = toScreen(leader.x, leader.y);
        debugLayer
          .moveTo(fx, fy)
          .lineTo(lx, ly)
          .stroke({ width: 1, color: 0xffffff, alpha: 0.3 });
      }

      // Oncoming yield lines (orange, thicker) — left-turners watching oncoming
      for (const v of frame.vehicles) {
        if (v.oncoming_yield_id == null) continue;
        const oncoming = byId.get(v.oncoming_yield_id);
        if (!oncoming) continue;
        const { sx: fx, sy: fy } = toScreen(v.x, v.y);
        const { sx: ox, sy: oy } = toScreen(oncoming.x, oncoming.y);
        debugLayer
          .moveTo(fx, fy)
          .lineTo(ox, oy)
          .stroke({ width: 2, color: COLOR.oncomingLine, alpha: 0.7 });
        // Small warning diamond at the oncoming vehicle
        const ds = 6;
        debugLayer
          .moveTo(ox, oy - ds).lineTo(ox + ds, oy)
          .lineTo(ox, oy + ds).lineTo(ox - ds, oy)
          .lineTo(ox, oy - ds)
          .stroke({ width: 1.5, color: COLOR.oncomingLine, alpha: 0.9 });
      }

      // Show/hide 'L' markers
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

      // Remove departed vehicles
      for (const [id, g] of vehicleGraphics) {
        if (!activeIds.has(id)) {
          dynamicLayer.removeChild(g);
          vehicleGraphics.delete(id);
          const label = vehicleLabels.get(id);
          if (label) { dynamicLayer.removeChild(label); vehicleLabels.delete(id); }
          const marker = leaderMarkers.get(id);
          if (marker) { dynamicLayer.removeChild(marker); leaderMarkers.delete(id); }
        }
      }

      for (const vehicle of frame.vehicles) {
        updateVehicle(vehicle);
      }

      updateStats(frame);
    });

    await listen("sim-done", () => {
      debugLayer.clear();
      tlLayer.clear();
      for (const g of vehicleGraphics.values()) dynamicLayer.removeChild(g);
      for (const l of vehicleLabels.values()) dynamicLayer.removeChild(l);
      for (const m of leaderMarkers.values()) dynamicLayer.removeChild(m);
      vehicleGraphics.clear();
      vehicleLabels.clear();
      leaderMarkers.clear();

      statsEl.style.color = "#44ff88";
      statsEl.textContent = "Simulation complete — all vehicles crossed.";
    });

    console.log("Listening for sim-frame / sim-done events...");
  } catch (err) {
    showError("listen() failed: " + err);
  }
}

main();
