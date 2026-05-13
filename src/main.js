// ─── Constants ────────────────────────────────────────────────────────────────

const VISUAL_SCALE   = 100;
const VEHICLE_W      = 22;   // vehicle length along heading [px]
const VEHICLE_H      = 13;   // vehicle width across heading [px]
const CANVAS_SIZE    = 800;
const CENTER         = CANVAS_SIZE / 2;

const ROAD_HALF_PX   = 0.5 * VISUAL_SCALE;

// Must stay in sync with Rust constants LANE_OFFSET and BEZIER_EXIT_DIST.
const LANE_OFFSET_W  = 0.2;
const BEZIER_EXIT_W  = 1.5;

const COLOR = {
  cruising:        0x00cc44,
  crossing:        0x44aaff,
  stopped:         0xff2222,
  yielding:        0xff9900,
  approach:        0xffee00,
  clear:           0xff8800,
  cross:           0xffffff,
  label:           0xffff00,
  stopLine:        0xffffff,
  blinker:         0xffdd00,
  oncomingLine:    0xff6600,
  lightGreen:      0x00ee44,
  lightRed:        0xee2222,
  turnArc:         0x55bbff,  // dynamic arc for in-progress Bezier turns
  pathLane:        0xffffff,  // static dashed lane-centre guides
  pathArc:         0x44aaff,  // static left-turn arc templates
  pathArcActive:   0x88ddff,  // highlighted arc for a vehicle that is currently turning
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

  // ── Layer order ────────────────────────────────────────────────────────────
  const sceneLayer   = new PIXI.Graphics();
  const pathsLayer   = new PIXI.Graphics(); // static lane guides + turn-arc templates (toggleable)
  const tlLayer      = new PIXI.Graphics(); // traffic light indicators
  const arcLayer     = new PIXI.Graphics(); // active Bezier turn-path previews
  const debugLayer   = new PIXI.Graphics(); // leader / oncoming lines
  const dynamicLayer = new PIXI.Container();
  app.stage.addChild(sceneLayer);
  app.stage.addChild(pathsLayer);
  app.stage.addChild(tlLayer);
  app.stage.addChild(arcLayer);
  app.stage.addChild(debugLayer);
  app.stage.addChild(dynamicLayer);

  pathsLayer.visible = false; // hidden by default; "Show Paths" button reveals it

  // ── "Show Paths" toggle button ─────────────────────────────────────────────
  const toggleBtn = document.createElement("button");
  toggleBtn.textContent = "Show Paths: OFF";
  toggleBtn.title = "Toggle lane-centre guides and all possible left-turn arcs.\n" +
    "Arcs follow right-hand traffic: simultaneous left-turners\n" +
    "from opposite approaches use different intersection corners\n" +
    "and never occupy the same space.";
  Object.assign(toggleBtn.style, {
    position:   "fixed",
    top:        "10px",
    right:      "10px",
    padding:    "5px 12px",
    background: "#1e293b",
    color:      "#94a3b8",
    border:     "1px solid #334155",
    borderRadius: "6px",
    fontFamily: "monospace",
    fontSize:   "12px",
    cursor:     "pointer",
    userSelect: "none",
    zIndex:     "10",
  });
  toggleBtn.addEventListener("click", () => {
    pathsLayer.visible = !pathsLayer.visible;
    toggleBtn.textContent = "Show Paths: " + (pathsLayer.visible ? "ON " : "OFF");
    toggleBtn.style.color  = pathsLayer.visible ? "#7dd3fc" : "#94a3b8";
    toggleBtn.style.border = pathsLayer.visible ? "1px solid #38bdf8" : "1px solid #334155";
  });
  document.body.appendChild(toggleBtn);

  // ── Static scene (drawn once) ──────────────────────────────────────────────

  function drawStaticScene(approachRadius, clearRadius, stopLineOffset) {
    sceneLayer.clear();
    const ap  = approachRadius  * VISUAL_SCALE;
    const cr  = clearRadius     * VISUAL_SCALE;
    const slo = stopLineOffset  * VISUAL_SCALE;
    const rw  = ROAD_HALF_PX;

    sceneLayer
      .circle(CENTER, CENTER, ap)
      .stroke({ width: 1.5, color: COLOR.approach, alpha: 0.5 });

    sceneLayer
      .circle(CENTER, CENTER, cr)
      .stroke({ width: 1.5, color: COLOR.clear, alpha: 0.5 });

    sceneLayer
      .moveTo(CENTER - 20, CENTER).lineTo(CENTER + 20, CENTER)
      .stroke({ width: 2, color: COLOR.cross });
    sceneLayer
      .moveTo(CENTER, CENTER - 20).lineTo(CENTER, CENTER + 20)
      .stroke({ width: 2, color: COLOR.cross });

    const stopN_y = CENTER - slo;
    const stopS_y = CENTER + slo;
    const stopE_x = CENTER + slo;
    const stopW_x = CENTER - slo;

    for (const sy of [stopN_y, stopS_y]) {
      sceneLayer
        .moveTo(CENTER - rw, sy).lineTo(CENTER + rw, sy)
        .stroke({ width: 3, color: COLOR.stopLine, alpha: 0.85 });
    }
    for (const sx of [stopE_x, stopW_x]) {
      sceneLayer
        .moveTo(sx, CENTER - rw).lineTo(sx, CENTER + rw)
        .stroke({ width: 3, color: COLOR.stopLine, alpha: 0.85 });
    }
  }

  // ── Path guides (drawn once, toggled by button) ───────────────────────────

  // Draw a dashed line between two screen points (PIXI has no native dash support).
  function drawDashedLine(g, x1, y1, x2, y2, dashLen, gapLen, style) {
    const dx  = x2 - x1, dy = y2 - y1;
    const len = Math.hypot(dx, dy);
    if (len === 0) return;
    const nx = dx / len, ny = dy / len;
    let pos = 0, on = true;
    while (pos < len) {
      const seg = Math.min(on ? dashLen : gapLen, len - pos);
      if (on) {
        g.moveTo(x1 + nx * pos,       y1 + ny * pos)
         .lineTo(x1 + nx * (pos + seg), y1 + ny * (pos + seg))
         .stroke(style);
      }
      pos += seg;
      on   = !on;
    }
  }

  // Draw a small arrowhead at (ex, ey) pointing in direction (dx, dy) [screen].
  function drawArrowHead(g, ex, ey, dx, dy, size, style) {
    const len  = Math.hypot(dx, dy);
    if (len === 0) return;
    const nx = dx / len, ny = dy / len;
    const px = -ny, py = nx; // perpendicular
    g.moveTo(ex, ey)
     .lineTo(ex - nx * size + px * size * 0.5, ey - ny * size + py * size * 0.5)
     .moveTo(ex, ey)
     .lineTo(ex - nx * size - px * size * 0.5, ey - ny * size - py * size * 0.5)
     .stroke(style);
  }

  // Draw all static path guides into pathsLayer (called once on first frame).
  // All 4 turn arcs comply with right-hand traffic: simultaneous left-turners
  // from opposite approaches occupy different intersection quadrants and their
  // paths never intersect — vehicles pass each other "back-to-back".
  function drawPathGuides(approachRadius, stopLineOffset) {
    pathsLayer.clear();
    const ar  = approachRadius * VISUAL_SCALE;
    const slo = stopLineOffset * VISUAL_SCALE;
    const lo  = LANE_OFFSET_W  * VISUAL_SCALE; // lane offset in px
    const ext = BEZIER_EXIT_W  * VISUAL_SCALE; // exit distance in px

    const dashStyle = { width: 1, color: COLOR.pathLane, alpha: 0.13 };
    const arcStyle  = { width: 1.5, color: COLOR.pathArc, alpha: 0.22 };
    const arwStyle  = { width: 1.5, color: COLOR.pathArc, alpha: 0.35 };

    // ── Straight lane centre-lines (dashed, from approach circle to stop line) ─
    // North approach (travelling south):  x = +lo
    drawDashedLine(pathsLayer, CENTER + lo, CENTER - ar, CENTER + lo, CENTER - slo, 7, 7, dashStyle);
    // South approach (travelling north):  x = -lo
    drawDashedLine(pathsLayer, CENTER - lo, CENTER + ar, CENTER - lo, CENTER + slo, 7, 7, dashStyle);
    // East  approach (travelling west):   y = CENTER + lo  (world y = -lo)
    drawDashedLine(pathsLayer, CENTER + ar, CENTER + lo, CENTER + slo, CENTER + lo, 7, 7, dashStyle);
    // West  approach (travelling east):   y = CENTER - lo  (world y = +lo)
    drawDashedLine(pathsLayer, CENTER - ar, CENTER - lo, CENTER - slo, CENTER - lo, 7, 7, dashStyle);

    // ── Left-turn Bezier arc templates ────────────────────────────────────────
    // Control points land at the four "elbow" corners (±lo, ±lo) — these are
    // the corners of the inner lane-width square.  Each arc lives in its own
    // quadrant, ensuring RHT back-to-back crossing for simultaneous left turns.
    //
    //  Arc                   start              ctrl              end              quadrant
    //  North → traveling E   (+lo, -slo)    (+lo, -lo)       (+ext, -lo)     top-right
    //  South → traveling W   (-lo, +slo)    (-lo, +lo)       (-ext, +lo)     bottom-left
    //  East  → traveling S   (+slo, +lo)    (+lo, +lo)       (+lo, +ext)     bottom-right
    //  West  → traveling N   (-slo, -lo)    (-lo, -lo)       (-lo, -ext)     top-left

    const arcs = [
      // [sx, sy,  cx, cy,  ex, ey,  exit-dx, exit-dy]  (all relative to CENTER)
      [ lo, -slo,  lo, -lo,  ext, -lo,    1,  0 ],  // North → E (top-right quad)
      [-lo,  slo, -lo,  lo, -ext,  lo,   -1,  0 ],  // South → W (bottom-left quad)
      [ slo, lo,   lo,  lo,  lo,  ext,    0,  1 ],  // East  → S (bottom-right quad)
      [-slo,-lo,  -lo, -lo, -lo, -ext,    0, -1 ],  // West  → N (top-left quad)
    ];

    for (const [sx, sy, cx, cy, ex, ey, edx, edy] of arcs) {
      pathsLayer
        .moveTo(CENTER + sx, CENTER + sy)
        .quadraticCurveTo(CENTER + cx, CENTER + cy, CENTER + ex, CENTER + ey)
        .stroke(arcStyle);

      // Small arrowhead at exit point showing travel direction
      drawArrowHead(
        pathsLayer,
        CENTER + ex, CENTER + ey,
        edx, edy, 7, arwStyle,
      );

      // Small dot at arc start (on the stop line)
      pathsLayer
        .circle(CENTER + sx, CENTER + sy, 2.5)
        .fill({ color: COLOR.pathArc, alpha: 0.35 });
    }
  }

  // ── Traffic light indicators ───────────────────────────────────────────────

  const TL_RADIUS = 8;
  const TL_OFFSET = ROAD_HALF_PX + 18;

  function drawTrafficLights(lightNS, lightEW, stopLineOffset) {
    tlLayer.clear();
    const slo   = stopLineOffset * VISUAL_SCALE;
    const nsCol = lightNS ? COLOR.lightGreen : COLOR.lightRed;
    const ewCol = lightEW ? COLOR.lightGreen : COLOR.lightRed;

    tlLayer.circle(CENTER + TL_OFFSET, CENTER - slo, TL_RADIUS).fill({ color: nsCol, alpha: 0.95 });
    tlLayer.circle(CENTER + TL_OFFSET, CENTER - slo, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    tlLayer.circle(CENTER - TL_OFFSET, CENTER + slo, TL_RADIUS).fill({ color: nsCol, alpha: 0.95 });
    tlLayer.circle(CENTER - TL_OFFSET, CENTER + slo, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    tlLayer.circle(CENTER + slo, CENTER + TL_OFFSET, TL_RADIUS).fill({ color: ewCol, alpha: 0.95 });
    tlLayer.circle(CENTER + slo, CENTER + TL_OFFSET, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });

    tlLayer.circle(CENTER - slo, CENTER - TL_OFFSET, TL_RADIUS).fill({ color: ewCol, alpha: 0.95 });
    tlLayer.circle(CENTER - slo, CENTER - TL_OFFSET, TL_RADIUS).stroke({ width: 1, color: 0xffffff, alpha: 0.4 });
  }

  // ── Active turn-arc layer (redrawn every frame) ───────────────────────────
  // For each vehicle currently on a Bezier turn:
  //   1. Full arc (start → end) — highlighted path overview
  //   2. Remaining sub-arc (current pos → end) — how far left to travel
  //   3. Progress dot at the current position

  function drawTurnArcs(vehicles) {
    arcLayer.clear();
    for (const v of vehicles) {
      if (v.turning_progress == null) continue;

      const s   = toScreen(v.turn_start_x, v.turn_start_y);
      const c   = toScreen(v.turn_ctrl_x,  v.turn_ctrl_y);
      const e   = toScreen(v.turn_end_x,   v.turn_end_y);
      const cur = toScreen(v.x, v.y);
      const p   = v.turning_progress;

      // Full path — bright highlight so it stands out from the static template
      arcLayer
        .moveTo(s.sx, s.sy)
        .quadraticCurveTo(c.sx, c.sy, e.sx, e.sy)
        .stroke({ width: 2.5, color: COLOR.pathArcActive, alpha: 0.55 });

      // Remaining sub-arc (De Casteljau approximation of the [p,1] sub-curve)
      const tR  = 1.0 - p;
      const mcx = c.sx + tR * (e.sx - c.sx);
      const mcy = c.sy + tR * (e.sy - c.sy);
      arcLayer
        .moveTo(cur.sx, cur.sy)
        .quadraticCurveTo(mcx, mcy, e.sx, e.sy)
        .stroke({ width: 2, color: COLOR.pathArcActive, alpha: 0.85 });

      // Current-position pulse dot
      arcLayer.circle(cur.sx, cur.sy, 3.5).fill({ color: COLOR.pathArcActive, alpha: 0.9 });

      // Exit point marker
      arcLayer.circle(e.sx, e.sy, 3).fill({ color: COLOR.pathArcActive, alpha: 0.45 });
    }
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

    // ── Rotated rectangle body ─────────────────────────────────────────────
    // Heading angle in screen space: dir_y is negated because canvas Y is down.
    const angle = Math.atan2(-v.dir_y, v.dir_x);
    const cos   = Math.cos(angle);
    const sin   = Math.sin(angle);

    // Rotate a local-space offset (dx, dy) to screen space around (sx, sy).
    function rot(dx, dy) {
      return { x: sx + dx * cos - dy * sin, y: sy + dx * sin + dy * cos };
    }

    const hw = VEHICLE_W / 2;
    const hh = VEHICLE_H / 2;
    const c0 = rot(-hw, -hh);
    const c1 = rot( hw, -hh);
    const c2 = rot( hw,  hh);
    const c3 = rot(-hw,  hh);

    // Body fill + stroke (draw path twice: once fill, once stroke)
    g.moveTo(c0.x, c0.y).lineTo(c1.x, c1.y).lineTo(c2.x, c2.y).lineTo(c3.x, c3.y).lineTo(c0.x, c0.y)
      .fill({ color, alpha: 0.92 });
    g.moveTo(c0.x, c0.y).lineTo(c1.x, c1.y).lineTo(c2.x, c2.y).lineTo(c3.x, c3.y).lineTo(c0.x, c0.y)
      .stroke({ width: 1.2, color: 0xffffff, alpha: 0.35 });

    // Front indicator: small bright dot at the nose
    const nose = rot(hw - 2, 0);
    g.circle(nose.x, nose.y, 2.5).fill({ color: 0xffffff, alpha: 0.85 });

    // Left-turn blinker: yellow dot on the left flank of the vehicle
    if (v.intent === "left") {
      const bl = rot(hw * 0.4, -(hh + 3));
      g.circle(bl.x, bl.y, 3.5).fill({ color: COLOR.blinker, alpha: 0.95 });
    }

    const intentTag = v.intent === "left" ? " ↰" : "";
    label.text =
      "#" + v.id + " " + v.status.toUpperCase() + intentTag + "\n" +
      "V:" + v.speed.toFixed(2) + " A:" + v.accel.toFixed(2);
    label.x = sx;
    label.y = sy - hh - 6;
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
        drawPathGuides(frame.approach_radius, frame.stop_line_offset);
        staticSceneDrawn = true;
      }

      drawTrafficLights(frame.light_ns, frame.light_ew, frame.stop_line_offset);
      drawTurnArcs(frame.vehicles);

      const byId = new Map(frame.vehicles.map((v) => [v.id, v]));

      // ── Debug laser lines ──────────────────────────────────────────────────
      debugLayer.clear();

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
        const ds = 6;
        debugLayer
          .moveTo(ox, oy - ds).lineTo(ox + ds, oy)
          .lineTo(ox, oy + ds).lineTo(ox - ds, oy)
          .lineTo(ox, oy - ds)
          .stroke({ width: 1.5, color: COLOR.oncomingLine, alpha: 0.9 });
      }

      const activeIds = new Set(frame.vehicles.map((v) => v.id));
      for (const v of frame.vehicles) {
        if (leaderIds.has(v.id)) {
          const m = getOrCreateLeaderMarker(v.id);
          const { sx, sy } = toScreen(v.x, v.y);
          m.x = sx + VEHICLE_W / 2 + 6;
          m.y = sy - VEHICLE_H / 2 - 2;
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
      arcLayer.clear();
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
