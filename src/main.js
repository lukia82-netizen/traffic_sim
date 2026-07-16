// ─── Constants ────────────────────────────────────────────────────────────────

const VISUAL_SCALE   = 100;
const VEHICLE_W      = 22;   // vehicle length along heading [px]
const VEHICLE_H      = 13;   // vehicle width across heading [px]
const CANVAS_SIZE    = 800;
const CENTER         = CANVAS_SIZE / 2;
// World coords at the canvas border: ±(CANVAS_SIZE/2)/VISUAL_SCALE — keep in sync with Rust VIEWPORT_HALF_WORLD.

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
  intersection:  0xff33cc,  // legacy (unused); conflict debug uses conflictIdle / conflictActive
  conflictIdle:    0x888888,
  conflictActive:  0xff2244,
  roadAsphalt:     0x2a2a32,
  roadEdge:        0x4a4a58,
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
  const roadLayer    = new PIXI.Graphics(); // thick road strips from Rust node–segment graph
  const conflictLayer = new PIXI.Graphics(); // conflict points + active yield debug
  const pathsLayer   = new PIXI.Graphics(); // dashed paths from sim (toggleable)
  const tlLayer      = new PIXI.Graphics(); // traffic light indicators
  const arcLayer     = new PIXI.Graphics(); // active Bezier turn-path previews
  const debugLayer   = new PIXI.Graphics(); // leader / oncoming lines
  const dynamicLayer = new PIXI.Container();
  app.stage.addChild(sceneLayer);
  app.stage.addChild(roadLayer);
  app.stage.addChild(conflictLayer);
  app.stage.addChild(pathsLayer);
  app.stage.addChild(tlLayer);
  app.stage.addChild(arcLayer);
  app.stage.addChild(debugLayer);
  app.stage.addChild(dynamicLayer);

  pathsLayer.visible = false; // hidden by default; "Show Paths" button reveals it

  // ── "Show Paths" toggle button ─────────────────────────────────────────────
  const toggleBtn = document.createElement("button");
  toggleBtn.textContent = "Show Paths: OFF";
  toggleBtn.title = "Toggle dashed path geometry from the Rust sim + collision-matrix crossings.";
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

  // ── Path rendering (dashed; data from Rust `paths` + `intersections`) ───────

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

  function quadBezierWorld(sx, sy, cx, cy, ex, ey, t) {
    const u = 1 - t;
    return {
      x: u * u * sx + 2 * u * t * cx + t * t * ex,
      y: u * u * sy + 2 * u * t * cy + t * t * ey,
    };
  }

  function drawDashedQuadraticWorld(g, sx, sy, cx, cy, ex, ey, dashLen, gapLen, style) {
    const steps = 48;
    let px = null;
    let py = null;
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const w = quadBezierWorld(sx, sy, cx, cy, ex, ey, t);
      const p = toScreen(w.x, w.y);
      if (px != null) {
        drawDashedLine(g, px, py, p.sx, p.sy, dashLen, gapLen, style);
      }
      px = p.sx;
      py = p.sy;
    }
  }

  function drawPolylineDashedWorld(g, worldPts, dashLen, gapLen, style) {
    if (!worldPts || worldPts.length < 2) return;
    for (let i = 0; i < worldPts.length - 1; i++) {
      const a = toScreen(worldPts[i][0], worldPts[i][1]);
      const b = toScreen(worldPts[i + 1][0], worldPts[i + 1][1]);
      drawDashedLine(g, a.sx, a.sy, b.sx, b.sy, dashLen, gapLen, style);
    }
  }

  /** Filled road cross-section from `left_edge` / `right_edge` world polylines (normals baked in Rust). */
  function drawRoadStripPolygon(g, leftW, rightW) {
    if (!leftW || !rightW || leftW.length < 2 || rightW.length < 2) return;
    const p0 = toScreen(leftW[0][0], leftW[0][1]);
    g.moveTo(p0.sx, p0.sy);
    for (let i = 1; i < leftW.length; i++) {
      const p = toScreen(leftW[i][0], leftW[i][1]);
      g.lineTo(p.sx, p.sy);
    }
    for (let i = rightW.length - 1; i >= 0; i--) {
      const p = toScreen(rightW[i][0], rightW[i][1]);
      g.lineTo(p.sx, p.sy);
    }
    g.closePath();
    g.fill({ color: COLOR.roadAsphalt, alpha: 0.94 });
    g.stroke({ width: 1.2, color: COLOR.roadEdge, alpha: 0.55 });
  }

  function drawRoadVisuals(strips) {
    roadLayer.clear();
    if (!strips || !strips.length) return;
    const dashLane = { width: 1.1, color: 0xffffff, alpha: 0.2 };
    const dashDiv  = { width: 1, color: 0xdddddd, alpha: 0.32 };
    for (const s of strips) {
      drawRoadStripPolygon(roadLayer, s.left_edge, s.right_edge);
      for (const line of s.lane_center_polylines || []) {
        drawPolylineDashedWorld(roadLayer, line, 7, 6, dashLane);
      }
      for (const div of s.lane_dividers || []) {
        drawPolylineDashedWorld(roadLayer, div, 6, 5, dashDiv);
      }
    }
  }

  /** Dashed path geometry from Rust catalogue (toggle). */
  function drawPathsFromFrame(paths) {
    pathsLayer.clear();
    const dashLine = { width: 1.2, color: COLOR.pathLane, alpha: 0.4 };
    const dashBez  = { width: 1.2, color: COLOR.pathArc, alpha: 0.5 };
    const list = paths || [];
    for (const p of list) {
      if (p.kind === 0) {
        const a = toScreen(p.x0, p.y0);
        const b = toScreen(p.x1, p.y1);
        drawDashedLine(pathsLayer, a.sx, a.sy, b.sx, b.sy, 8, 6, dashLine);
      } else {
        drawDashedQuadraticWorld(
          pathsLayer,
          p.x0, p.y0, p.x1, p.y1, p.x2, p.y2,
          7, 5, dashBez,
        );
      }
    }
  }

  /** All conflict points (grey); active TTC yield (red + lines to both vehicles). */
  function drawConflictOverlay(conflictPoints, activeConflicts, vehicles) {
    conflictLayer.clear();
    const pts = conflictPoints || [];
    for (const pt of pts) {
      const s = toScreen(pt[0], pt[1]);
      conflictLayer.circle(s.sx, s.sy, 3.5).fill({ color: COLOR.conflictIdle, alpha: 0.38 });
    }
    const byId = new Map((vehicles || []).map((v) => [v.id, v]));
    const acts = activeConflicts || [];
    const lineStyle = { width: 1.8, color: COLOR.conflictActive, alpha: 0.75 };
    for (const a of acts) {
      const s = toScreen(a.x, a.y);
      conflictLayer.circle(s.sx, s.sy, 6).fill({ color: COLOR.conflictActive, alpha: 0.88 });
      const yv = byId.get(a.yielder_id);
      const ov = byId.get(a.other_id);
      if (yv && ov) {
        const py = toScreen(yv.x, yv.y);
        const po = toScreen(ov.x, ov.y);
        conflictLayer.moveTo(s.sx, s.sy).lineTo(py.sx, py.sy).stroke(lineStyle);
        conflictLayer.moveTo(s.sx, s.sy).lineTo(po.sx, po.sy).stroke(lineStyle);
      }
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

  // ── Active turn-arc layer (vehicle on Bezier path_id from Rust) ───────────

  function drawTurnArcs(vehicles, paths) {
    arcLayer.clear();
    const byId = new Map((paths || []).map((p) => [p.id, p]));
    for (const v of vehicles) {
      if (!v.on_bezier_turn) continue;
      const def = byId.get(v.path_id);
      if (!def || def.kind !== 1) continue;

      const s   = toScreen(def.x0, def.y0);
      const c   = toScreen(def.x1, def.y1);
      const e   = toScreen(def.x2, def.y2);
      const cur = toScreen(v.x, v.y);
      const pr  = v.path_progress;

      arcLayer
        .moveTo(s.sx, s.sy)
        .quadraticCurveTo(c.sx, c.sy, e.sx, e.sy)
        .stroke({ width: 2.5, color: COLOR.pathArcActive, alpha: 0.55 });

      const tR = 1.0 - Math.min(pr, 1);
      const mcx = c.sx + tR * (e.sx - c.sx);
      const mcy = c.sy + tR * (e.sy - c.sy);
      arcLayer
        .moveTo(cur.sx, cur.sy)
        .quadraticCurveTo(mcx, mcy, e.sx, e.sy)
        .stroke({ width: 2, color: COLOR.pathArcActive, alpha: 0.85 });

      arcLayer.circle(cur.sx, cur.sy, 3.5).fill({ color: COLOR.pathArcActive, alpha: 0.9 });
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
      "V:" + v.speed.toFixed(2) + " P:" + (v.path_progress != null ? v.path_progress.toFixed(2) : "?");
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
  let staticRoadDrawn  = false;

  try {
    const { listen } = window.__TAURI__.event;

    await listen("sim-frame", (event) => {
      const frame = event.payload;

      if (!staticSceneDrawn) {
        drawStaticScene(frame.approach_radius, frame.clear_radius, frame.stop_line_offset);
        staticSceneDrawn = true;
      }

      if (!staticRoadDrawn && frame.road_visuals && frame.road_visuals.length) {
        drawRoadVisuals(frame.road_visuals);
        staticRoadDrawn = true;
      }

      if (pathsLayer.visible) {
        drawPathsFromFrame(frame.paths);
      }

      drawConflictOverlay(
        frame.conflict_points || frame.intersections,
        frame.active_conflicts,
        frame.vehicles,
      );

      drawTrafficLights(frame.light_ns, frame.light_ew, frame.stop_line_offset);
      drawTurnArcs(frame.vehicles, frame.paths);

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
      conflictLayer.clear();
      roadLayer.clear();
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
