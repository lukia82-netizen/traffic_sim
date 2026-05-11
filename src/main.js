// ─── Constants ────────────────────────────────────────────────────────────────

const VISUAL_SCALE = 100;
const VEHICLE_RADIUS = 10;
const CANVAS_SIZE = 800;
const CENTER = CANVAS_SIZE / 2;

const COLOR = {
  cruising: 0x00cc44,
  crossing: 0x00cc44,
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
    const ap = approachRadius * VISUAL_SCALE;
    const cr = clearRadius * VISUAL_SCALE;

    sceneLayer
      .circle(CENTER, CENTER, ap)
      .stroke({ width: 1.5, color: COLOR.approach, alpha: 0.7 });

    sceneLayer
      .circle(CENTER, CENTER, cr)
      .stroke({ width: 1.5, color: COLOR.clear, alpha: 0.7 });

    const arm = 20;
    sceneLayer
      .moveTo(CENTER - arm, CENTER)
      .lineTo(CENTER + arm, CENTER)
      .stroke({ width: 2, color: COLOR.cross });
    sceneLayer
      .moveTo(CENTER, CENTER - arm)
      .lineTo(CENTER, CENTER + arm)
      .stroke({ width: 2, color: COLOR.cross });
  }

  // 5. Dynamic vehicle layer
  const dynamicLayer = new PIXI.Container();
  app.stage.addChild(dynamicLayer);

  const vehicleGraphics = new Map();
  const vehicleLabels   = new Map();

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
    }
    return { g: vehicleGraphics.get(id), label: vehicleLabels.get(id) };
  }

  function updateVehicle(v) {
    const { g, label } = getOrCreateVehicle(v.id);
    const { sx, sy } = toScreen(v.x, v.y);
    const color = v.status === "waiting" ? COLOR.waiting : COLOR.cruising;

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
  }

  // 6. FPS counter
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

  // 7. Subscribe to Tauri events
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

      // Collect ids present this frame to remove departed vehicles.
      const activeIds = new Set(frame.vehicles.map((v) => v.id));
      for (const [id, g] of vehicleGraphics) {
        if (!activeIds.has(id)) {
          dynamicLayer.removeChild(g);
          vehicleGraphics.delete(id);
          const label = vehicleLabels.get(id);
          if (label) { dynamicLayer.removeChild(label); vehicleLabels.delete(id); }
        }
      }

      for (const vehicle of frame.vehicles) {
        updateVehicle(vehicle);
      }

      updateStats(frame);
    });

    await listen("sim-done", () => {
      // Clear all remaining vehicle graphics.
      for (const g of vehicleGraphics.values()) dynamicLayer.removeChild(g);
      for (const l of vehicleLabels.values()) dynamicLayer.removeChild(l);
      vehicleGraphics.clear();
      vehicleLabels.clear();

      statsEl.style.color = "#44ff88";
      statsEl.textContent = "Simulation complete — all 4 vehicles crossed.";
    });

    console.log("Listening for sim-frame / sim-done events...");
  } catch (err) {
    showError("listen() failed: " + err);
  }
}

main();
