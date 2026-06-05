/* Motion Planner Web Demo - frontend
 *
 * Tools:
 *   - polygon: click to add vertices, double-click or Enter to close
 *   - start  : click and drag the green marker
 *   - goal   : click and drag the red marker
 *   - erase  : click a polygon (or vertex) to delete it
 *
 * The world frame is (xmin, ymin) .. (xmax, ymax).  The screen frame is
 * (0, 0) top-left -> (W, H) bottom-right with Y flipped.
 */

(() => {
  "use strict";

  // ----------------------------------------------------------- DOM handles
  const mainCv  = document.getElementById("main");
  const mainCtx = mainCv.getContext("2d");
  const stageCvs = Array.from(document.querySelectorAll("canvas.stage"));
  const logEl   = document.getElementById("log");
  const hintEl  = document.getElementById("hint");

  const btnPlan   = document.getElementById("btn-plan");
  const btnAnim   = document.getElementById("btn-animate");
  const btnStop   = document.getElementById("btn-stop");
  const btnReset  = document.getElementById("btn-reset");
  const btnPreset = document.getElementById("btn-preset");

  const toolBtns = Array.from(document.querySelectorAll(".tool"));

  const fpShape = document.getElementById("fp-shape");
  const fpW     = document.getElementById("fp-w");
  const fpH     = document.getElementById("fp-h");
  const fpR     = document.getElementById("fp-r");
  const presetSelect = document.getElementById("preset-select");

  const startTheta    = document.getElementById("start-theta");
  const startThetaVal = document.getElementById("start-theta-val");
  const goalTheta     = document.getElementById("goal-theta");
  const goalThetaVal  = document.getElementById("goal-theta-val");

  const showAstar = document.getElementById("show-astar");
  const showSC    = document.getElementById("show-sc");
  const showRS2   = document.getElementById("show-rs2");
  const showSM    = document.getElementById("show-sm");
  const showRS    = document.getElementById("show-rs");
  const showRSAnchor = document.getElementById("show-rsanchor");
  const showFinal = document.getElementById("show-final");
  const showFP    = document.getElementById("show-fp");
  const showESDF  = document.getElementById("show-esdf");
  const esdfAlpha = document.getElementById("esdf-alpha");

  const paramsBody   = document.getElementById("params-body");
  const btnResetCfg  = document.getElementById("btn-reset-config");

  // --------------------------------------------------------------- state
  const state = {
    bounds: [-3, -3, 3, 3],
    polygons: [],
    start: { x: -2.5, y: 0.0, theta: 0.0 },
    goal:  { x:  2.5, y: 0.0, theta: 0.0 },
    tool: "polygon",
    draw: { verts: [], hover: null },  // active polygon in progress
    drag: null,                         // null | "start" | "goal"
    result: null,                       // last plan() response
    animation: { active: false, t: 0, traj: null },
    esdfCanvas: null,                   // off-screen heatmap canvas
    esdfMax: 0,                         // max abs distance for colour scale
    config: {},                         // PlannerConfig overrides, keyed by name
    configSchema: null,                 // schema fetched from /api/config
  };

  const MARKER_R = 9;          // px radius for start/goal hit test
  const DRAG_R   = 14;         // px hit radius

  // ----------------------------------------------------------- utilities
  function log(msg, cls = "") {
    const line = document.createElement("div");
    if (cls) line.className = "log-" + cls;
    const ts = new Date().toLocaleTimeString();
    line.textContent = `[${ts}] ${msg}`;
    logEl.appendChild(line);
    logEl.scrollTop = logEl.scrollHeight;
  }

  function setHint(msg) { hintEl.textContent = msg; }

  function worldToScreen(x, y, cv) {
    const W = cv.width, H = cv.height;
    const [xmin, ymin, xmax, ymax] = state.bounds;
    const sx = (x - xmin) / (xmax - xmin) * W;
    const sy = H - (y - ymin) / (ymax - ymin) * H;
    return [sx, sy];
  }

  function screenToWorld(sx, sy, cv) {
    const W = cv.width, H = cv.height;
    const [xmin, ymin, xmax, ymax] = state.bounds;
    const x = xmin + (sx / W) * (xmax - xmin);
    const y = ymin + ((H - sy) / H) * (ymax - ymin);
    return [x, y];
  }

  function footprintSpec() {
    if (fpShape.value === "circle") {
      return { shape: "circle", radius: Number(fpR.value) };
    }
    return { shape: "rect", width: Number(fpW.value), height: Number(fpH.value) };
  }

  function setStatus(l1, l2, l3, length, ms) {
    const fill = (el, v) => {
      if (v == null) { el.textContent = "–"; el.className = ""; return; }
      el.textContent = v.ok
        ? `OK (clr ${v.min_clearance.toFixed(3)})`
        : `FAIL — ${v.message}`;
      el.className = v.ok ? "ok" : "bad";
    };
    fill(document.getElementById("st-l1"), l1);
    fill(document.getElementById("st-l2"), l2);
    fill(document.getElementById("st-l3"), l3);
    document.getElementById("st-len").textContent  = length != null ? length.toFixed(2) + " m" : "–";
    document.getElementById("st-time").textContent  = ms != null     ? ms.toFixed(0)    + " ms" : "–";
  }

  // ----------------------------------------------------------- drawing
  function drawWorld(cv, ctx) {
    const W = cv.width, H = cv.height;
    ctx.clearRect(0, 0, W, H);

    // grid
    ctx.strokeStyle = "#1f2630";
    ctx.lineWidth = 1;
    ctx.beginPath();
    const [xmin, ymin, xmax, ymax] = state.bounds;
    const step = niceStep((xmax - xmin) / 8);
    for (let x = Math.ceil(xmin / step) * step; x <= xmax; x += step) {
      const [sx] = worldToScreen(x, 0, cv);
      ctx.moveTo(sx, 0); ctx.lineTo(sx, H);
    }
    for (let y = Math.ceil(ymin / step) * step; y <= ymax; y += step) {
      const [, sy] = worldToScreen(0, y, cv);
      ctx.moveTo(0, sy); ctx.lineTo(W, sy);
    }
    ctx.stroke();

    // axes
    ctx.strokeStyle = "#3a4757";
    const [ox0, oy0] = worldToScreen(xmin, 0, cv);
    const [ox1, oy1] = worldToScreen(xmax, 0, cv);
    ctx.beginPath(); ctx.moveTo(ox0, oy0); ctx.lineTo(ox1, oy1); ctx.stroke();
    const [, ay0] = worldToScreen(0, ymin, cv);
    const [, ay1] = worldToScreen(0, ymax, cv);
    ctx.beginPath(); ctx.moveTo(0, ay0); ctx.lineTo(W, ay1); ctx.stroke();

    // bounds label
    ctx.fillStyle = "#6e7681";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText(`x:[${xmin.toFixed(1)}, ${xmax.toFixed(1)}]`, 6, H - 6);
    ctx.fillText(`y:[${ymin.toFixed(1)}, ${ymax.toFixed(1)}]`, 6, 14);
  }

  function drawObstacles(ctx, cv) {
    ctx.fillStyle = "#f0f6fc";
    ctx.strokeStyle = "#8b949e";
    ctx.lineWidth = 1;
    for (const poly of state.polygons) {
      if (poly.length < 2) continue;
      ctx.beginPath();
      poly.forEach(([x, y], i) => {
        const [sx, sy] = worldToScreen(x, y, cv);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
  }

  // ----------------------------------------------------------- ESDF heatmap
  // Build an off-screen canvas at the ESDF grid resolution and colour each
  // cell by signed distance: free space goes red -> yellow -> green -> blue
  // as you get further from obstacles; cells inside obstacles fade to dark
  // red.  The canvas is rebuilt only when /api/plan returns new data.
  function buildEsdfCanvas(esdf) {
    if (!esdf || !esdf.values || !esdf.shape) return null;
    const [nx, ny] = esdf.shape;
    const vals = esdf.values;
    if (vals.length !== nx * ny) return null;
    // colour scale = max abs distance in the field, clamped to a sensible
    // upper bound so far-away cells don't all collapse to the same blue
    let maxAbs = 0;
    for (let i = 0; i < vals.length; i++) {
      const a = Math.abs(vals[i]);
      if (a > maxAbs) maxAbs = a;
    }
    maxAbs = Math.max(0.1, Math.min(maxAbs, 2.0));
    state.esdfMax = maxAbs;

    const off = document.createElement("canvas");
    off.width = nx;
    off.height = ny;
    const octx = off.getContext("2d");
    const img = octx.createImageData(nx, ny);
    for (let gy = 0; gy < ny; gy++) {
      // flip Y so world +y is canvas up
      const srcRow = ny - 1 - gy;
      for (let gx = 0; gx < nx; gx++) {
        const d = vals[srcRow * nx + gx];
        const [r, g, b] = esdfColor(d, maxAbs);
        const off4 = (gy * nx + gx) * 4;
        img.data[off4 + 0] = r;
        img.data[off4 + 1] = g;
        img.data[off4 + 2] = b;
        img.data[off4 + 3] = 255;
      }
    }
    octx.putImageData(img, 0, 0);
    return off;
  }

  function esdfColor(d, dmax) {
    // d < 0 : inside obstacle, dark red -> red
    // d > 0 : free space, red -> yellow -> green -> cyan -> blue
    if (d <= 0) {
      const t = Math.max(-1, d / dmax);     // -1 .. 0
      const v = 60 + (255 - 60) * (1 + t);   // 60 (deep) .. 255 (boundary)
      return [Math.round(v), 0, 0];
    }
    const t = Math.min(1, d / dmax);         // 0 .. 1
    // hue 0 (red) -> 240 (blue) through 60 (yellow), 120 (green), 180 (cyan)
    return hslToRgb(t * 240, 1.0, 0.5);
  }

  function hslToRgb(h, s, l) {
    const c = (1 - Math.abs(2 * l - 1)) * s;
    const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
    const m = l - c / 2;
    let r = 0, g = 0, b = 0;
    if (h <  60) { r = c; g = x; }
    else if (h < 120) { r = x; g = c; }
    else if (h < 180) { g = c; b = x; }
    else if (h < 240) { g = x; b = c; }
    else if (h < 300) { r = x; b = c; }
    else              { r = c; b = x; }
    return [
      Math.round((r + m) * 255),
      Math.round((g + m) * 255),
      Math.round((b + m) * 255),
    ];
  }

  function drawEsdfHeatmap(ctx, cv) {
    if (!state.esdfCanvas || !state.result || !state.result.esdf) return;
    if (!showESDF.checked) return;
    const esdf = state.result.esdf;
    const [exmin, eymin, exmax, eymax] = esdf.bounds;
    // map ESDF bounds rectangle to canvas pixel rectangle in *world* space.
    // The ESDF bounds and the canvas world bounds (state.bounds) are not
    // guaranteed identical (the ESDF tightly hugs the map bounds, while
    // state.bounds is the editor bounds).  Use worldToScreen for both
    // corners so the heatmap lands exactly on the obstacles it represents.
    const [sx0, sy1] = worldToScreen(exmin, eymin, cv);  // bottom-left world
    const [sx1, sy0] = worldToScreen(exmax, eymax, cv);  // top-right world
    const dw = sx1 - sx0;
    const dh = sy1 - sy0;
    const a = Number(esdfAlpha.value) / 100;
    ctx.save();
    ctx.globalAlpha = isFinite(a) ? a : 0.7;
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(state.esdfCanvas, sx0, sy0, dw, dh);
    ctx.restore();
  }

  function drawPath(ctx, cv, points, color, width = 2) {
    if (!points || points.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineJoin = "round";
    ctx.beginPath();
    points.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(x, y, cv);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  function drawPathWithDots(ctx, cv, points, color, width = 1.5, dotR = 2) {
    if (!points || points.length < 1) return;
    if (points.length >= 2) drawPath(ctx, cv, points, color, width);
    ctx.fillStyle = color;
    for (const [x, y] of points) {
      const [sx, sy] = worldToScreen(x, y, cv);
      ctx.beginPath();
      ctx.arc(sx, sy, dotR, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  function drawTrajectory(ctx, cv, traj, color, width = 2.5) {
    if (!traj || traj.length < 2) return;
    drawPath(ctx, cv, traj.map(p => [p[0], p[1]]), color, width);
    // heading arrows every ~10%
    const stride = Math.max(1, Math.floor(traj.length / 10));
    ctx.fillStyle = color;
    for (let i = 0; i < traj.length; i += stride) {
      const [x, y, th] = traj[i];
      const [sx, sy] = worldToScreen(x, y, cv);
      const dx = Math.cos(th) * 6;
      const dy = -Math.sin(th) * 6;
      ctx.beginPath();
      ctx.moveTo(sx + dx, sy + dy);
      ctx.lineTo(sx - dx * 0.5 - dy * 0.5, sy - dy * 0.5 + dx * 0.5);
      ctx.lineTo(sx - dx * 0.5 + dy * 0.5, sy - dy * 0.5 - dx * 0.5);
      ctx.closePath();
      ctx.fill();
    }
  }

  // Draw the RS planner's anchor polyline as filled circles at each
  // (x, y, theta), with a short heading tick.  Anchors are the discrete
  // waypoints that the Dubins interpolation is stitched between, so
  // showing them makes the path's piecewise structure obvious.
  function drawRSAnchors(ctx, cv, anchors) {
    if (!anchors || anchors.length === 0) return;
    const [xmin, ymin, xmax, ymax] = state.bounds;
    const sxPerM = cv.width / (xmax - xmin);
    for (const a of anchors) {
      const x = a[0], y = a[1], th = a.length > 2 ? a[2] : 0;
      const [sx, sy] = worldToScreen(x, y, cv);
      ctx.beginPath();
      ctx.arc(sx, sy, 4, 0, Math.PI * 2);
      ctx.fillStyle = "#f78166";
      ctx.fill();
      ctx.strokeStyle = "white";
      ctx.lineWidth = 1;
      ctx.stroke();
      // short heading tick (length in metres -> px)
      const len = 8 / sxPerM;
      const tx = sx + Math.cos(th) * len * sxPerM;
      const ty = sy - Math.sin(th) * len * sxPerM;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(tx, ty);
      ctx.strokeStyle = "#f78166";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }

  function drawPoseFootprint(ctx, cv, x, y, theta) {
    const fp = footprintSpec();
    ctx.save();
    const [sx, sy] = worldToScreen(x, y, cv);
    ctx.translate(sx, sy);
    const [xmin, ymin, xmax, ymax] = state.bounds;
    const sxPerM = cv.width / (xmax - xmin);
    ctx.scale(sxPerM, -sxPerM);  // also flip Y so theta matches world frame
    ctx.rotate(theta);
    if (fp.shape === "circle") {
      ctx.beginPath();
      ctx.arc(0, 0, fp.radius, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(63, 185, 80, 0.25)";
      ctx.strokeStyle = "rgba(63, 185, 80, 0.9)";
      ctx.lineWidth = 1.5 / sxPerM;
      ctx.fill(); ctx.stroke();
    } else {
      const w = fp.width, h = fp.height;
      ctx.beginPath();
      ctx.rect(-w / 2, -h / 2, w, h);
      ctx.fillStyle = "rgba(63, 185, 80, 0.25)";
      ctx.strokeStyle = "rgba(63, 185, 80, 0.9)";
      ctx.lineWidth = 1.5 / sxPerM;
      ctx.fill(); ctx.stroke();
      // heading indicator
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(w / 2, 0);
      ctx.strokeStyle = "rgba(63, 185, 80, 1.0)";
      ctx.lineWidth = 2 / sxPerM;
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawFootprintSamples(ctx, cv) {
    if (!state.result) return;
    const traj = state.result.stages.final;
    if (!traj || !traj.length) return;
    const stride = Math.max(1, Math.floor(traj.length / 12));
    for (let i = 0; i < traj.length; i += stride) {
      const [x, y, th] = traj[i];
      drawPoseFootprint(ctx, cv, x, y, th);
    }
  }

  function drawStartGoal(ctx, cv) {
    for (const [pt, color, label] of [
      [state.start, "#3fb950", "S"],
      [state.goal,  "#f85149", "G"],
    ]) {
      const [sx, sy] = worldToScreen(pt.x, pt.y, cv);
      ctx.beginPath();
      ctx.arc(sx, sy, MARKER_R, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.stroke();
      // heading tick
      const hx = sx + Math.cos(pt.theta) * (MARKER_R + 4);
      const hy = sy - Math.sin(pt.theta) * (MARKER_R + 4);
      ctx.beginPath();
      ctx.moveTo(sx, sy); ctx.lineTo(hx, hy);
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.stroke();
      // label
      ctx.fillStyle = "white";
      ctx.font = "bold 10px ui-monospace, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(label, sx, sy);
    }
  }

  function drawDrawingPolygon(ctx, cv) {
    const verts = state.draw.verts;
    if (verts.length === 0 && !state.draw.hover) return;
    ctx.strokeStyle = "#d29922";
    ctx.fillStyle   = "rgba(210, 153, 34, 0.4)";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    verts.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(x, y, cv);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    if (state.draw.hover) {
      const [hx, hy] = worldToScreen(state.draw.hover[0], state.draw.hover[1], cv);
      ctx.lineTo(hx, hy);
    }
    if (verts.length >= 2) {
      // close preview
      const [fx, fy] = worldToScreen(verts[0][0], verts[0][1], cv);
      ctx.lineTo(fx, fy);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    // vertices
    ctx.fillStyle = "#d29922";
    for (const [x, y] of verts) {
      const [sx, sy] = worldToScreen(x, y, cv);
      ctx.beginPath();
      ctx.arc(sx, sy, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  function drawAnimation(cv, ctx) {
    if (!state.animation.active || !state.animation.traj) return;
    const pose = interpolateTraj(state.animation.traj, state.animation.t);
    if (!pose) return;
    drawPoseFootprint(ctx, cv, pose[0], pose[1], pose[2]);
  }

  function redrawMain() {
    drawWorld(mainCv, mainCtx);
    const ctx = mainCv.getContext("2d");
    drawEsdfHeatmap(ctx, mainCv);
    drawObstacles(ctx, mainCv);
    if (state.result) {
      if (showAstar.checked) drawPath(ctx, mainCv, state.result.stages.raw_astar, "#d29922", 1.5);
      if (showSC.checked)    drawPath(ctx, mainCv, state.result.stages.shortcut, "#db61a2", 1.5);
      if (showRS2.checked)   drawPathWithDots(ctx, mainCv, state.result.stages.resampled, "#ffa657", 1.5, 2);
      if (showSM.checked)    drawPath(ctx, mainCv, state.result.stages.smoothed_xy, "#58a6ff", 1.8);
      if (showRS.checked)    drawTrajectory(ctx, mainCv, state.result.stages.rs, "#bc8cff", 1.8);
      if (showRSAnchor.checked) drawRSAnchors(ctx, mainCv, state.result.stages.rs_anchors);
      if (showFinal.checked) drawTrajectory(ctx, mainCv, state.result.stages.final, "#a5d6ff", 2.5);
      if (showFP.checked)    drawFootprintSamples(ctx, mainCv);
    }
    drawStartGoal(ctx, mainCv);
    drawDrawingPolygon(ctx, mainCv);
    drawAnimation(mainCv, ctx);
  }

  function redrawStages() {
    const ctxMap = {
      astar:     () => state.result ? state.result.stages.raw_astar : null,
      shortcut:  () => state.result ? state.result.stages.shortcut : null,
      resampled: () => state.result ? state.result.stages.resampled : null,
      smoothed:  () => state.result ? state.result.stages.smoothed_xy : null,
      rs:        () => state.result ? state.result.stages.rs : null,
      final:     () => state.result ? state.result.stages.final : null,
    };
    const colorMap = {
      astar: "#d29922", shortcut: "#db61a2", resampled: "#ffa657",
      smoothed: "#58a6ff", rs: "#bc8cff", final: "#a5d6ff",
    };
    for (const cv of stageCvs) {
      const key = cv.dataset.stage;
      const ctx = cv.getContext("2d");
      drawWorld(cv, ctx);
      drawObstacles(ctx, cv);
      drawStartGoal(ctx, cv);
      const data = ctxMap[key]();
      if (!data) continue;
      if (key === "final" || key === "rs") {
        drawTrajectory(ctx, cv, data, colorMap[key], 2);
        if (key === "rs" && showRSAnchor.checked && state.result) {
          drawRSAnchors(ctx, cv, state.result.stages.rs_anchors);
        }
        if (key === "final" && showFP.checked) {
          const stride = Math.max(1, Math.floor(data.length / 8));
          for (let i = 0; i < data.length; i += stride) {
            drawPoseFootprint(ctx, cv, data[i][0], data[i][1], data[i][2]);
          }
        }
      } else if (key === "resampled") {
        drawPathWithDots(ctx, cv, data, colorMap[key], 1.5, 2);
      } else {
        drawPath(ctx, cv, data, colorMap[key], 2);
      }
    }
  }

  // ----------------------------------------------------- trajectory utils
  function trajLength(traj) {
    let L = 0;
    for (let i = 1; i < traj.length; i++) {
      const dx = traj[i][0] - traj[i-1][0];
      const dy = traj[i][1] - traj[i-1][1];
      L += Math.hypot(dx, dy);
    }
    return L;
  }

  function interpolateTraj(traj, t) {
    if (!traj || traj.length === 0) return null;
    if (traj.length === 1) return traj[0];
    if (t <= 0) return traj[0];
    if (t >= 1) return traj[traj.length - 1];
    // map t to segment index by arc length
    const L = trajLength(traj);
    if (L < 1e-6) return traj[0];
    const target = t * L;
    let acc = 0;
    for (let i = 1; i < traj.length; i++) {
      const seg = Math.hypot(traj[i][0] - traj[i-1][0],
                             traj[i][1] - traj[i-1][1]);
      if (acc + seg >= target) {
        const a = (target - acc) / Math.max(seg, 1e-9);
        const x = traj[i-1][0] + a * (traj[i][0] - traj[i-1][0]);
        const y = traj[i-1][1] + a * (traj[i][1] - traj[i-1][1]);
        // unwrap heading to avoid 2pi jumps
        let th1 = traj[i-1][2], th2 = traj[i][2];
        while (th2 - th1 >  Math.PI) th2 -= 2 * Math.PI;
        while (th2 - th1 < -Math.PI) th2 += 2 * Math.PI;
        const th = th1 + a * (th2 - th1);
        return [x, y, th];
      }
      acc += seg;
    }
    return traj[traj.length - 1];
  }

  function niceStep(target) {
    // pick a "nice" grid step close to target
    const pow = Math.pow(10, Math.floor(Math.log10(target)));
    const n = target / pow;
    let nice;
    if      (n < 1.5) nice = 1;
    else if (n < 3.5) nice = 2;
    else if (n < 7.5) nice = 5;
    else              nice = 10;
    return nice * pow;
  }

  // ----------------------------------------------------------- input
  function pointerPos(ev) {
    const rect = mainCv.getBoundingClientRect();
    const sx = (ev.clientX - rect.left) * (mainCv.width  / rect.width);
    const sy = (ev.clientY - rect.top ) * (mainCv.height / rect.height);
    return { sx, sy, world: screenToWorld(sx, sy, mainCv) };
  }

  function hitMarker(pt) {
    const [sx, sy] = worldToScreen(pt.x, pt.y, mainCv);
    return Math.hypot(sx, sy);  // used in pointerPos comparison
  }

  function nearestPose(p) {
    const [sx0, sy0] = worldToScreen(state.start.x, state.start.y, mainCv);
    const [sx1, sy1] = worldToScreen(state.goal.x,  state.goal.y,  mainCv);
    const d0 = Math.hypot(p.sx - sx0, p.sy - sy0);
    const d1 = Math.hypot(p.sx - sx1, p.sy - sy1);
    return d0 <= d1
      ? { name: "start", dist: d0 }
      : { name: "goal",  dist: d1 };
  }

  function movePoseTo(name, wx, wy) {
    state[name].x = wx; state[name].y = wy;
    state.result = null;
    setStatus(null, null, null, null, null);
    redrawMain(); redrawStages();
    schedulePlan();
  }

  mainCv.addEventListener("mousedown", (ev) => {
    const p = pointerPos(ev);
    if (state.tool === "polygon") {
      state.draw.verts.push(p.world);
      redrawMain();
      return;
    }
    if (state.tool === "move") {
      // pick the closest marker; if the click lands inside its drag radius,
      // begin a drag, otherwise teleport it to the click.
      const m = nearestPose(p);
      state.drag = m.name;
      if (m.dist > DRAG_R) movePoseTo(m.name, p.world[0], p.world[1]);
      return;
    }
    if (state.tool === "erase") {
      // hit-test: polygon contains point
      for (let i = state.polygons.length - 1; i >= 0; i--) {
        const poly = state.polygons[i];
        if (pointInPolygon(p.world, poly)) {
          state.polygons.splice(i, 1);
          state.result = null;
          setStatus(null, null, null, null, null);
          redrawMain(); redrawStages();
          schedulePlan();
          return;
        }
      }
    }
  });

  mainCv.addEventListener("mousemove", (ev) => {
    const p = pointerPos(ev);
    if (state.drag) {
      movePoseTo(state.drag, p.world[0], p.world[1]);
      return;
    }
    if (state.tool === "polygon" && state.draw.verts.length > 0) {
      state.draw.hover = p.world;
      redrawMain();
    }
  });

  window.addEventListener("mouseup", () => {
    if (state.drag) {
      state.drag = null;
    }
  });

  mainCv.addEventListener("dblclick", (ev) => {
    if (state.tool === "polygon" && state.draw.verts.length >= 3) {
      finishPolygon();
    }
  });

  window.addEventListener("keydown", (ev) => {
    if (ev.key === "Enter" && state.tool === "polygon" && state.draw.verts.length >= 3) {
      finishPolygon();
    }
    if (ev.key === "Escape") {
      state.draw.verts = []; state.draw.hover = null;
      redrawMain();
    }
  });

  function finishPolygon() {
    if (state.draw.verts.length < 3) return;
    state.polygons.push(state.draw.verts.slice());
    state.draw.verts = []; state.draw.hover = null;
    state.result = null;
    setStatus(null, null, null, null, null);
    redrawMain(); redrawStages();
    log(`polygon added (${state.polygons[state.polygons.length - 1].length} verts)`);
    schedulePlan();
  }

  function pointInPolygon(pt, poly) {
    // ray casting; poly is [[x,y], ...]
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i][0], yi = poly[i][1];
      const xj = poly[j][0], yj = poly[j][1];
      const intersect = ((yi > pt[1]) !== (yj > pt[1])) &&
        (pt[0] < (xj - xi) * (pt[1] - yi) / ((yj - yi) || 1e-12) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  // ----------------------------------------------------------- tool btns
  for (const btn of toolBtns) {
    btn.addEventListener("click", () => {
      state.tool = btn.dataset.tool;
      for (const b of toolBtns) b.classList.remove("active");
      btn.classList.add("active");
      if (state.tool !== "polygon") {
        state.draw.verts = []; state.draw.hover = null;
      }
      const hints = {
        polygon: "Click to place polygon vertices. Double-click (or press Enter) to close. Esc to cancel.",
        move:    "Click to relocate the nearest pose (start/goal), or drag its marker.",
        erase:   "Click a polygon to delete it.",
      };
      setHint(hints[state.tool]);
      redrawMain();
    });
  }

  // ----------------------------------------------------------- plan / api
  let _planTimer = null;
  let _planning = false;
  const PLAN_DEBOUNCE_MS = 300;

  async function checkHealth() {
    try {
      const r = await fetch("/api/health");
      const j = await r.json();
      const dot = document.getElementById("health-dot");
      const txt = document.getElementById("health-text");
      if (j.ok) { dot.className = "dot dot-green"; txt.textContent = "online"; }
      else      { dot.className = "dot dot-red";   txt.textContent = "error"; }
    } catch {
      const dot = document.getElementById("health-dot");
      const txt = document.getElementById("health-text");
      dot.className = "dot dot-red";
      txt.textContent = "offline";
    }
  }

  async function loadPresets() {
    try {
      const r = await fetch("/api/presets");
      const j = await r.json();
      presetSelect.innerHTML = '<option value="">-- choose --</option>';
      for (const p of j.presets) {
        const opt = document.createElement("option");
        opt.value = String(j.presets.indexOf(p));
        opt.textContent = p.name;
        presetSelect.appendChild(opt);
      }
      window.__PRESETS__ = j.presets;
    } catch (e) {
      log(`failed to load presets: ${e}`, "err");
    }
  }

  btnPreset.addEventListener("click", () => {
    const i = Number(presetSelect.value);
    if (Number.isNaN(i) || i < 0 || !window.__PRESETS__ || !window.__PRESETS__[i]) {
      log("select a preset first", "err");
      return;
    }
    const p = window.__PRESETS__[i];
    state.bounds = p.bounds.slice();
    state.polygons = p.polygons.map(poly => poly.map(p => p.slice()));
    state.start = Object.assign({}, p.start);
    state.goal  = Object.assign({}, p.goal);
    state.draw.verts = []; state.draw.hover = null;
    state.result = null;
    setStatus(null, null, null, null, null);
    syncThetaSlider(startTheta, startThetaVal, state.start);
    syncThetaSlider(goalTheta,  goalThetaVal,  state.goal);
    redrawMain(); redrawStages();
    log(`loaded preset "${p.name}"`);
    schedulePlan();
  });

  async function doPlan() {
    if (_planning) return;
    _planning = true;
    btnPlan.disabled = true;
    setHint("planning…");
    try {
      const body = {
        bounds: state.bounds,
        polygons: state.polygons,
        start: state.start,
        goal:  state.goal,
        footprint: footprintSpec(),
      };
      if (state.config && Object.keys(state.config).length > 0) {
        body.config = state.config;
      }
      const r = await fetch("/api/plan", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const j = await r.json();
      if (!j.ok) {
        log(`plan failed: ${j.error}`, "err");
        setHint(`error: ${j.error}`);
        return;
      }
      state.result = j;
      state.esdfCanvas = buildEsdfCanvas(j.esdf);
      const finalPts = j.stages.final;
      const L = trajLength(finalPts);
      setStatus(j.validation.l1, j.validation.l2, j.validation.l3, L, j.elapsed_ms);
      log(`plan OK in ${j.elapsed_ms.toFixed(0)} ms, length ${L.toFixed(2)} m, `
        + `L3=${j.validation.l3.ok ? "ok" : "FAIL"}`);
      setHint("Plan ready. Use Display toggles to switch layers; Animate to play.");
    } catch (e) {
      log(`network error: ${e}`, "err");
      setHint(`network error: ${e}`);
    } finally {
      _planning = false;
      btnPlan.disabled = false;
      redrawMain(); redrawStages();
    }
  }

  function schedulePlan(immediate) {
    if (_planTimer) { clearTimeout(_planTimer); _planTimer = null; }
    if (immediate) { doPlan(); return; }
    _planTimer = setTimeout(() => { _planTimer = null; doPlan(); }, PLAN_DEBOUNCE_MS);
  }

  btnPlan.addEventListener("click", () => schedulePlan(true));

  // ----------------------------------------------------------- animation
  let animRAF = null;
  let animStart = 0;
  const ANIM_SECONDS = 5.0;

  function startAnimation() {
    if (!state.result || !state.result.stages.final.length) {
      log("nothing to animate — plan first", "err");
      return;
    }
    if (state.animation.active) return;
    state.animation.active = true;
    state.animation.traj = state.result.stages.final;
    state.animation.t = 0;
    animStart = performance.now();
    const tick = (now) => {
      if (!state.animation.active) return;
      const dt = (now - animStart) / 1000.0;
      state.animation.t = Math.min(1, dt / ANIM_SECONDS);
      redrawMain();
      if (state.animation.t < 1) {
        animRAF = requestAnimationFrame(tick);
      } else {
        state.animation.active = false;
        animRAF = null;
        // hold final pose for a moment
        setTimeout(() => redrawMain(), 100);
      }
    };
    animRAF = requestAnimationFrame(tick);
    log("animation started");
  }

  function stopAnimation() {
    if (animRAF) cancelAnimationFrame(animRAF);
    animRAF = null;
    state.animation.active = false;
    state.animation.t = 0;
    redrawMain();
  }

  btnAnim.addEventListener("click", startAnimation);
  btnStop.addEventListener("click", stopAnimation);

  btnReset.addEventListener("click", () => {
    stopAnimation();
    state.polygons = [];
    state.draw.verts = []; state.draw.hover = null;
    state.start.theta = 0;
    state.goal.theta  = 0;
    startTheta.value = "0"; syncThetaLabel(startTheta, startThetaVal);
    goalTheta.value  = "0"; syncThetaLabel(goalTheta,  goalThetaVal);
    state.result = null;
    state.esdfCanvas = null;
    setStatus(null, null, null, null, null);
    redrawMain(); redrawStages();
    log("reset");
  });

  // ----------------------------------------------------------- misc ui
  for (const el of [showAstar, showSC, showRS2, showSM, showRS, showRSAnchor, showFinal, showFP, showESDF]) {
    el.addEventListener("change", () => { redrawMain(); redrawStages(); });
  }
  esdfAlpha.addEventListener("input", () => redrawMain());
  for (const el of [fpShape, fpW, fpH, fpR]) {
    el.addEventListener("change", () => {
      schedulePlan();
    });
  }

  // ----------------------------------------------------------- heading sliders
  // The sliders report degrees; Pose2D.theta is radians, so we convert here.
  // Moving either slider invalidates the last plan (a different start/goal
  // heading produces a different trajectory even when x/y are unchanged).
  const DEG = Math.PI / 180;
  function syncThetaLabel(slider, label) {
    const deg = Number(slider.value);
    label.textContent = `${deg}°`;
  }
  function syncThetaSlider(slider, label, pose) {
    const deg = Math.round((pose.theta || 0) / DEG);
    slider.value = String(deg);
    syncThetaLabel(slider, label);
  }
  function bindThetaSlider(slider, label, pose, who) {
    syncThetaLabel(slider, label);
    slider.addEventListener("input", () => {
      const deg = Number(slider.value);
      pose.theta = deg * DEG;
      syncThetaLabel(slider, label);
      redrawMain(); redrawStages();
      schedulePlan();
    });
  }
  bindThetaSlider(startTheta, startThetaVal, state.start, "start");
  bindThetaSlider(goalTheta,  goalThetaVal,  state.goal,  "goal");

  // ----------------------------------------------------------- params panel
  // Renders a collapsible section per category.  All categories are
  // collapsed by default; clicking a section header toggles it.  Only
  // fields whose value differs from the default are sent to /api/plan
  // so the payload stays small and the server's defaults remain the
  // source of truth.
  function renderParamsPanel(schema) {
    state.configSchema = schema;
    state.config = {};
    paramsBody.innerHTML = "";
    if (!schema || !schema.categories || schema.categories.length === 0) {
      paramsBody.innerHTML = '<div class="params-loading">no parameters</div>';
      return;
    }
    for (const cat of schema.categories) {
      const sec = document.createElement("section");
      sec.className = "params-cat";
      sec.dataset.open = "false";   // default: collapsed
      const head = document.createElement("button");
      head.className = "params-cat-head";
      head.type = "button";
      head.innerHTML =
        '<span class="chev">▶</span>' +
        `<span class="cat-name"></span>` +
        `<span class="cat-count">${cat.fields.length}</span>`;
      head.querySelector(".cat-name").textContent = cat.name;
      head.addEventListener("click", () => {
        sec.dataset.open = sec.dataset.open === "true" ? "false" : "true";
      });
      sec.appendChild(head);
      const body = document.createElement("div");
      body.className = "params-cat-body";
      for (const f of cat.fields) {
        body.appendChild(buildParamField(f));
      }
      sec.appendChild(body);
      paramsBody.appendChild(sec);
    }
  }

  function buildParamField(f) {
    const row = document.createElement("div");
    row.className = "param-field" + (f.type === "bool" ? " pbool" : "");
    row.dataset.key = f.key;
    row.dataset.default = String(f.default);
    const name = document.createElement("span");
    name.className = "pname";
    name.textContent = f.label;
    if (f.help) name.title = `${f.label} — ${f.help}`;
    row.appendChild(name);
    if (f.type === "bool") {
      const wrap = document.createElement("label");
      wrap.className = "pval-bool";
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = !!f.default;
      cb.addEventListener("change", () => {
        recordParam(f, cb.checked, row);
      });
      const txt = document.createElement("span");
      txt.textContent = cb.checked ? "on" : "off";
      cb.addEventListener("change", () => { txt.textContent = cb.checked ? "on" : "off"; });
      wrap.appendChild(cb);
      wrap.appendChild(txt);
      row.appendChild(wrap);
    } else {
      const wrap = document.createElement("span");
      wrap.style.display = "inline-flex";
      wrap.style.alignItems = "center";
      const inp = document.createElement("input");
      inp.type = "number";
      if (f.min != null) inp.min = f.min;
      if (f.max != null) inp.max = f.max;
      if (f.step != null) inp.step = f.step;
      inp.value = f.default;
      inp.addEventListener("change", () => {
        const v = Number(inp.value);
        if (Number.isFinite(v)) recordParam(f, f.type === "int" ? Math.round(v) : v, row);
      });
      wrap.appendChild(inp);
      const reset = document.createElement("button");
      reset.className = "preset-btn";
      reset.type = "button";
      reset.textContent = "↺";
      reset.title = "reset to default";
      reset.addEventListener("click", () => {
        inp.value = f.default;
        delete state.config[f.key];
        row.classList.remove("dirty");
        schedulePlan();
      });
      wrap.appendChild(reset);
      row.appendChild(wrap);
    }
    return row;
  }

  function recordParam(f, value, row) {
    const def = f.default;
    const isDefault = (f.type === "bool") ? (value === !!def) : (Number(value) === Number(def));
    if (isDefault) {
      delete state.config[f.key];
      row.classList.remove("dirty");
    } else {
      state.config[f.key] = value;
      row.classList.add("dirty");
    }
    schedulePlan();
  }

  async function loadConfigSchema() {
    try {
      const r = await fetch("/api/config");
      const j = await r.json();
      renderParamsPanel(j);
    } catch (e) {
      paramsBody.innerHTML = `<div class="params-loading">failed: ${e}</div>`;
      log(`failed to load config schema: ${e}`, "err");
    }
  }

  btnResetCfg.addEventListener("click", () => {
    if (!state.configSchema) return;
    state.config = {};
    for (const row of paramsBody.querySelectorAll(".param-field")) {
      const key = row.dataset.key;
      const def = row.dataset.default;
      const cb = row.querySelector('input[type="checkbox"]');
      const inp = row.querySelector('input[type="number"]');
      const txt = row.querySelector(".pval-bool span");
      if (cb) {
        cb.checked = def === "true";
        if (txt) txt.textContent = cb.checked ? "on" : "off";
      } else if (inp) {
        inp.value = def;
      }
      row.classList.remove("dirty");
    }
    log("parameters reset to defaults");
    schedulePlan();
  });

  // ----------------------------------------------------------- boot
  checkHealth();
  loadPresets();
  loadConfigSchema();
  redrawMain();
  redrawStages();
  log("ready");
})();
