#!/usr/bin/env node
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import {
  MightyClient,
  MightyWebDevice,
} from "../../js/index.js";

function parseArgs(argv) {
  const out = {
    host: "",
    seconds: 120,
    out: "loopclosure_node.svg",
    calibration: "",
    noStart: false,
  };
  for (let i = 2; i < argv.length; i += 1) {
    const arg = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`${arg} requires a value`);
      return argv[++i];
    };
    if (arg === "--host") out.host = next();
    else if (arg === "--seconds") out.seconds = Number(next());
    else if (arg === "--out") out.out = next();
    else if (arg === "--calibration") out.calibration = fs.readFileSync(next(), "utf8");
    else if (arg === "--no-start") out.noStart = true;
    else if (arg === "--help" || arg === "-h") {
      console.log("Usage: node loopclosure_wasm.mjs [--host URL] [--seconds N] [--out FILE] [--calibration FILE] [--no-start]");
      process.exit(0);
    } else {
      throw new Error(`unknown argument: ${arg}`);
    }
  }
  return out;
}

function pointBounds(raw, opt) {
  const all = raw.concat(opt);
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of all.length ? all : [[0, 0], [1, 1]]) {
    minX = Math.min(minX, p[0]);
    minY = Math.min(minY, p[1]);
    maxX = Math.max(maxX, p[0]);
    maxY = Math.max(maxY, p[1]);
  }
  if (Math.abs(maxX - minX) < 1e-9) { minX -= 1; maxX += 1; }
  if (Math.abs(maxY - minY) < 1e-9) { minY -= 1; maxY += 1; }
  return { minX, minY, maxX, maxY };
}

function writeSvg(events, keyframes, outPath) {
  const points = keyframes.map((p) => [p.positionM[0], p.positionM[1]]);
  const loops = events.map((e) => [e.currentKeyframe, e.matchedKeyframe]);
  const { minX, minY, maxX, maxY } = pointBounds(points, []);
  const width = 920, height = 760, margin = 48, headerH = 128;
  const plotW = width - 2 * margin;
  const plotH = height - headerH - margin;
  const scale = Math.min(plotW / (maxX - minX), plotH / (maxY - minY));
  const xOffset = (plotW - (maxX - minX) * scale) * 0.5;
  const yOffset = (plotH - (maxY - minY) * scale) * 0.5;
  const x0 = margin;
  const sx = (p, x0) => x0 + xOffset + (p[0] - minX) * scale;
  const sy = (p) => headerH + plotH - yOffset - (p[1] - minY) * scale;
  const poly = (points, x0) => points.map((p) => `${sx(p, x0).toFixed(2)},${sy(p).toFixed(2)}`).join(" ");

  let svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" width="${width}" height="${height}">`;
  svg += `<rect width="${width}" height="${height}" fill="#f8f8f4"/>`;
  svg += `<text x="48" y="40" font-family="system-ui" font-size="18" font-weight="700" fill="#0f172a">Mighty Node WASM Loop Closure SDK</text>`;
  svg += `<text x="48" y="66" font-family="system-ui" font-size="13" fill="#0f172a">keyframes: ${keyframes.length} | loops: ${loops.length}</text>`;
  svg += `<rect x="${x0}" y="${headerH}" width="${plotW}" height="${plotH}" fill="#fff" stroke="#d4d4ce"/>`;
  for (let i = 1; i < 10; i += 1) {
    const gx = x0 + plotW * i / 10;
    const gy = headerH + plotH * i / 10;
    svg += `<line x1="${gx}" y1="${headerH}" x2="${gx}" y2="${headerH + plotH}" stroke="#e4e4df"/>`;
    svg += `<line x1="${x0}" y1="${gy}" x2="${x0 + plotW}" y2="${gy}" stroke="#e4e4df"/>`;
  }
  svg += `<text x="${x0}" y="${headerH - 14}" font-family="system-ui" font-size="14" font-weight="700" fill="#0f172a">Corrected keyframes</text>`;
  for (const [current, matched] of loops) {
    if (current < points.length && matched < points.length) {
      const a = points[current], b = points[matched];
      svg += `<line x1="${sx(a, x0)}" y1="${sy(a)}" x2="${sx(b, x0)}" y2="${sy(b)}" stroke="#0099ff" stroke-width="2.5" stroke-dasharray="8 5" opacity="0.85"/>`;
    }
  }
  if (points.length >= 2) svg += `<polyline points="${poly(points, x0)}" fill="none" stroke="#ff0055" stroke-width="4" opacity="0.95"/>`;
  for (const p of points) svg += `<circle cx="${sx(p, x0)}" cy="${sy(p)}" r="3.5" fill="#ffcc00" stroke="#ff0055" stroke-width="1"/>`;
  svg += "</svg>\n";
  fs.mkdirSync(path.dirname(path.resolve(outPath)), { recursive: true });
  fs.writeFileSync(outPath, svg);
}

async function main() {
  const args = parseArgs(process.argv);
  const here = path.dirname(fileURLToPath(import.meta.url));
  const wasmDir = path.resolve(here, "../../lib/loopclosure/wasm/lib");
  const wasmUrl = pathToFileURL(path.join(wasmDir, "mighty_loopclosure_device.wasm")).href;

  const device = args.host ? new MightyWebDevice({ baseUrl: args.host }) : new MightyWebDevice();
  const client = new MightyClient(device, {
    autoReconnect: true,
    loopclosure: true,
    loopclosureWasmUrl: wasmUrl,
  });
  const events = [];
  const keyframes = [];
  let frames = 0;
  let streamClosedAt = 0;
  client.onImage((image) => {
    frames += 1;
  });
  client.onPose((pose) => {
    if (pose.isKeyframe && Array.isArray(pose.positionM)) keyframes.push(pose);
  });
  client.onLoopclosure((event) => {
    events.push(event);
    console.log("[loop]", event.type, "current=", event.currentKeyframe, "matched=", event.matchedKeyframe);
    if (keyframes.length) writeSvg(events, keyframes, args.out);
  });
  client.onError((err) => {
    if (err.scope === "transport" && err.code === "stream_error" && frames > 0) {
      if (!streamClosedAt) streamClosedAt = Date.now();
      return;
    }
    console.error("[error]", `${err.scope}:${err.code}`, err.message);
  });

  await client.connect();
  console.log("transport=http source=", device.getInfo().source);
  if (args.calibration) {
    console.log("loop-closure calibration loaded=", client.setLoopclosureCalibrationYaml(args.calibration));
  } else {
    for (let i = 0; i < 40; i += 1) {
      const cfg = await client.configGet("calib", { as: "text" });
      if (cfg.ok && cfg.found && cfg.value) {
        console.log("loop-closure calibration loaded=", client.setLoopclosureCalibrationYaml(cfg.value));
        break;
      }
      await new Promise((resolve) => setTimeout(resolve, 250));
    }
  }
  if (!args.noStart) {
    const res = await client.startVio();
    console.log("start_vio ok=", res.ok, "message=", res.message);
  }

  const stopAt = Date.now() + Math.max(1, args.seconds) * 1000;
  while (Date.now() < stopAt) {
    if (streamClosedAt && Date.now() - streamClosedAt > 1000) break;
    await new Promise((resolve) => setTimeout(resolve, 100));
  }

  if (!streamClosedAt) {
    if (!args.noStart) await client.stopVio().catch(() => {});
  }
  await client.disconnect();
  client.closeLoopclosure();
  if (keyframes.length) writeSvg(events, keyframes, args.out);
  console.log("wrote", args.out);
  console.log("stats", client.stats());
}

main().catch((err) => {
  console.error(err?.stack || err);
  process.exit(1);
});
