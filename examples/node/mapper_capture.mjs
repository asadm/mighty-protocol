#!/usr/bin/env node
import fs from "node:fs";
import path from "node:path";

import {
  MightyClient,
  MightyWebDevice,
} from "../../js/index.js";

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function parseArgs(argv) {
  const out = {
    host: "http://127.0.0.1:8084",
    outDir: "tmp/mapper-replay",
    images: 240,
  };
  for (let i = 2; i < argv.length; i += 1) {
    const arg = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`${arg} requires a value`);
      return argv[++i];
    };
    if (arg === "--host") out.host = next();
    else if (arg === "--out") out.outDir = next();
    else if (arg === "--images") out.images = Math.max(1, Number(next()));
    else if (arg === "--help" || arg === "-h") {
      console.log("Usage: node mapper_capture.mjs [--host URL] [--out DIR] [--images N]");
      process.exit(0);
    } else {
      throw new Error(`unknown argument: ${arg}`);
    }
  }
  return out;
}

function pickPrimaryRaw(imageEvt) {
  if (!imageEvt) return null;
  if (imageEvt.kind === "raw") return imageEvt;
  if (imageEvt.kind !== "stereo_raw") return null;
  return imageEvt.left || imageEvt.right || null;
}

function isMapperPose(pose) {
  return !!pose?.isPublic &&
    Array.isArray(pose.orientationXyzw) &&
    (pose.poseType === "body" || pose.poseType === "camera" ||
     pose.poseTypeRaw === 0 || pose.poseTypeRaw === 1);
}

function eventLine(fields) {
  return `${fields.join(",")}\n`;
}

async function main() {
  const args = parseArgs(process.argv);
  const outDir = path.resolve(args.outDir);
  const framesDir = path.join(outDir, "frames");
  fs.mkdirSync(framesDir, { recursive: true });

  const eventsPath = path.join(outDir, "events.csv");
  const events = fs.createWriteStream(eventsPath, { flags: "w" });
  events.write("event,seq,frame_id,timestamp_ns,width,height,format,raw_path,px,py,pz,qx,qy,qz,qw,pose_frame,confidence\n");

  const device = new MightyWebDevice({ baseUrl: args.host });
  const client = new MightyClient(device, { autoReconnect: false, streamStallTimeoutMs: 8000 });
  let running = true;
  let streamClosedAt = 0;
  let images = 0;
  let poses = 0;

  client.onImage((image) => {
    if (!running) return;
    const raw = pickPrimaryRaw(image);
    if (!raw || !raw.timestampNs) return;
    const data = raw.data instanceof Uint8Array ? raw.data : new Uint8Array(raw.data || []);
    const frameId = images;
    const rawName = `frames/frame_${String(frameId).padStart(6, "0")}.raw`;
    fs.writeFileSync(path.join(outDir, rawName), data);
    events.write(eventLine([
      "image",
      images,
      frameId,
      String(raw.timestampNs),
      raw.width || 0,
      raw.height || 0,
      raw.format ?? 0,
      rawName,
      "", "", "", "", "", "", "", "", "",
    ]));
    images += 1;
    if (images >= args.images) running = false;
  });

  client.onPose((pose) => {
    if (!running || !isMapperPose(pose)) return;
    const q = pose.orientationXyzw || [0, 0, 0, 1];
    const p = pose.positionM || [0, 0, 0];
    const frame = (pose.poseType || pose.pose_type) === "camera" ||
        pose.poseTypeRaw === 1 || pose.pose_type_raw === 1
      ? 1
      : 0;
    events.write(eventLine([
      "pose",
      poses,
      "",
      String(pose.timestampNs ?? pose.timestamp_ns ?? 0),
      "", "", "", "",
      Number(p[0] || 0),
      Number(p[1] || 0),
      Number(p[2] || 0),
      Number(q[0] || 0),
      Number(q[1] || 0),
      Number(q[2] || 0),
      Number(q[3] ?? 1),
      frame,
      Number(pose.confidence ?? 1),
    ]));
    poses += 1;
  });

  client.onError((err) => {
    if (err.scope === "transport") streamClosedAt = Date.now();
  });

  await client.connect();
  let calibration = "";
  for (let i = 0; i < 40; i += 1) {
    const cfg = await client.configGet("calib", { as: "text" });
    if (cfg.ok && cfg.found && cfg.value) {
      calibration = cfg.value;
      fs.writeFileSync(path.join(outDir, "calib.yaml"), calibration);
      break;
    }
    await sleep(250);
  }
  if (!calibration) throw new Error("calibration not available");

  while (running && !streamClosedAt) {
    await sleep(20);
  }
  await client.disconnect().catch(() => {});
  events.end();
  await new Promise((resolve) => events.on("finish", resolve));

  console.log(`captured dir=${outDir} images=${images} poses=${poses} events=${eventsPath}`);
}

main().catch((err) => {
  console.error(err?.stack || err);
  process.exit(1);
});
