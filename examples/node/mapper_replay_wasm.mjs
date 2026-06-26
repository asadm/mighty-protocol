#!/usr/bin/env node
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import createMightyLoopClosureModule from "../../lib/loopclosure/wasm/lib/mighty_loopclosure_device.js";
import {
  NativeMapperWasm,
  createLoopClosureWasmModule,
} from "../../js/index.js";

function parseArgs(argv) {
  const out = {
    replayDir: "",
    stopAtProcessed: 180,
    outPath: "tmp/wasm-replay-map.csv",
  };
  for (let i = 2; i < argv.length; i += 1) {
    const arg = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) throw new Error(`${arg} requires a value`);
      return argv[++i];
    };
    if (arg === "--replay") out.replayDir = next();
    else if (arg === "--dump-after-processed") out.stopAtProcessed = Math.max(1, Number(next()));
    else if (arg === "--dump-map") out.outPath = next();
    else if (arg === "--help" || arg === "-h") {
      console.log("Usage: node mapper_replay_wasm.mjs --replay DIR [--dump-after-processed N] [--dump-map CSV]");
      process.exit(0);
    } else {
      throw new Error(`unknown argument: ${arg}`);
    }
  }
  if (!out.replayDir) throw new Error("--replay DIR is required");
  return out;
}

function readRows(eventsPath) {
  const lines = fs.readFileSync(eventsPath, "utf8").trimEnd().split(/\r?\n/);
  return lines.slice(1).filter(Boolean).map((line) => line.split(","));
}

function applyUpdate(frameMap, update) {
  const stats = { frameRecords: 0, pointRecords: 0 };
  if (update?.reset) frameMap.clear();
  for (const frame of update?.frames || []) {
    stats.frameRecords += 1;
    const frameId = Number(frame.frameId);
    if (!Number.isFinite(frameId)) continue;
    if (frame.remove) {
      frameMap.delete(frameId);
      continue;
    }
    const positions = frame.pointPositions;
    if (positions instanceof Float32Array) {
      frameMap.set(frameId, new Float32Array(positions));
      stats.pointRecords += Math.floor(positions.length / 3);
    }
  }
  return stats;
}

function pointCount(frameMap) {
  let total = 0;
  for (const positions of frameMap.values()) total += Math.floor(positions.length / 3);
  return total;
}

function writeCsv(frameMap, outPath) {
  fs.mkdirSync(path.dirname(path.resolve(outPath)), { recursive: true });
  const lines = ["frame_id,point_index,x,y,z"];
  const frameIds = [...frameMap.keys()].sort((a, b) => a - b);
  for (const frameId of frameIds) {
    const positions = frameMap.get(frameId) || new Float32Array(0);
    for (let i = 0; i + 2 < positions.length; i += 3) {
      lines.push(`${frameId},${i / 3},${positions[i]},${positions[i + 1]},${positions[i + 2]}`);
    }
  }
  fs.writeFileSync(outPath, `${lines.join("\n")}\n`);
}

async function main() {
  const args = parseArgs(process.argv);
  const replayDir = path.resolve(args.replayDir);
  const here = path.dirname(fileURLToPath(import.meta.url));
  const wasmPath = path.resolve(here, "../../lib/loopclosure/wasm/lib/mighty_loopclosure_device.wasm");
  const module = await createLoopClosureWasmModule({
    moduleFactory: createMightyLoopClosureModule,
    wasmBinary: fs.readFileSync(wasmPath),
  });
  const mapper = new NativeMapperWasm(module, { quiet: true });
  mapper.setCalibrationYaml(fs.readFileSync(path.join(replayDir, "calib.yaml"), "utf8"));

  const frameMap = new Map();
  let updates = 0;
  mapper.onMapUpdate((update) => {
    applyUpdate(frameMap, update);
    updates += 1;
  });

  let lastResult = null;
  let events = 0;
  for (const row of readRows(path.join(replayDir, "events.csv"))) {
    const type = row[0];
    if (type === "image") {
      const data = new Uint8Array(fs.readFileSync(path.join(replayDir, row[7])));
      lastResult = mapper.pushImage({
        frameId: Number(row[2]),
        timestampNs: row[3],
        width: Number(row[4]),
        height: Number(row[5]),
        format: Number(row[6]),
        data,
      }) || lastResult;
    } else if (type === "pose") {
      lastResult = mapper.pushPose({
        timestampNs: row[3],
        positionM: [Number(row[8]), Number(row[9]), Number(row[10])],
        orientationXyzw: [Number(row[11]), Number(row[12]), Number(row[13]), Number(row[14])],
        poseType: Number(row[15]) === 1 ? "camera" : "body",
        confidence: Number(row[16] || 1),
      }) || lastResult;
    }
    events += 1;
    if (Number(lastResult?.framesProcessed || 0) >= args.stopAtProcessed) break;
  }

  applyUpdate(frameMap, mapper.pollMapUpdates({ typedPoints: true }));
  writeCsv(frameMap, args.outPath);
  mapper.close();

  console.log(
    `wasm replay events=${events} processed=${lastResult?.framesProcessed || 0}` +
      ` updates=${updates} map_points=${pointCount(frameMap)}`,
  );
}

main().catch((err) => {
  console.error(err?.stack || err);
  process.exit(1);
});
