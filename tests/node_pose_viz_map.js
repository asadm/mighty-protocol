import fs from "node:fs";
import path from "node:path";

const here = path.dirname(new URL(import.meta.url).pathname);
const fixturePath = path.join(here, "pose_viz_cases.json");
const fixture = JSON.parse(fs.readFileSync(fixturePath, "utf8"));

const R = [
  [0.0, -1.0, 0.0],
  [0.0, 0.0, 1.0],
  [-1.0, 0.0, 0.0],
];
const Q_VIZ_FROM_ODOM = [0.5, -0.5, 0.5, 0.5]; // xyzw

function matVec3(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

function qMulXYWZ(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

function qNorm(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (!Number.isFinite(n) || n <= 1e-12) return [0.0, 0.0, 0.0, 1.0];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

const out = {
  language: "js",
  cases: fixture.cases.map((c) => {
    const qIn = qNorm(c.quat);
    const qViz = qNorm(qMulXYWZ(Q_VIZ_FROM_ODOM, qIn));
    return {
      name: c.name,
      position: matVec3(R, c.position),
      quat: qViz,
    };
  }),
};

process.stdout.write(`${JSON.stringify(out)}\n`);
