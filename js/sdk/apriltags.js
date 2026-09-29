/** WPILib tag poses use +X normal, +Y left, +Z up, in the file's field frame. */
export const FRC_TAG_SIZE_M = 0.1651;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(`${label} must be a finite number`);
  }
  return value;
}

export function normalizeTagQuaternion(values) {
  if (!Array.isArray(values) || values.length !== 4) {
    throw new Error("orientationXyzw must contain four numbers");
  }
  const q = values.map((v) => finiteNumber(v, "quaternion component"));
  const scale = Math.max(...q.map(Math.abs));
  if (scale === 0) throw new Error("quaternion must be nonzero");
  const scaled = q.map((v) => v / scale);
  const norm = Math.hypot(...scaled);
  return scaled.map((v) => v / norm);
}

export function wpilibTagQuaternionToMighty(values) {
  const [x, y, z, w] = normalizeTagQuaternion(values);
  // Right/down/normal PnP axes are -Y/-Z/+X in WPILib tag coordinates.
  return [(-w + x - y - z) / 2, (w + x + y - z) / 2,
    (-w + x + y + z) / 2, (w + x - y + z) / 2];
}

export function mightyTagQuaternionToWpilib(values) {
  const [x, y, z, w] = normalizeTagQuaternion(values);
  return [(w + x + y + z) / 2, (-w - x + y + z) / 2,
    (w - x - y + z) / 2, (w - x + y - z) / 2];
}

function normalizeTag(tag, seen) {
  if (!tag || typeof tag !== "object" || !Number.isInteger(tag.tagId) || tag.tagId < 0 || tag.tagId > 2147483647) {
    throw new Error("tagId must be a non-negative 32-bit integer");
  }
  if (seen.has(tag.tagId)) throw new Error(`duplicate tag ID ${tag.tagId}`);
  seen.add(tag.tagId);
  const sizeM = finiteNumber(tag.sizeM, "sizeM");
  if (sizeM <= 0) throw new Error("sizeM must be positive");
  if (!Array.isArray(tag.positionM) || tag.positionM.length !== 3) {
    throw new Error("positionM must contain three numbers");
  }
  return { tagId: tag.tagId, sizeM,
    positionM: tag.positionM.map((v) => finiteNumber(v, "position")),
    orientationXyzw: normalizeTagQuaternion(tag.orientationXyzw) };
}

export function parseWpilibFieldLayout(source, { tagSizeM } = {}) {
  const layout = typeof source === "string" ? JSON.parse(source) : source;
  if (!layout || !Array.isArray(layout.tags) || !layout.tags.length) {
    throw new Error("WPILib layout must contain a nonempty tags array");
  }
  if (layout.mighty !== undefined) {
    if (!layout.mighty || typeof layout.mighty !== "object" || Array.isArray(layout.mighty)) {
      throw new Error("mighty metadata must be an object");
    }
    if (layout.mighty.tagFamily !== undefined && layout.mighty.tagFamily !== "tag36h11") {
      throw new Error("Mighty requires tag36h11 tags");
    }
  }
  // WPILib ignores this optional extension. Explicit size overrides take priority.
  const sizeM = tagSizeM === undefined ? (layout.mighty?.tagSizeM ?? FRC_TAG_SIZE_M) : tagSizeM;
  if (layout.mighty?.tagSizeM === null && tagSizeM === undefined) throw new Error("tagSizeM must be a finite number");
  const seen = new Set();
  const tags = layout.tags.map((row) => {
    const p = row?.pose?.translation;
    const q = row?.pose?.rotation?.quaternion;
    if (!p || !q) throw new Error("each tag must contain ID, translation and quaternion");
    return normalizeTag({ tagId: row.ID, sizeM,
      positionM: [p.x, p.y, p.z], orientationXyzw: [q.X, q.Y, q.Z, q.W] }, seen);
  });
  const field = {};
  if (layout.field !== undefined && (!layout.field || typeof layout.field !== "object" || Array.isArray(layout.field))) {
    throw new Error("field must be an object");
  }
  for (const key of ["length", "width"]) {
    if (layout.field?.[key] !== undefined) {
      field[key] = finiteNumber(layout.field[key], `field.${key}`);
      if (field[key] <= 0) throw new Error("field dimensions must be positive");
    }
  }
  return { tags, field };
}

export function serializeApriltagMapYaml(tagMap) {
  if (!tagMap || !Array.isArray(tagMap.tags) || !tagMap.tags.length) {
    throw new Error("tag map must contain a nonempty tags array");
  }
  const lines = ["%YAML:1.0", "---", "apriltags:"];
  const seen = new Set();
  for (const row of tagMap.tags) {
    const tag = normalizeTag(row, seen);
    lines.push(`  - tag_id: ${tag.tagId}`, `    size_m: ${tag.sizeM}`,
      `    position_m: [${tag.positionM.join(", ")}]`,
      "    orientation: quaternion",
      `    orientation_xyzw: [${wpilibTagQuaternionToMighty(tag.orientationXyzw).join(", ")}]`);
  }
  return `${lines.join("\n")}\n`;
}
