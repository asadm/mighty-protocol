const NUMBER_RE = /[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?/g;

function canonicalCameraModel(raw) {
  const token = String(raw || "").trim().replace(/^['"]|['"]$/g, "")
    .toLowerCase().replace(/[-_\s]/g, "");
  if (token === "ds" || token === "doublesphere") return "double_sphere";
  if (token === "pinhole") return "pinhole";
  return "unknown";
}

function canonicalDistortionModel(raw) {
  if (raw == null || String(raw).trim() === "") return "unknown";
  const token = String(raw || "").trim().replace(/^['"]|['"]$/g, "")
    .toLowerCase().replace(/[-_\s]/g, "");
  if (token === "none" || token === "off" || token === "disabled") return "none";
  if (token === "radtan" || token === "radialtangential") return "radtan";
  if (token === "equidistant" || token === "fisheye") return "equidistant";
  return "unknown";
}

function escapeRegExp(value) {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function findKey(text, key) {
  const pattern = new RegExp(`^[ \\t]*${escapeRegExp(key)}[ \\t]*:`, "m");
  const match = pattern.exec(text);
  if (!match) return null;
  return match.index + match[0].length;
}

function scalarText(text, key) {
  const start = findKey(text, key);
  if (start == null) return null;
  const newline = text.indexOf("\n", start);
  let value = text.slice(start, newline < 0 ? text.length : newline).trim();
  const comment = value.indexOf("#");
  if (comment >= 0) value = value.slice(0, comment).trim();
  if (value.length >= 2 && ((value[0] === '"' && value.at(-1) === '"') ||
      (value[0] === "'" && value.at(-1) === "'"))) {
    value = value.slice(1, -1);
  }
  return value || null;
}

function scalarNumber(text, key) {
  const raw = scalarText(text, key);
  if (raw == null || raw === "") return null;
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

function matchingBracket(text, open) {
  let depth = 0;
  for (let index = open; index < text.length; index += 1) {
    if (text[index] === "[") depth += 1;
    else if (text[index] === "]") {
      depth -= 1;
      if (depth === 0) return index;
    }
  }
  return -1;
}

function numbersInRange(text, begin, end) {
  const values = [];
  const source = text.slice(begin, end);
  for (const match of source.matchAll(NUMBER_RE)) {
    const value = Number(match[0]);
    if (!Number.isFinite(value)) throw new Error("calibration contains a non-finite number");
    values.push(value);
  }
  return values;
}

function bracketNumbers(text, key, adjacentRowTarget = 0) {
  const valueStart = findKey(text, key);
  if (valueStart == null) return null;
  let open = text.indexOf("[", valueStart);
  if (open < 0 || open - valueStart > 512) return null;
  const prefix = text.slice(valueStart, open);
  const firstPrefixLine = prefix.split("\n", 1)[0];
  if (!firstPrefixLine.includes("!!opencv-matrix") && !/^[\s-]*$/.test(prefix)) return null;

  const values = [];
  while (open >= 0) {
    const close = matchingBracket(text, open);
    if (close < 0) return null;
    values.push(...numbersInRange(text, open + 1, close));
    if (!adjacentRowTarget || values.length >= adjacentRowTarget) return values;
    const nextOpen = text.indexOf("[", close + 1);
    if (nextOpen < 0 || nextOpen - close > 128 ||
        !/^[\s-]*$/.test(text.slice(close + 1, nextOpen))) {
      return null;
    }
    open = nextOpen;
  }
  return null;
}

function determinant3(matrix) {
  return matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
    matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
    matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
}

function isRigidMatrix(matrix) {
  if (!Array.isArray(matrix) || matrix.length !== 16 || matrix.some((v) => !Number.isFinite(v))) return false;
  if (Math.abs(matrix[12]) > 1e-6 || Math.abs(matrix[13]) > 1e-6 ||
      Math.abs(matrix[14]) > 1e-6 || Math.abs(matrix[15] - 1) > 1e-6 ||
      Math.abs(determinant3(matrix) - 1) > 0.1) return false;
  let error2 = 0;
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      let dot = 0;
      for (let index = 0; index < 3; index += 1) dot += matrix[row * 4 + index] * matrix[col * 4 + index];
      const error = dot - (row === col ? 1 : 0);
      error2 += error * error;
    }
  }
  return Math.sqrt(error2) <= 0.1;
}

function transform(matrix = null) {
  return {
    matrix: matrix ? matrix.map(Number) : [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ],
    valid: !!matrix,
  };
}

function inverseRigid(matrix) {
  if (!isRigidMatrix(matrix)) return transform();
  const out = transform().matrix;
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) out[row * 4 + col] = matrix[col * 4 + row];
    out[row * 4 + 3] = -(out[row * 4] * matrix[3] + out[row * 4 + 1] * matrix[7] + out[row * 4 + 2] * matrix[11]);
  }
  return transform(out);
}

function deriveBodyFromCamera(cameraFromImu) {
  if (!isRigidMatrix(cameraFromImu)) return transform();
  return transform([
    0, 0, 1, -cameraFromImu[11],
    -1, 0, 0, cameraFromImu[3],
    0, -1, 0, cameraFromImu[7],
    0, 0, 0, 1,
  ]);
}

function matrixForKey(text, key) {
  const values = bracketNumbers(text, key, 16);
  return values?.length === 16 && isRigidMatrix(values) ? values : null;
}

function positiveInt(value) {
  return Number.isFinite(value) && value > 0 ? Math.round(value) : 0;
}

function parseCamera(text, id) {
  const cameraModel = canonicalCameraModel(scalarText(text, "camera_model"));
  const distortionModel = canonicalDistortionModel(scalarText(text, "distortion_model"));
  const intrinsicsList = bracketNumbers(text, "intrinsics");
  const intrinsics = { fx: 0, fy: 0, cx: 0, cy: 0 };
  let projectionParameters = [];
  if (intrinsicsList?.length >= 4) {
    [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy] = intrinsicsList.slice(-4);
    projectionParameters = intrinsicsList.slice(0, -4);
  } else {
    intrinsics.fx = scalarNumber(text, "fx") ?? 0;
    intrinsics.fy = scalarNumber(text, "fy") ?? 0;
    intrinsics.cx = scalarNumber(text, "cx") ?? 0;
    intrinsics.cy = scalarNumber(text, "cy") ?? 0;
    const xi = scalarNumber(text, "xi");
    const alpha = scalarNumber(text, "alpha");
    if (cameraModel === "double_sphere" && xi != null && alpha != null) projectionParameters = [xi, alpha];
  }

  let distortionCoefficients = bracketNumbers(text, "distortion_coeffs") || [];
  if (!distortionCoefficients.length && distortionModel !== "none" && cameraModel !== "double_sphere") {
    distortionCoefficients = ["k1", "k2", "p1", "p2"]
      .map((name) => scalarNumber(text, name)).filter((value) => value != null);
  }

  const resolutionList = bracketNumbers(text, "resolution");
  const resolution = resolutionList?.length >= 2
    ? { width: positiveInt(resolutionList[0]), height: positiveInt(resolutionList[1]) }
    : {
      width: positiveInt(scalarNumber(text, "resolution_width")),
      height: positiveInt(scalarNumber(text, "resolution_height")),
    };

  let cameraFromImu = transform();
  let bodyFromCamera = transform();
  const tCameraImu = matrixForKey(text, "T_cam_imu");
  const tImuCamera = matrixForKey(text, "T_imu_cam");
  const tCameraBody = matrixForKey(text, "T_cam_body");
  if (tCameraImu) {
    cameraFromImu = transform(tCameraImu);
    bodyFromCamera = deriveBodyFromCamera(tCameraImu);
  } else if (tImuCamera) {
    cameraFromImu = inverseRigid(tImuCamera);
    if (cameraFromImu.valid) bodyFromCamera = deriveBodyFromCamera(cameraFromImu.matrix);
  } else if (tCameraBody) {
    bodyFromCamera = inverseRigid(tCameraBody);
  }

  const timeShiftCameraImuS = scalarNumber(text, "timeshift_cam_imu") ?? scalarNumber(text, "td") ?? 0;
  return {
    id,
    cameraModel,
    distortionModel,
    intrinsics,
    projectionParameters,
    distortionCoefficients,
    resolution,
    topic: scalarText(text, "rostopic") || "",
    timeShiftCameraImuS,
    cameraFromImu,
    bodyFromCamera,
    valid: Number.isFinite(intrinsics.fx) && intrinsics.fx > 0 &&
      Number.isFinite(intrinsics.fy) && intrinsics.fy > 0 &&
      Number.isFinite(intrinsics.cx) && Number.isFinite(intrinsics.cy),
  };
}

function cameraSections(yaml) {
  const matches = [...yaml.matchAll(/^(cam\d+):\s*$/gm)];
  return matches.map((match, index) => ({
    id: match[1],
    text: yaml.slice(match.index, index + 1 < matches.length ? matches[index + 1].index : yaml.length),
  }));
}

/** Normalize Mighty OpenCV or Kalibr calibration YAML into the SDK schema. */
export function parseCalibrationYaml(yaml) {
  if (typeof yaml !== "string" || !yaml.trim()) throw new Error("calibration YAML is empty");
  let sections = cameraSections(yaml);
  const sourceFormat = sections.length ? "kalibr" : "opencv";
  if (!sections.length) sections = [{ id: "cam0", text: yaml }];
  const cameras = sections.map(({ id, text }) => parseCamera(text, id)).filter((camera) => camera.valid);
  if (!cameras.length) throw new Error("calibration has no camera with valid intrinsics");
  return { sourceFormat, cameras };
}
