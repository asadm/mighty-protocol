import { RAW_FORMAT } from "../core/protocol.js";
import { parseCalibrationYaml } from "./calibration.js";
import { toU8 } from "./utils.js";

const OCCUPIED = 2;
const MATCH_TOLERANCE_NS = 15_000_000n;
const POSE_HISTORY_LIMIT = 240;
const PRIMARY_CHANNELS = new Set(["", "cam0", "preview", "left"]);

const LAYOUT_FIELDS = [
  "optionsSize",
  "optionsResolutionMOffset",
  "optionsHasRollingRadiusOffset",
  "optionsRollingRadiusMOffset",
  "calibrationSize",
  "calibrationWidthOffset",
  "calibrationHeightOffset",
  "calibrationFxOffset",
  "calibrationFyOffset",
  "calibrationCxOffset",
  "calibrationCyOffset",
  "calibrationCameraModelOffset",
  "calibrationDistortionModelOffset",
  "calibrationProjectionParametersOffset",
  "calibrationProjectionParameterCountOffset",
  "calibrationDistortionCoefficientsOffset",
  "calibrationDistortionCoefficientCountOffset",
  "calibrationBodyFromCameraOffset",
  "imageSize",
  "imageDataOffset",
  "imageSizeBytesOffset",
  "imageWidthOffset",
  "imageHeightOffset",
  "imageStrideBytesOffset",
  "imageFormatOffset",
  "poseSize",
  "poseTimestampNsOffset",
  "posePxOffset",
  "posePyOffset",
  "posePzOffset",
  "poseQwOffset",
  "poseQxOffset",
  "poseQyOffset",
  "poseQzOffset",
  "poseFrameOffset",
  "poseConfidenceOffset",
  "poseIsKeyframeHintOffset",
  "frameSize",
  "frameTimestampNsOffset",
  "frameImageOffset",
  "framePoseOffset",
  "frameTranslationMetricsAvailableOffset",
  "frameTranslationConfidenceOffset",
  "frameTranslationObservabilityOffset",
  "frameDegradedReasonFlagsOffset",
  "updateSize",
  "updateBaseRevisionOffset",
  "updateRevisionOffset",
  "updateTimestampNsOffset",
  "updateResetOffset",
  "updateResolutionMOffset",
  "updateDisplayCellSizeMOffset",
  "updateHasRollingCenterOffset",
  "updateRollingCenterOffset",
  "updateHasRollingRadiusOffset",
  "updateRollingRadiusMOffset",
  "updateChangesOffset",
  "updateChangeCountOffset",
  "cellSize",
  "cellIndexXOffset",
  "cellIndexYOffset",
  "cellIndexZOffset",
  "cellStateOffset",
  "cellOccupancyOffset",
  "cellIntensityOffset",
  "cellSupportOffset",
  "cellVisibleOffset",
  "statsSize",
  "statsInputFramesOffset",
  "statsProcessedFramesOffset",
  "statsIntegratedFramesOffset",
  "statsRejectedFramesOffset",
  "statsIntegratedPointsOffset",
  "statsCellCountOffset",
  "statsOccupiedCellCountOffset",
  "statsMemoryBytesOffset",
];

function timestampNs(value) {
  try {
    return typeof value === "bigint" ? value : BigInt(value || 0);
  } catch (_) {
    return 0n;
  }
}

function writeU64(module, ptr, value) {
  module.setValue(ptr, timestampNs(value), "i64");
}

function readU64(module, ptr) {
  return timestampNs(module.getValue(ptr, "i64"));
}

function clearMemory(module, ptr, size) {
  module.HEAPU8.fill(0, ptr, ptr + size);
}

function readLayout(module) {
  const ptr = module._malloc(LAYOUT_FIELDS.length * 4);
  try {
    module._ma_occupancy_abi_layout(ptr);
    const layout = {};
    LAYOUT_FIELDS.forEach((name, index) => {
      layout[name] = module.getValue(ptr + index * 4, "i32") >>> 0;
    });
    return layout;
  } finally {
    module._free(ptr);
  }
}

function finiteArray(value, length, fallback) {
  if (!value || value.length < length) return fallback.slice();
  const output = Array.from(value).slice(0, length).map(Number);
  return output.every(Number.isFinite) ? output : fallback.slice();
}

function primaryChannel(image) {
  const channel = String(image?.channelAlias || image?.channel || "").trim().toLowerCase();
  return PRIMARY_CHANNELS.has(channel);
}

function selectRawImage(image) {
  if (!image) return null;
  if (image.kind === "raw" || image.kind === "jpg") return image;
  if (image.kind !== "stereo_raw") return null;
  if (primaryChannel(image.left)) return image.left;
  if (primaryChannel(image.right)) return image.right;
  return image.left || image.right || null;
}

async function decodeJpeg(image) {
  if (typeof createImageBitmap !== "function" || typeof Blob === "undefined") {
    throw new Error("JPEG occupancy input requires browser image APIs");
  }
  const bitmap = await createImageBitmap(new Blob([toU8(image.data)], { type: "image/jpeg" }));
  try {
    const width = bitmap.width || 0;
    const height = bitmap.height || 0;
    if (width <= 0 || height <= 0) throw new Error("decoded JPEG has invalid dimensions");
    const canvas = typeof OffscreenCanvas === "function"
      ? new OffscreenCanvas(width, height)
      : document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext("2d", { willReadFrequently: true });
    if (!context) throw new Error("unable to create JPEG decode canvas");
    context.drawImage(bitmap, 0, 0);
    return {
      kind: "raw",
      timestampNs: image.timestampNs,
      width,
      height,
      format: RAW_FORMAT.RGBA32,
      channel: image.channel,
      channelAlias: image.channelAlias,
      data: new Uint8Array(context.getImageData(0, 0, width, height).data),
    };
  } finally {
    bitmap.close?.();
  }
}

function selectCamera(calibration, cameraId = "") {
  const parsed = typeof calibration === "string" ? parseCalibrationYaml(calibration) : calibration;
  const cameras = parsed?.cameras || [];
  const requested = String(cameraId || "").trim().toLowerCase();
  const camera = (requested && cameras.find((item) => String(item.id || "").toLowerCase() === requested))
    || cameras.find((item) => String(item.id || "").toLowerCase() === "cam0")
    || cameras[0];
  if (!camera?.valid) throw new Error("occupancy grid requires valid camera calibration");
  return camera;
}

function nativePixelFormat(format) {
  switch (format) {
    case RAW_FORMAT.RGB24: return 1;
    case RAW_FORMAT.BGR24: return 2;
    case RAW_FORMAT.RGBA32: return 3;
    case RAW_FORMAT.BGRA32: return 4;
    case RAW_FORMAT.YUV420SP: return 5;
    case RAW_FORMAT.YUV420P: return 6;
    case RAW_FORMAT.GRAY8:
    case RAW_FORMAT.UNKNOWN:
    default: return 0;
  }
}

function imageStride(image) {
  if (image.format === RAW_FORMAT.RGB24 || image.format === RAW_FORMAT.BGR24) return image.width * 3;
  if (image.format === RAW_FORMAT.RGBA32 || image.format === RAW_FORMAT.BGRA32) return image.width * 4;
  return image.width;
}

class NativeOccupancyGrid {
  constructor(module, camera, options = {}) {
    if (!module?._ma_occupancy_create) {
      throw new Error("Mighty Algorithms WASM does not include occupancy-grid support");
    }
    this.module = module;
    this.layout = readLayout(module);
    this.handle = 0;
    this._create(camera, options);
  }

  _create(camera, options) {
    const m = this.module;
    const l = this.layout;
    const optionsPtr = m._malloc(l.optionsSize);
    const calibrationPtr = m._malloc(l.calibrationSize);
    const outPtr = m._malloc(4);
    const allocations = [];
    try {
      clearMemory(m, optionsPtr, l.optionsSize);
      m._ma_occupancy_options_default(optionsPtr);
      if (options.resolutionM !== undefined) {
        m.setValue(optionsPtr + l.optionsResolutionMOffset, Number(options.resolutionM), "float");
      }
      if (options.rollingRadiusM === null || options.rollingRadiusM === false) {
        m.setValue(optionsPtr + l.optionsHasRollingRadiusOffset, 0, "i32");
      } else if (options.rollingRadiusM !== undefined) {
        m.setValue(optionsPtr + l.optionsHasRollingRadiusOffset, 1, "i32");
        m.setValue(optionsPtr + l.optionsRollingRadiusMOffset, Number(options.rollingRadiusM), "float");
      }

      clearMemory(m, calibrationPtr, l.calibrationSize);
      const resolution = camera.resolution || {};
      const intrinsics = camera.intrinsics || {};
      m.setValue(calibrationPtr + l.calibrationWidthOffset, Number(resolution.width || 0), "i32");
      m.setValue(calibrationPtr + l.calibrationHeightOffset, Number(resolution.height || 0), "i32");
      m.setValue(calibrationPtr + l.calibrationFxOffset, Number(intrinsics.fx || 0), "double");
      m.setValue(calibrationPtr + l.calibrationFyOffset, Number(intrinsics.fy || 0), "double");
      m.setValue(calibrationPtr + l.calibrationCxOffset, Number(intrinsics.cx || 0), "double");
      m.setValue(calibrationPtr + l.calibrationCyOffset, Number(intrinsics.cy || 0), "double");
      m.setValue(calibrationPtr + l.calibrationCameraModelOffset, camera.cameraModel === "double_sphere" ? 1 : 0, "i32");
      const distortionModel = camera.distortionModel === "radtan" ? 1
        : (camera.distortionModel === "equidistant" ? 2 : 0);
      m.setValue(calibrationPtr + l.calibrationDistortionModelOffset, distortionModel, "i32");

      const writeDoubleArray = (values, pointerOffset, countOffset) => {
        const array = Float64Array.from(values || []);
        let ptr = 0;
        if (array.length) {
          ptr = m._malloc(array.byteLength);
          allocations.push(ptr);
          m.HEAPU8.set(new Uint8Array(array.buffer), ptr);
        }
        m.setValue(calibrationPtr + pointerOffset, ptr, "*");
        m.setValue(calibrationPtr + countOffset, array.length, "i32");
      };
      writeDoubleArray(
        camera.projectionParameters,
        l.calibrationProjectionParametersOffset,
        l.calibrationProjectionParameterCountOffset
      );
      writeDoubleArray(
        camera.distortionCoefficients,
        l.calibrationDistortionCoefficientsOffset,
        l.calibrationDistortionCoefficientCountOffset
      );

      const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
      const bodyFromCamera = finiteArray(camera.bodyFromCamera?.matrix, 16, identity);
      bodyFromCamera.forEach((value, index) => {
        m.setValue(calibrationPtr + l.calibrationBodyFromCameraOffset + index * 8, value, "double");
      });

      clearMemory(m, outPtr, 4);
      this._check(m._ma_occupancy_create(optionsPtr, calibrationPtr, outPtr), "occupancy grid create");
      this.handle = m.getValue(outPtr, "*");
    } finally {
      allocations.forEach((ptr) => m._free(ptr));
      m._free(outPtr);
      m._free(calibrationPtr);
      m._free(optionsPtr);
    }
  }

  close() {
    if (!this.handle) return;
    this.module._ma_occupancy_destroy(this.handle);
    this.handle = 0;
  }

  updatePose(pose) {
    const ptr = this.module._malloc(this.layout.poseSize);
    try {
      clearMemory(this.module, ptr, this.layout.poseSize);
      this._writePose(ptr, pose);
      this._check(this.module._ma_occupancy_update_pose(this.handle, ptr), "occupancy pose update");
    } finally {
      this.module._free(ptr);
    }
  }

  clear() {
    const ptr = this.module._malloc(this.layout.updateSize);
    try {
      clearMemory(this.module, ptr, this.layout.updateSize);
      this._check(this.module._ma_occupancy_clear(this.handle, ptr), "occupancy grid clear");
      return this._readUpdate(ptr);
    } finally {
      this.module._ma_occupancy_update_destroy(ptr);
      this.module._free(ptr);
    }
  }

  processFrames(frames) {
    if (!frames.length) return null;
    const m = this.module;
    const l = this.layout;
    const framesPtr = m._malloc(l.frameSize * frames.length);
    const updatePtr = m._malloc(l.updateSize);
    const imagePointers = [];
    try {
      clearMemory(m, framesPtr, l.frameSize * frames.length);
      clearMemory(m, updatePtr, l.updateSize);
      frames.forEach((frame, index) => {
        const framePtr = framesPtr + index * l.frameSize;
        const image = frame.image;
        const bytes = toU8(image.data);
        const dataPtr = m._malloc(bytes.length);
        imagePointers.push(dataPtr);
        m.HEAPU8.set(bytes, dataPtr);
        writeU64(m, framePtr + l.frameTimestampNsOffset, image.timestampNs);

        const imagePtr = framePtr + l.frameImageOffset;
        m.setValue(imagePtr + l.imageDataOffset, dataPtr, "*");
        m.setValue(imagePtr + l.imageSizeBytesOffset, bytes.length, "i32");
        m.setValue(imagePtr + l.imageWidthOffset, Number(image.width || 0), "i32");
        m.setValue(imagePtr + l.imageHeightOffset, Number(image.height || 0), "i32");
        m.setValue(imagePtr + l.imageStrideBytesOffset, imageStride(image), "i32");
        m.setValue(imagePtr + l.imageFormatOffset, nativePixelFormat(image.format), "i32");

        this._writePose(framePtr + l.framePoseOffset, frame.pose.pose);
        const metrics = frame.pose.metrics;
        m.setValue(framePtr + l.frameTranslationMetricsAvailableOffset, metrics.available ? 1 : 0, "i32");
        m.setValue(framePtr + l.frameTranslationConfidenceOffset, metrics.translationConfidence, "float");
        m.setValue(framePtr + l.frameTranslationObservabilityOffset, metrics.translationObservability, "float");
        m.setValue(framePtr + l.frameDegradedReasonFlagsOffset, metrics.degradedReasonFlags >>> 0, "i32");
      });
      this._check(
        m._ma_occupancy_process_frames(this.handle, framesPtr, frames.length, updatePtr),
        "occupancy frame processing"
      );
      return this._readUpdate(updatePtr);
    } finally {
      m._ma_occupancy_update_destroy(updatePtr);
      imagePointers.forEach((ptr) => m._free(ptr));
      m._free(updatePtr);
      m._free(framesPtr);
    }
  }

  stats() {
    const m = this.module;
    const l = this.layout;
    const ptr = m._malloc(l.statsSize);
    try {
      clearMemory(m, ptr, l.statsSize);
      this._check(m._ma_occupancy_stats(this.handle, ptr), "occupancy grid stats");
      return {
        inputFrames: readU64(m, ptr + l.statsInputFramesOffset),
        processedFrames: readU64(m, ptr + l.statsProcessedFramesOffset),
        integratedFrames: readU64(m, ptr + l.statsIntegratedFramesOffset),
        rejectedFrames: readU64(m, ptr + l.statsRejectedFramesOffset),
        integratedPoints: readU64(m, ptr + l.statsIntegratedPointsOffset),
        cellCount: m.getValue(ptr + l.statsCellCountOffset, "i32") >>> 0,
        occupiedCellCount: m.getValue(ptr + l.statsOccupiedCellCountOffset, "i32") >>> 0,
        memoryBytes: m.getValue(ptr + l.statsMemoryBytesOffset, "i32") >>> 0,
      };
    } finally {
      m._free(ptr);
    }
  }

  _writePose(ptr, pose) {
    const m = this.module;
    const l = this.layout;
    const position = finiteArray(pose.rawPositionM || pose.raw_position_m || pose.positionM || pose.position_m, 3, [0, 0, 0]);
    const orientation = finiteArray(pose.orientationXyzw || pose.orientation_xyzw, 4, [0, 0, 0, 1]);
    writeU64(m, ptr + l.poseTimestampNsOffset, pose.timestampNs ?? pose.timestamp_ns);
    m.setValue(ptr + l.posePxOffset, position[0], "double");
    m.setValue(ptr + l.posePyOffset, position[1], "double");
    m.setValue(ptr + l.posePzOffset, position[2], "double");
    m.setValue(ptr + l.poseQwOffset, orientation[3], "double");
    m.setValue(ptr + l.poseQxOffset, orientation[0], "double");
    m.setValue(ptr + l.poseQyOffset, orientation[1], "double");
    m.setValue(ptr + l.poseQzOffset, orientation[2], "double");
    m.setValue(ptr + l.poseFrameOffset, (pose.poseType || pose.pose_type) === "camera" ? 1 : 0, "i32");
    m.setValue(ptr + l.poseConfidenceOffset, Number(pose.confidence ?? 1), "float");
    m.setValue(ptr + l.poseIsKeyframeHintOffset, pose.isKeyframe || pose.is_keyframe ? 1 : 0, "i32");
  }

  _readUpdate(ptr) {
    const m = this.module;
    const l = this.layout;
    const count = m.getValue(ptr + l.updateChangeCountOffset, "i32") >>> 0;
    const cellsPtr = m.getValue(ptr + l.updateChangesOffset, "*");
    const indices = new Int32Array(count * 3);
    const states = new Uint8Array(count);
    const occupancy = new Uint8Array(count);
    const intensity = new Uint8Array(count);
    const support = new Uint8Array(count);
    const visible = new Uint8Array(count);
    for (let index = 0; index < count; index += 1) {
      const cell = cellsPtr + index * l.cellSize;
      indices[index * 3] = m.getValue(cell + l.cellIndexXOffset, "i32");
      indices[index * 3 + 1] = m.getValue(cell + l.cellIndexYOffset, "i32");
      indices[index * 3 + 2] = m.getValue(cell + l.cellIndexZOffset, "i32");
      states[index] = m.getValue(cell + l.cellStateOffset, "i32") & 0xff;
      occupancy[index] = m.getValue(cell + l.cellOccupancyOffset, "i8") & 0xff;
      intensity[index] = m.getValue(cell + l.cellIntensityOffset, "i8") & 0xff;
      support[index] = m.getValue(cell + l.cellSupportOffset, "i8") & 0xff;
      visible[index] = m.getValue(cell + l.cellVisibleOffset, "i8") & 0xff;
    }
    const hasCenter = !!m.getValue(ptr + l.updateHasRollingCenterOffset, "i32");
    return {
      baseRevision: readU64(m, ptr + l.updateBaseRevisionOffset),
      revision: readU64(m, ptr + l.updateRevisionOffset),
      timestampNs: readU64(m, ptr + l.updateTimestampNsOffset),
      reset: !!m.getValue(ptr + l.updateResetOffset, "i32"),
      resolutionM: m.getValue(ptr + l.updateResolutionMOffset, "float"),
      displayCellSizeM: m.getValue(ptr + l.updateDisplayCellSizeMOffset, "float"),
      rollingCenterM: hasCenter ? [0, 1, 2].map((axis) => (
        m.getValue(ptr + l.updateRollingCenterOffset + axis * 4, "float")
      )) : null,
      rollingRadiusM: m.getValue(ptr + l.updateHasRollingRadiusOffset, "i32")
        ? m.getValue(ptr + l.updateRollingRadiusMOffset, "float")
        : null,
      changeCount: count,
      indices,
      states,
      occupancy,
      intensity,
      support,
      visible,
    };
  }

  _check(status, operation) {
    if (status === 0) return;
    const messagePtr = this.module._ma_status_message(status);
    const message = messagePtr ? this.module.UTF8ToString(messagePtr) : "unknown error";
    throw new Error(`${operation} failed: ${message}`);
  }
}

export class MightyOccupancyGrid {
  constructor(client, module, options = {}) {
    if (!client?.onImage || !client?.onPose) {
      throw new Error("MightyOccupancyGrid requires a MightyClient");
    }
    this.client = client;
    this.options = { ...options };
    this.native = new NativeOccupancyGrid(
      module,
      selectCamera(options.calibration || options.calibrationYaml, options.cameraId),
      options
    );
    this.closed = false;
    this.autoProcess = options.autoProcess !== false;
    this.maxPendingFrames = options.maxPendingFrames == null
      ? null
      : Math.max(1, Math.floor(Number(options.maxPendingFrames) || 1));
    this.queue = [];
    this.queuedImages = 0;
    this.pendingImages = [];
    this.poses = [];
    this.latestMetrics = {
      available: false,
      translationConfidence: 1,
      translationObservability: 1,
      degradedReasonFlags: 0,
    };
    this.receivedFrames = 0;
    this.droppedFrames = 0;
    this.processing = false;
    this.processScheduled = false;
    this.onUpdate = typeof options.onUpdate === "function" ? options.onUpdate : null;
    this.onError = typeof options.onError === "function" ? options.onError : null;
    this.unsubscribers = [
      client.onImage((image) => this._receiveImage(image)),
      client.onPose((pose) => this._enqueuePose(pose)),
      client.onVioState((metrics) => this._enqueue({ type: "metrics", metrics })),
      client.onReset(() => this._enqueue({ type: "reset" })),
    ];
  }

  close() {
    if (this.closed) return;
    this.closed = true;
    this.unsubscribers.splice(0).forEach((unsubscribe) => unsubscribe?.());
    this.queue.length = 0;
    this.pendingImages.length = 0;
    this.poses.length = 0;
    this.native.close();
  }

  clear() {
    if (this.closed) return null;
    this.queue.length = 0;
    this.pendingImages.length = 0;
    this.poses.length = 0;
    this.queuedImages = 0;
    const update = this.native.clear();
    this._emitUpdate(update);
    return update;
  }

  process() {
    if (this.closed || this.processing || this.queue.length === 0) return false;
    this.processing = true;
    const batch = this.queue.splice(0);
    this.queuedImages = 0;
    try {
      for (const event of batch) {
        if (event.type === "metrics") {
          this.latestMetrics = {
            available: event.metrics?.translationConfidence01 != null
              || event.metrics?.translationObservability01 != null,
            translationConfidence: Number(event.metrics?.translationConfidence01 ?? 1),
            translationObservability: Number(event.metrics?.translationObservability01 ?? 1),
            degradedReasonFlags: Number(event.metrics?.degradedReasonFlags ?? 0) >>> 0,
          };
        } else if (event.type === "pose") {
          this.native.updatePose(event.pose);
          this.poses.push({ pose: event.pose, metrics: { ...this.latestMetrics } });
          if (this.poses.length > POSE_HISTORY_LIMIT) this.poses.splice(0, this.poses.length - POSE_HISTORY_LIMIT);
        } else if (event.type === "image") {
          this.pendingImages.push(event.image);
          if (this.maxPendingFrames != null && this.pendingImages.length > this.maxPendingFrames) {
            const removed = this.pendingImages.length - this.maxPendingFrames;
            this.pendingImages.splice(0, removed);
            this.droppedFrames += removed;
          }
        } else if (event.type === "reset") {
          this.pendingImages.length = 0;
          this.poses.length = 0;
          this._emitUpdate(this.native.clear());
        }
      }

      const ready = this._matchFrames();
      const update = this.native.processFrames(ready);
      if (update) this._emitUpdate(update);
      return true;
    } finally {
      this.processing = false;
      if (this.autoProcess && this.queue.length) this._scheduleProcess();
    }
  }

  stats() {
    return {
      ...this.native.stats(),
      receivedFrames: this.receivedFrames,
      droppedFrames: this.droppedFrames,
      pendingFrames: this.queuedImages + this.pendingImages.length,
    };
  }

  _receiveImage(image) {
    const selected = selectRawImage(image);
    if (!selected || !primaryChannel(selected) || timestampNs(selected.timestampNs) === 0n) return;
    if (selected.kind === "jpg") {
      void decodeJpeg(selected).then((decoded) => {
        if (!this.closed) this._enqueueImage(decoded);
      }).catch((error) => this._reportError(error));
      return;
    }
    this._enqueueImage(selected);
  }

  _enqueueImage(image) {
    this.receivedFrames += 1;
    this._enqueue({ type: "image", image });
  }

  _enqueuePose(pose) {
    const type = pose?.poseType || pose?.pose_type || "";
    const orientation = pose?.orientationXyzw || pose?.orientation_xyzw;
    if (pose?.isPublic === false || pose?.is_public === false || !orientation
        || !["", "body", "camera"].includes(type)
        || timestampNs(pose?.timestampNs ?? pose?.timestamp_ns) === 0n) return;
    this._enqueue({ type: "pose", pose });
  }

  _enqueue(event) {
    if (this.closed) return;
    if (event.type === "image" && this.maxPendingFrames != null
        && this.queuedImages >= this.maxPendingFrames) {
      const oldest = this.queue.findIndex((candidate) => candidate.type === "image");
      if (oldest >= 0) {
        this.queue.splice(oldest, 1);
        this.queuedImages -= 1;
        this.droppedFrames += 1;
      }
    }
    if (event.type === "image") this.queuedImages += 1;
    this.queue.push(event);
    if (this.autoProcess) this._scheduleProcess();
  }

  _scheduleProcess() {
    if (this.closed || this.processScheduled) return;
    this.processScheduled = true;
    setTimeout(() => {
      this.processScheduled = false;
      if (this.closed) return;
      try {
        this.process();
      } catch (error) {
        this._reportError(error);
      }
    }, 0);
  }

  _matchFrames() {
    const ready = [];
    for (let imageIndex = 0; imageIndex < this.pendingImages.length;) {
      const image = this.pendingImages[imageIndex];
      const imageTimestamp = timestampNs(image.timestampNs);
      let bestIndex = -1;
      let bestDelta = null;
      for (let poseIndex = 0; poseIndex < this.poses.length; poseIndex += 1) {
        const poseTimestamp = timestampNs(this.poses[poseIndex].pose.timestampNs ?? this.poses[poseIndex].pose.timestamp_ns);
        const delta = imageTimestamp > poseTimestamp ? imageTimestamp - poseTimestamp : poseTimestamp - imageTimestamp;
        if (bestDelta === null || delta < bestDelta) {
          bestDelta = delta;
          bestIndex = poseIndex;
        }
      }
      if (bestIndex >= 0 && bestDelta <= MATCH_TOLERANCE_NS) {
        ready.push({ image, pose: this.poses[bestIndex] });
        this.pendingImages.splice(imageIndex, 1);
        continue;
      }
      const newestPose = this.poses.at(-1);
      const newestTimestamp = timestampNs(newestPose?.pose?.timestampNs ?? newestPose?.pose?.timestamp_ns);
      if (newestTimestamp > imageTimestamp + MATCH_TOLERANCE_NS) {
        this.pendingImages.splice(imageIndex, 1);
        this.droppedFrames += 1;
        continue;
      }
      imageIndex += 1;
    }

    if (this.pendingImages.length && this.poses.length) {
      const oldestImage = timestampNs(this.pendingImages[0].timestampNs);
      const minimum = oldestImage > MATCH_TOLERANCE_NS ? oldestImage - MATCH_TOLERANCE_NS : 0n;
      while (this.poses.length
          && timestampNs(this.poses[0].pose.timestampNs ?? this.poses[0].pose.timestamp_ns) < minimum) {
        this.poses.shift();
      }
    }
    return ready;
  }

  _emitUpdate(update) {
    if (!update) return;
    try {
      this.onUpdate?.(update);
    } catch (error) {
      this._reportError(error);
    }
  }

  _reportError(error) {
    if (this.closed) return;
    if (this.onError) this.onError(error);
    else if (typeof console !== "undefined") console.error("Mighty occupancy grid error", error);
  }
}

export const OCCUPANCY_STATE = Object.freeze({ UNKNOWN: 0, FREE: 1, OCCUPIED });
