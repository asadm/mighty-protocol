from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np


@dataclass
class DoubleSphereCamera:
    xi: float
    alpha: float
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_kalibr(cls, camera: Dict[str, Any]) -> "DoubleSphereCamera":
        model = str(camera.get("camera_model", "")).lower()
        if model != "ds":
            raise ValueError(f"unsupported camera_model {model!r}; expected Kalibr double-sphere 'ds'")
        intrinsics = list(camera.get("intrinsics") or [])
        resolution = list(camera.get("resolution") or [])
        if len(intrinsics) != 6:
            raise ValueError("double-sphere camera requires intrinsics [xi, alpha, fx, fy, cx, cy]")
        if len(resolution) != 2:
            raise ValueError("camera requires resolution [width, height]")
        return cls(
            xi=float(intrinsics[0]),
            alpha=float(intrinsics[1]),
            fx=float(intrinsics[2]),
            fy=float(intrinsics[3]),
            cx=float(intrinsics[4]),
            cy=float(intrinsics[5]),
            width=int(resolution[0]),
            height=int(resolution[1]),
        )

    def scaled_to(self, width: int, height: int) -> "DoubleSphereCamera":
        sx = float(width) / max(1.0, float(self.width))
        sy = float(height) / max(1.0, float(self.height))
        return DoubleSphereCamera(
            xi=self.xi,
            alpha=self.alpha,
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=int(width),
            height=int(height),
        )

    def pinhole_matrix(self) -> np.ndarray:
        return np.asarray(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass
class StereoDepthSetup:
    map1x: np.ndarray
    map1y: np.ndarray
    map2x: np.ndarray
    map2y: np.ndarray
    sgbm_matcher: Any
    width: int
    height: int
    raw_width: int
    raw_height: int
    num_disparities: int
    min_disparity: int
    focal_px: float
    baseline_m: float


def image_to_gray_u8(cv2: Any, image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        gray = image
    elif image.shape[2] == 4:
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if gray.dtype == np.uint8:
        return gray
    clipped = np.clip(gray, 0, 255)
    return clipped.astype(np.uint8)


def project_double_sphere(camera: DoubleSphereCamera, rays: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    x = rays[0]
    y = rays[1]
    z = rays[2]
    d1 = np.sqrt(x * x + y * y + z * z)
    z_xi = camera.xi * d1 + z
    d2 = np.sqrt(x * x + y * y + z_xi * z_xi)
    denom = camera.alpha * d2 + (1.0 - camera.alpha) * z_xi

    valid = np.isfinite(denom) & (denom > 1e-8)
    u = np.full_like(x, -1.0, dtype=np.float64)
    v = np.full_like(y, -1.0, dtype=np.float64)
    u[valid] = camera.fx * x[valid] / denom[valid] + camera.cx
    v[valid] = camera.fy * y[valid] / denom[valid] + camera.cy
    valid &= np.isfinite(u) & np.isfinite(v)
    valid &= (u >= 0.0) & (u < float(camera.width - 1)) & (v >= 0.0) & (v < float(camera.height - 1))
    u[~valid] = -1.0
    v[~valid] = -1.0
    return u.astype(np.float32), v.astype(np.float32)


def build_ds_rectify_maps(
    camera: DoubleSphereCamera,
    rect_rotation: np.ndarray,
    projection: np.ndarray,
    width: int,
    height: int,
) -> Tuple[np.ndarray, np.ndarray]:
    xs, ys = np.meshgrid(
        np.arange(width, dtype=np.float64),
        np.arange(height, dtype=np.float64),
    )
    fx = float(projection[0, 0])
    fy = float(projection[1, 1])
    cx = float(projection[0, 2])
    cy = float(projection[1, 2])
    x_rect = (xs.reshape(-1) - cx) / max(abs(fx), 1e-9)
    y_rect = (ys.reshape(-1) - cy) / max(abs(fy), 1e-9)
    rays_rect = np.vstack([x_rect, y_rect, np.ones_like(x_rect)])
    rays_camera = np.asarray(rect_rotation, dtype=np.float64).T @ rays_rect
    map_x, map_y = project_double_sphere(camera, rays_camera)
    return map_x.reshape((height, width)), map_y.reshape((height, width))


def colorize_metric_depth(
    cv2: Any,
    disparity: np.ndarray,
    min_disp: int,
    focal_px: float,
    baseline_m: float,
    min_depth_m: float,
    max_depth_m: float,
) -> Tuple[np.ndarray, Dict[str, float]]:
    finite = np.isfinite(disparity)
    valid = finite & (disparity > float(min_disp))
    depth_m = np.full(disparity.shape, np.nan, dtype=np.float32)
    depth_m[valid] = float(focal_px) * float(baseline_m) / np.maximum(disparity[valid], 1e-6)
    valid_depth = valid & np.isfinite(depth_m) & (depth_m > 0.0)
    in_range = valid_depth & (depth_m >= float(min_depth_m)) & (depth_m <= float(max_depth_m))
    stats = {
        "valid_pct": 0.0,
        "in_range_pct": 0.0,
        "median_depth_m": 0.0,
        "p05_depth_m": 0.0,
        "p95_depth_m": 0.0,
    }
    normalized = np.zeros(disparity.shape, dtype=np.uint8)
    if np.any(valid_depth):
        values = depth_m[valid_depth]
        span = max(float(max_depth_m) - float(min_depth_m), 1e-6)
        normalized[valid_depth] = np.clip((float(max_depth_m) - depth_m[valid_depth]) * 255.0 / span, 0, 255).astype(np.uint8)
        stats["valid_pct"] = float(np.count_nonzero(valid_depth)) * 100.0 / float(valid_depth.size)
        stats["in_range_pct"] = float(np.count_nonzero(in_range)) * 100.0 / float(valid_depth.size)
        stats["median_depth_m"] = float(np.median(values))
        stats["p05_depth_m"] = float(np.percentile(values, 5.0))
        stats["p95_depth_m"] = float(np.percentile(values, 95.0))
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[~valid_depth] = (18, 18, 18)
    return colored, stats


class StereoDepthProcessor:
    def __init__(self, calib_path: Path, depth_scale: float = 0.5, min_depth_m: float = 0.2, max_depth_m: float = 5.0):
        self.calib_path = Path(calib_path).expanduser()
        if not self.calib_path.exists():
            raise FileNotFoundError(f"depth calibration YAML not found: {self.calib_path}")
        self.depth_scale = min(1.0, max(0.2, float(depth_scale)))
        self.min_depth_m = max(0.01, float(min_depth_m))
        self.max_depth_m = max(self.min_depth_m + 0.01, float(max_depth_m))
        self.left_camera, self.right_camera, self.right_from_left = self._load_calibration(self.calib_path)
        self.baseline_m = float(np.linalg.norm(self.right_from_left[:3, 3]))
        self._setup_key: Optional[Tuple[int, int, int, int]] = None
        self._setup: Optional[StereoDepthSetup] = None

    @staticmethod
    def _load_calibration(calib_path: Path) -> Tuple[DoubleSphereCamera, DoubleSphereCamera, np.ndarray]:
        try:
            import yaml
        except Exception as exc:
            raise RuntimeError("PyYAML is required for --depth. Install with: pip install PyYAML") from exc
        with calib_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        cam0 = data.get("cam0") or {}
        cam1 = data.get("cam1") or {}
        if not cam0 or not cam1:
            raise ValueError("calibration YAML must contain cam0 and cam1")
        transform = cam1.get("T_cn_cnm1")
        if transform is None:
            raise ValueError("cam1.T_cn_cnm1 is required for stereo rectification")
        right_from_left = np.asarray(transform, dtype=np.float64)
        if right_from_left.shape != (4, 4):
            raise ValueError("cam1.T_cn_cnm1 must be a 4x4 matrix")
        return DoubleSphereCamera.from_kalibr(cam0), DoubleSphereCamera.from_kalibr(cam1), right_from_left

    def _make_setup(self, cv2: Any, raw_width: int, raw_height: int) -> StereoDepthSetup:
        rect_width = max(96, int(round(float(raw_width) * self.depth_scale)))
        rect_height = max(64, int(round(float(raw_height) * self.depth_scale)))
        key = (int(raw_width), int(raw_height), int(rect_width), int(rect_height))
        if self._setup is not None and self._setup_key == key:
            return self._setup

        left = self.left_camera.scaled_to(raw_width, raw_height)
        right = self.right_camera.scaled_to(raw_width, raw_height)
        r_lr = self.right_from_left[:3, :3]
        t_lr = self.right_from_left[:3, 3].reshape(3, 1)
        r1, r2, p1, p2, _q, _roi1, _roi2 = cv2.stereoRectify(
            left.pinhole_matrix(),
            np.zeros(5, dtype=np.float64),
            right.pinhole_matrix(),
            np.zeros(5, dtype=np.float64),
            (int(raw_width), int(raw_height)),
            r_lr,
            t_lr,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0,
        )
        sx = float(rect_width) / float(raw_width)
        sy = float(rect_height) / float(raw_height)
        p1_scaled = p1.copy()
        p2_scaled = p2.copy()
        p1_scaled[0, :] *= sx
        p1_scaled[1, :] *= sy
        p2_scaled[0, :] *= sx
        p2_scaled[1, :] *= sy

        map1x, map1y = build_ds_rectify_maps(left, r1, p1_scaled, rect_width, rect_height)
        map2x, map2y = build_ds_rectify_maps(right, r2, p2_scaled, rect_width, rect_height)

        num_disparities = int(math.ceil(max(64.0, float(rect_width) / 4.0) / 16.0) * 16)
        num_disparities = min(192, max(16, num_disparities))
        min_disparity = 0
        block_size = 5
        sgbm_matcher = cv2.StereoSGBM_create(
            minDisparity=min_disparity,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            preFilterCap=31,
            uniquenessRatio=10,
            speckleWindowSize=80,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM,
        )
        setup = StereoDepthSetup(
            map1x=map1x,
            map1y=map1y,
            map2x=map2x,
            map2y=map2y,
            sgbm_matcher=sgbm_matcher,
            width=rect_width,
            height=rect_height,
            raw_width=int(raw_width),
            raw_height=int(raw_height),
            num_disparities=num_disparities,
            min_disparity=min_disparity,
            focal_px=float(p1_scaled[0, 0]),
            baseline_m=self.baseline_m,
        )
        self._setup_key = key
        self._setup = setup
        return setup

    def compute(self, cv2: Any, left_image: np.ndarray, right_image: np.ndarray) -> Dict[str, Any]:
        started_s = time.perf_counter()
        left_gray = image_to_gray_u8(cv2, left_image)
        right_gray = image_to_gray_u8(cv2, right_image)
        raw_height, raw_width = left_gray.shape[:2]
        if right_gray.shape[:2] != (raw_height, raw_width):
            right_gray = cv2.resize(right_gray, (raw_width, raw_height), interpolation=cv2.INTER_AREA)

        setup = self._make_setup(cv2, raw_width, raw_height)
        left_rect = cv2.remap(
            left_gray,
            setup.map1x,
            setup.map1y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        right_rect = cv2.remap(
            right_gray,
            setup.map2x,
            setup.map2y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        sgbm_started_s = time.perf_counter()
        disparity = setup.sgbm_matcher.compute(left_rect, right_rect).astype(np.float32) / 16.0
        sgbm_view, sgbm_stats = colorize_metric_depth(
            cv2,
            disparity,
            setup.min_disparity,
            setup.focal_px,
            setup.baseline_m,
            self.min_depth_m,
            self.max_depth_m,
        )
        sgbm_compute_ms = (time.perf_counter() - sgbm_started_s) * 1000.0

        elapsed_ms = (time.perf_counter() - started_s) * 1000.0
        return {
            "left_rect": cv2.cvtColor(left_rect, cv2.COLOR_GRAY2BGR),
            "right_rect": cv2.cvtColor(right_rect, cv2.COLOR_GRAY2BGR),
            "sgbm": sgbm_view,
            "stats": {
                "elapsed_ms": elapsed_ms,
                "width": setup.width,
                "height": setup.height,
                "num_disparities": setup.num_disparities,
                "focal_px": setup.focal_px,
                "baseline_m": setup.baseline_m,
                "min_depth_m": self.min_depth_m,
                "max_depth_m": self.max_depth_m,
                "sgbm_valid_pct": sgbm_stats["valid_pct"],
                "sgbm_in_range_pct": sgbm_stats["in_range_pct"],
                "sgbm_median_depth_m": sgbm_stats["median_depth_m"],
                "sgbm_p05_depth_m": sgbm_stats["p05_depth_m"],
                "sgbm_p95_depth_m": sgbm_stats["p95_depth_m"],
                "sgbm_compute_ms": sgbm_compute_ms,
            },
        }

    def summary(self) -> str:
        return (
            f"{self.calib_path} baseline={self.baseline_m:.4f}m "
            f"scale={self.depth_scale:.2f} range={self.min_depth_m:.2f}-{self.max_depth_m:.2f}m"
        )
