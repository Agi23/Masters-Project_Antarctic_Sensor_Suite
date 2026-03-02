#!/usr/bin/env python3
"""Interactive synchronized LiDAR + stereo frame viewer."""

from __future__ import annotations

import argparse
import csv
import functools
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import matplotlib.pyplot as plt
import numpy as np

IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")
TS_FROM_NAME_RE = re.compile(r"_(\d{10,})\.[^.]+$")
FRAME_FROM_LIDAR_RE = re.compile(r"^lidar_(\d+)\.pcd$", re.IGNORECASE)


@dataclass(frozen=True)
class FrameRecord:
    frame_index: int
    lidar_file: Path
    camera_a_file: Path
    camera_b_file: Path
    lidar_event_time_ns: int | None
    camera_a_capture_time_ns: int | None
    camera_b_capture_time_ns: int | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "View synchronized LiDAR scans with stereo image pairs and timing offsets. "
            "Use keyboard arrows to move frame-by-frame."
        )
    )
    parser.add_argument(
        "-session-dir",
        "--session-dir",
        required=True,
        help="Path to capture session directory (e.g. captures/session_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument(
        "--metadata-csv",
        default="",
        help="Optional override for metadata CSV path (default: <session>/metadata/frame_metadata.csv)",
    )
    parser.add_argument(
        "--camera-a",
        default="",
        help="Optional camera A prefix override (defaults to metadata detection, then left/right)",
    )
    parser.add_argument(
        "--camera-b",
        default="",
        help="Optional camera B prefix override (defaults to metadata detection, then left/right)",
    )
    parser.add_argument(
        "--max-lidar-points",
        type=int,
        default=30000,
        help="Maximum LiDAR points displayed per frame (default: 30000)",
    )
    parser.add_argument(
        "--lidar-point-size",
        type=float,
        default=0.6,
        help="Scatter marker size for LiDAR points (default: 0.6)",
    )
    return parser.parse_args()


def safe_int(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def parse_timestamp_from_name(path: Path) -> int | None:
    match = TS_FROM_NAME_RE.search(path.name)
    if not match:
        return None
    return safe_int(match.group(1))


def resolve_relative(session_dir: Path, file_value: str) -> Path:
    path = Path(file_value)
    return path if path.is_absolute() else session_dir / path


def infer_camera_names_from_header(fieldnames: Sequence[str]) -> tuple[str, str] | None:
    candidates = [name[:-5] for name in fieldnames if name.endswith("_file") and name != "lidar_file"]
    unique = sorted(set(candidates))
    if len(unique) < 2:
        return None
    if "left" in unique and "right" in unique:
        return ("left", "right")
    return (unique[0], unique[1])


def detect_camera_names(session_dir: Path, metadata_path: Path, cam_a_arg: str, cam_b_arg: str) -> tuple[str, str]:
    if cam_a_arg and cam_b_arg:
        return cam_a_arg, cam_b_arg

    metadata_detected: tuple[str, str] | None = None
    if metadata_path.exists():
        with metadata_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames:
                metadata_detected = infer_camera_names_from_header(reader.fieldnames)

    if metadata_detected:
        camera_a = cam_a_arg or metadata_detected[0]
        camera_b = cam_b_arg or metadata_detected[1]
        if camera_a == camera_b:
            raise ValueError(f"Camera names must be different, got '{camera_a}'")
        return camera_a, camera_b

    stereo_dir = session_dir / "stereo"
    if stereo_dir.exists():
        dirs = sorted(entry.name for entry in stereo_dir.iterdir() if entry.is_dir())
        if "left" in dirs and "right" in dirs:
            camera_a = cam_a_arg or "left"
            camera_b = cam_b_arg or "right"
            if camera_a == camera_b:
                raise ValueError(f"Camera names must be different, got '{camera_a}'")
            return camera_a, camera_b
        if len(dirs) >= 2:
            camera_a = cam_a_arg or dirs[0]
            camera_b = cam_b_arg or dirs[1]
            if camera_a == camera_b:
                raise ValueError(f"Camera names must be different, got '{camera_a}'")
            return camera_a, camera_b

    if cam_a_arg and not cam_b_arg:
        raise ValueError("--camera-b is required when --camera-a is set")
    if cam_b_arg and not cam_a_arg:
        raise ValueError("--camera-a is required when --camera-b is set")

    raise ValueError(
        "Could not detect stereo camera names. Pass --camera-a and --camera-b explicitly."
    )


def find_image_for_frame(camera_dir: Path, camera_name: str, frame_index: int) -> Path | None:
    for ext in IMAGE_EXTENSIONS:
        pattern = f"{camera_name}_{frame_index:06d}_*{ext}"
        matches = sorted(camera_dir.glob(pattern))
        if matches:
            return matches[0]
    return None


def build_frames_from_metadata(
    session_dir: Path,
    metadata_path: Path,
    camera_a: str,
    camera_b: str,
) -> list[FrameRecord]:
    rows_by_index: dict[int, dict[str, str]] = {}

    with metadata_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            frame_index = safe_int(row.get("frame_index"))
            if frame_index is None:
                continue
            rows_by_index[frame_index] = row

    lidar_dir = session_dir / "lidar" / "frames"
    cam_a_dir = session_dir / "stereo" / camera_a
    cam_b_dir = session_dir / "stereo" / camera_b
    records: list[FrameRecord] = []

    for frame_index in sorted(rows_by_index):
        row = rows_by_index[frame_index]

        lidar_file_value = row.get("lidar_file", "").strip()
        lidar_file = resolve_relative(session_dir, lidar_file_value) if lidar_file_value else (
            lidar_dir / f"lidar_{frame_index:06d}.pcd"
        )
        cam_a_file_value = row.get(f"{camera_a}_file", "").strip()
        cam_b_file_value = row.get(f"{camera_b}_file", "").strip()
        cam_a_file = (
            resolve_relative(session_dir, cam_a_file_value)
            if cam_a_file_value
            else find_image_for_frame(cam_a_dir, camera_a, frame_index)
        )
        cam_b_file = (
            resolve_relative(session_dir, cam_b_file_value)
            if cam_b_file_value
            else find_image_for_frame(cam_b_dir, camera_b, frame_index)
        )

        if cam_a_file is None or cam_b_file is None:
            continue
        if not lidar_file.exists() or not cam_a_file.exists() or not cam_b_file.exists():
            continue

        records.append(
            FrameRecord(
                frame_index=frame_index,
                lidar_file=lidar_file,
                camera_a_file=cam_a_file,
                camera_b_file=cam_b_file,
                lidar_event_time_ns=safe_int(row.get("lidar_event_time_ns")),
                camera_a_capture_time_ns=safe_int(row.get(f"{camera_a}_capture_time_ns")),
                camera_b_capture_time_ns=safe_int(row.get(f"{camera_b}_capture_time_ns")),
            )
        )

    return records


def collect_lidar_files(session_dir: Path) -> dict[int, Path]:
    lidar_files: dict[int, Path] = {}
    lidar_dir = session_dir / "lidar" / "frames"
    for lidar_file in sorted(lidar_dir.glob("*.pcd")):
        match = FRAME_FROM_LIDAR_RE.match(lidar_file.name)
        if not match:
            continue
        lidar_files[int(match.group(1))] = lidar_file
    return lidar_files


def collect_camera_files(session_dir: Path, camera_name: str) -> tuple[dict[int, Path], dict[int, int | None]]:
    frame_to_file: dict[int, Path] = {}
    frame_to_ts: dict[int, int | None] = {}
    camera_dir = session_dir / "stereo" / camera_name
    for ext in IMAGE_EXTENSIONS:
        for image_file in sorted(camera_dir.glob(f"{camera_name}_*_*{ext}")):
            parts = image_file.stem.split("_")
            if len(parts) < 3:
                continue
            frame_index = safe_int(parts[-2])
            if frame_index is None:
                continue
            frame_to_file[frame_index] = image_file
            frame_to_ts[frame_index] = parse_timestamp_from_name(image_file)
    return frame_to_file, frame_to_ts


def build_frames_without_metadata(session_dir: Path, camera_a: str, camera_b: str) -> list[FrameRecord]:
    lidar_files = collect_lidar_files(session_dir)
    cam_a_files, cam_a_ts = collect_camera_files(session_dir, camera_a)
    cam_b_files, cam_b_ts = collect_camera_files(session_dir, camera_b)

    frame_indices = sorted(set(lidar_files) & set(cam_a_files) & set(cam_b_files))
    return [
        FrameRecord(
            frame_index=frame_index,
            lidar_file=lidar_files[frame_index],
            camera_a_file=cam_a_files[frame_index],
            camera_b_file=cam_b_files[frame_index],
            lidar_event_time_ns=None,
            camera_a_capture_time_ns=cam_a_ts.get(frame_index),
            camera_b_capture_time_ns=cam_b_ts.get(frame_index),
        )
        for frame_index in frame_indices
    ]


def load_records(
    session_dir: Path,
    metadata_path: Path,
    camera_a: str,
    camera_b: str,
) -> list[FrameRecord]:
    if metadata_path.exists():
        records = build_frames_from_metadata(
            session_dir=session_dir,
            metadata_path=metadata_path,
            camera_a=camera_a,
            camera_b=camera_b,
        )
        if records:
            return records
    return build_frames_without_metadata(session_dir=session_dir, camera_a=camera_a, camera_b=camera_b)


def read_ascii_pcd(pcd_path: Path) -> np.ndarray:
    header_lines = 0
    data_mode = ""
    with pcd_path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        for line_num, line in enumerate(file_obj, start=1):
            stripped = line.strip()
            if stripped.upper().startswith("DATA"):
                data_mode = stripped.split(maxsplit=1)[-1].lower()
                header_lines = line_num
                break

    if header_lines == 0:
        raise ValueError(f"PCD header missing DATA line: {pcd_path}")
    if data_mode != "ascii":
        raise ValueError(f"Only DATA ascii PCD files are supported in this viewer: {pcd_path}")

    points = np.loadtxt(str(pcd_path), dtype=np.float32, skiprows=header_lines)
    if points.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    if points.ndim == 1:
        points = np.expand_dims(points, axis=0)
    if points.shape[1] < 3:
        raise ValueError(f"Expected at least 3 columns (x,y,z) in {pcd_path}")
    if points.shape[1] == 3:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.hstack((points, intensity))
    return points[:, :4]


@functools.lru_cache(maxsize=16)
def load_lidar_points_cached(pcd_path_str: str, max_points: int) -> np.ndarray:
    points = read_ascii_pcd(Path(pcd_path_str))
    if max_points > 0 and len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[indices]
    return points


@functools.lru_cache(maxsize=32)
def load_image_cached(image_path_str: str) -> np.ndarray:
    image_bgr = cv2.imread(image_path_str, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError(f"Failed to read image: {image_path_str}")
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def format_timestamp(value: int | None) -> str:
    return str(value) if value is not None else "N/A"


def format_delta_ns(reference_ns: int | None, target_ns: int | None) -> str:
    if reference_ns is None or target_ns is None:
        return "N/A"
    delta_ms = (target_ns - reference_ns) / 1e6
    return f"{delta_ms:+.3f} ms"


class SyncViewer:
    def __init__(
        self,
        records: list[FrameRecord],
        camera_a: str,
        camera_b: str,
        max_lidar_points: int,
        lidar_point_size: float,
    ) -> None:
        self.records = records
        self.camera_a = camera_a
        self.camera_b = camera_b
        self.max_lidar_points = max_lidar_points
        self.lidar_point_size = lidar_point_size
        self.index = 0

        self.fig = plt.figure("LiDAR + Stereo Sync Viewer", figsize=(18, 6.5))
        grid = self.fig.add_gridspec(1, 3, width_ratios=[1.2, 1.0, 1.0])
        self.ax_lidar = self.fig.add_subplot(grid[0, 0], projection="3d")
        self.ax_cam_a = self.fig.add_subplot(grid[0, 1])
        self.ax_cam_b = self.fig.add_subplot(grid[0, 2])
        self.fig.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.render_current_frame()

    def move(self, offset: int) -> None:
        self.index = max(0, min(self.index + offset, len(self.records) - 1))
        self.render_current_frame()

    def jump_to(self, index: int) -> None:
        self.index = max(0, min(index, len(self.records) - 1))
        self.render_current_frame()

    def on_key_press(self, event: object) -> None:
        key = getattr(event, "key", None)
        if key in ("right", "d"):
            self.move(1)
        elif key in ("left", "a"):
            self.move(-1)
        elif key in ("up", "w"):
            self.move(10)
        elif key in ("down", "s"):
            self.move(-10)
        elif key == "home":
            self.jump_to(0)
        elif key == "end":
            self.jump_to(len(self.records) - 1)
        elif key in ("q", "escape"):
            plt.close(self.fig)

    def render_current_frame(self) -> None:
        record = self.records[self.index]

        lidar_view = self.capture_lidar_view()
        self.ax_lidar.clear()
        self.ax_cam_a.clear()
        self.ax_cam_b.clear()

        self.draw_lidar(record, lidar_view)
        self.draw_camera(self.ax_cam_a, record.camera_a_file, self.camera_a)
        self.draw_camera(self.ax_cam_b, record.camera_b_file, self.camera_b)

        lidar_ts = record.lidar_event_time_ns
        a_ts = record.camera_a_capture_time_ns
        b_ts = record.camera_b_capture_time_ns
        frame_count = len(self.records)

        title = (
            f"Frame {self.index + 1}/{frame_count}  (frame_index={record.frame_index})\n"
            f"LiDAR ns: {format_timestamp(lidar_ts)} | "
            f"{self.camera_a} ns: {format_timestamp(a_ts)} | "
            f"{self.camera_b} ns: {format_timestamp(b_ts)}\n"
            f"{self.camera_a}-LiDAR: {format_delta_ns(lidar_ts, a_ts)} | "
            f"{self.camera_b}-LiDAR: {format_delta_ns(lidar_ts, b_ts)} | "
            f"{self.camera_b}-{self.camera_a}: {format_delta_ns(a_ts, b_ts)}\n"
            "Keys: Left/Right=prev/next, Up/Down=+/-10, Home/End=first/last, Q/Esc=quit | "
            "LiDAR: mouse-drag rotate, scroll zoom, toolbar Pan tool to pan"
        )
        self.fig.suptitle(title, fontsize=10)
        self.fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.88))
        self.fig.canvas.draw_idle()

    def capture_lidar_view(self) -> dict[str, object] | None:
        if not self.ax_lidar.collections:
            return None
        return {
            "elev": float(self.ax_lidar.elev),
            "azim": float(self.ax_lidar.azim),
            "xlim": tuple(self.ax_lidar.get_xlim3d()),
            "ylim": tuple(self.ax_lidar.get_ylim3d()),
            "zlim": tuple(self.ax_lidar.get_zlim3d()),
        }

    def style_lidar_axis(self) -> None:
        self.ax_lidar.set_facecolor("black")
        self.ax_lidar.xaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
        self.ax_lidar.yaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
        self.ax_lidar.zaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))

        self.ax_lidar.tick_params(colors="white")
        self.ax_lidar.xaxis.label.set_color("white")
        self.ax_lidar.yaxis.label.set_color("white")
        self.ax_lidar.zaxis.label.set_color("white")
        self.ax_lidar.title.set_color("white")

        for axis in (self.ax_lidar.xaxis, self.ax_lidar.yaxis, self.ax_lidar.zaxis):
            axis._axinfo["grid"]["color"] = (0.8, 0.8, 0.8, 0.25)
            axis._axinfo["grid"]["linewidth"] = 0.5

    def apply_lidar_view(self, lidar_view: dict[str, object] | None) -> None:
        if lidar_view is None:
            self.ax_lidar.view_init(elev=20.0, azim=-60.0)
            return
        self.ax_lidar.view_init(elev=float(lidar_view["elev"]), azim=float(lidar_view["azim"]))
        xlim = lidar_view["xlim"]
        ylim = lidar_view["ylim"]
        zlim = lidar_view["zlim"]
        if isinstance(xlim, tuple) and isinstance(ylim, tuple) and isinstance(zlim, tuple):
            self.ax_lidar.set_xlim3d(*xlim)
            self.ax_lidar.set_ylim3d(*ylim)
            self.ax_lidar.set_zlim3d(*zlim)

    def set_equal_3d_limits(self, x_vals: np.ndarray, y_vals: np.ndarray, z_vals: np.ndarray) -> None:
        x_mid = float((np.min(x_vals) + np.max(x_vals)) / 2.0)
        y_mid = float((np.min(y_vals) + np.max(y_vals)) / 2.0)
        z_mid = float((np.min(z_vals) + np.max(z_vals)) / 2.0)

        x_range = float(np.max(x_vals) - np.min(x_vals))
        y_range = float(np.max(y_vals) - np.min(y_vals))
        z_range = float(np.max(z_vals) - np.min(z_vals))
        max_half_range = max(x_range, y_range, z_range) / 2.0
        max_half_range = max(max_half_range, 0.5)

        self.ax_lidar.set_xlim3d(x_mid - max_half_range, x_mid + max_half_range)
        self.ax_lidar.set_ylim3d(y_mid - max_half_range, y_mid + max_half_range)
        self.ax_lidar.set_zlim3d(z_mid - max_half_range, z_mid + max_half_range)

    def draw_lidar(self, record: FrameRecord, lidar_view: dict[str, object] | None) -> None:
        self.style_lidar_axis()
        try:
            points = load_lidar_points_cached(str(record.lidar_file), self.max_lidar_points)
        except Exception as error:  # pylint: disable=broad-except
            self.ax_lidar.text2D(
                0.5,
                0.5,
                f"Failed to load LiDAR:\n{error}",
                transform=self.ax_lidar.transAxes,
                ha="center",
                va="center",
                color="white",
            )
            self.ax_lidar.set_title(f"LiDAR: {record.lidar_file.name}")
            self.apply_lidar_view(lidar_view)
            return

        if len(points) == 0:
            self.ax_lidar.text2D(
                0.5,
                0.5,
                "No points",
                transform=self.ax_lidar.transAxes,
                ha="center",
                va="center",
                color="white",
            )
            self.ax_lidar.set_title(f"LiDAR: {record.lidar_file.name}")
            self.apply_lidar_view(lidar_view)
            return

        x_vals = points[:, 0]
        y_vals = points[:, 1]
        z_vals = points[:, 2]
        intensity_vals = points[:, 3]
        color_values = intensity_vals if np.ptp(intensity_vals) > 1e-6 else z_vals

        self.ax_lidar.scatter(
            x_vals,
            y_vals,
            z_vals,
            c=color_values,
            cmap="turbo",
            depthshade=False,
            alpha=0.95,
            s=self.lidar_point_size,
            linewidths=0.0,
        )
        self.ax_lidar.set_xlabel("X (m)")
        self.ax_lidar.set_ylabel("Y (m)")
        self.ax_lidar.set_zlabel("Z (m)")
        self.ax_lidar.grid(True, alpha=0.25)
        self.ax_lidar.set_title(f"LiDAR 3D View: {record.lidar_file.name}")
        self.set_equal_3d_limits(x_vals, y_vals, z_vals)
        self.apply_lidar_view(lidar_view)

    def draw_camera(self, axis: plt.Axes, image_path: Path, camera_name: str) -> None:
        try:
            image_rgb = load_image_cached(str(image_path))
            axis.imshow(image_rgb)
            axis.set_axis_off()
            axis.set_title(f"{camera_name}: {image_path.name}")
        except Exception as error:  # pylint: disable=broad-except
            axis.text(0.5, 0.5, f"Failed to load image:\n{error}", ha="center", va="center")
            axis.set_axis_off()
            axis.set_title(f"{camera_name}: {image_path.name}")


def main() -> int:
    args = parse_args()
    session_dir = Path(args.session_dir).expanduser().resolve()
    if not session_dir.exists():
        print(f"Session directory not found: {session_dir}", file=sys.stderr)
        return 1

    metadata_path = (
        Path(args.metadata_csv).expanduser().resolve()
        if args.metadata_csv
        else (session_dir / "metadata" / "frame_metadata.csv")
    )
    try:
        camera_a, camera_b = detect_camera_names(
            session_dir=session_dir,
            metadata_path=metadata_path,
            cam_a_arg=args.camera_a.strip(),
            cam_b_arg=args.camera_b.strip(),
        )
    except ValueError as error:
        print(f"Camera detection error: {error}", file=sys.stderr)
        return 1

    records = load_records(
        session_dir=session_dir,
        metadata_path=metadata_path,
        camera_a=camera_a,
        camera_b=camera_b,
    )
    if not records:
        print(
            "No synchronized frames found. Check that LiDAR and stereo files exist for matching frame indices.",
            file=sys.stderr,
        )
        return 1

    viewer = SyncViewer(
        records=records,
        camera_a=camera_a,
        camera_b=camera_b,
        max_lidar_points=max(0, int(args.max_lidar_points)),
        lidar_point_size=max(0.1, float(args.lidar_point_size)),
    )
    print(
        f"Loaded {len(records)} frames using cameras '{camera_a}' and '{camera_b}'. "
        "Use keyboard arrows in the viewer window."
    )
    plt.show()
    del viewer
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
