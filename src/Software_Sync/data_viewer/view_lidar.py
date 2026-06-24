#!/usr/bin/env python3
"""Standalone LiDAR PCD frame viewer.

Usage:
    python3 view_lidar.py --session-dir captures/session_YYYYMMDD_HHMMSS
    python3 view_lidar.py --pcd-dir /path/to/folder/of/pcds

Keyboard controls:
    Right / D       next frame
    Left  / A       previous frame
    Up    / W       +10 frames
    Down  / S       -10 frames
    Home            first frame
    End             last frame
    Q / Escape      quit
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View LiDAR PCD frames from a capture session.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--session-dir",
        help="Path to a capture session folder (e.g. captures/session_YYYYMMDD_HHMMSS). "
             "Looks for PCD files in <session>/lidar/frames/",
    )
    group.add_argument(
        "--pcd-dir",
        help="Path to any folder containing .pcd files directly",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=50000,
        help="Maximum points rendered per frame — reduce if slow (default: 50000)",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=0.8,
        help="Scatter marker size (default: 0.8)",
    )
    return parser.parse_args()


def find_pcd_files(pcd_dir: Path) -> list[Path]:
    files = sorted(pcd_dir.glob("*.pcd"))
    if not files:
        sys.exit(f"No .pcd files found in: {pcd_dir}")
    return files


def read_pcd(path: Path) -> np.ndarray:
    header_lines = 0
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for i, line in enumerate(f, start=1):
            stripped = line.strip()
            if stripped.upper().startswith("DATA"):
                data_mode = stripped.split(maxsplit=1)[-1].lower()
                if data_mode != "ascii":
                    raise ValueError(f"Only ASCII PCD supported, got '{data_mode}': {path.name}")
                header_lines = i
                break

    if header_lines == 0:
        raise ValueError(f"No DATA line found in {path.name}")

    points = np.loadtxt(str(path), dtype=np.float32, skiprows=header_lines)
    if points.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    if points.ndim == 1:
        points = points[np.newaxis, :]
    if points.shape[1] < 3:
        raise ValueError(f"Expected at least 3 columns (x y z) in {path.name}")
    if points.shape[1] == 3:
        points = np.hstack([points, np.zeros((len(points), 1), dtype=np.float32)])
    return points[:, :4]


def subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points > 0 and len(points) > max_points:
        idx = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        return points[idx]
    return points


class LidarViewer:
    def __init__(self, files: list[Path], max_points: int, point_size: float) -> None:
        self.files = files
        self.max_points = max_points
        self.point_size = point_size
        self.index = 0
        self._view: dict | None = None

        self.fig = plt.figure("LiDAR PCD Viewer", figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._draw()

    def _on_key(self, event: object) -> None:
        key = getattr(event, "key", None)
        if key in ("right", "d"):
            self._go(1)
        elif key in ("left", "a"):
            self._go(-1)
        elif key in ("up", "w"):
            self._go(10)
        elif key in ("down", "s"):
            self._go(-10)
        elif key == "home":
            self._go(-len(self.files))
        elif key == "end":
            self._go(len(self.files))
        elif key in ("q", "escape"):
            plt.close(self.fig)

    def _go(self, offset: int) -> None:
        self.index = max(0, min(self.index + offset, len(self.files) - 1))
        self._draw()

    def _save_view(self) -> None:
        if self.ax.collections:
            self._view = {
                "elev": self.ax.elev,
                "azim": self.ax.azim,
                "xlim": self.ax.get_xlim3d(),
                "ylim": self.ax.get_ylim3d(),
                "zlim": self.ax.get_zlim3d(),
            }

    def _restore_view(self) -> None:
        if self._view is None:
            self.ax.view_init(elev=25.0, azim=-60.0)
            return
        self.ax.view_init(elev=self._view["elev"], azim=self._view["azim"])
        self.ax.set_xlim3d(*self._view["xlim"])
        self.ax.set_ylim3d(*self._view["ylim"])
        self.ax.set_zlim3d(*self._view["zlim"])

    def _set_equal_limits(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> None:
        half = max((x.ptp(), y.ptp(), z.ptp(), 1.0)) / 2.0
        self.ax.set_xlim3d(x.mean() - half, x.mean() + half)
        self.ax.set_ylim3d(y.mean() - half, y.mean() + half)
        self.ax.set_zlim3d(z.mean() - half, z.mean() + half)

    def _draw(self) -> None:
        self._save_view()
        self.ax.clear()

        self.ax.set_facecolor("black")
        for pane in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            pane.set_pane_color((0.05, 0.05, 0.05, 1.0))
            pane.label.set_color("white")
            pane._axinfo["grid"]["color"] = (0.8, 0.8, 0.8, 0.2)
        self.ax.tick_params(colors="white")

        path = self.files[self.index]
        n_total = len(self.files)

        try:
            points = read_pcd(path)
        except Exception as err:
            self.ax.text2D(0.5, 0.5, f"Error loading file:\n{err}",
                           transform=self.ax.transAxes, ha="center", va="center", color="red")
            self.fig.suptitle(f"Frame {self.index + 1}/{n_total} — {path.name}", color="white")
            self.fig.canvas.draw_idle()
            return

        if len(points) == 0:
            self.ax.text2D(0.5, 0.5, "Empty frame (0 points)",
                           transform=self.ax.transAxes, ha="center", va="center", color="yellow")
            self._restore_view()
            self.fig.suptitle(f"Frame {self.index + 1}/{n_total} — {path.name}  |  0 pts", color="white")
            self.fig.canvas.draw_idle()
            return

        pts = subsample(points, self.max_points)
        x, y, z, intensity = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
        colour = intensity if np.ptp(intensity) > 1e-6 else z

        self.ax.scatter(x, y, z, c=colour, cmap="turbo",
                        s=self.point_size, depthshade=False, alpha=0.95, linewidths=0.0)
        self.ax.set_xlabel("X (m)", color="white")
        self.ax.set_ylabel("Y (m)", color="white")
        self.ax.set_zlabel("Z (m)", color="white")
        self._set_equal_limits(x, y, z)
        self._restore_view()

        n_shown = len(pts)
        n_raw = len(points)
        pts_label = f"{n_shown:,} pts" if n_shown == n_raw else f"{n_shown:,}/{n_raw:,} pts"
        self.fig.suptitle(
            f"Frame {self.index + 1}/{n_total} — {path.name}  |  {pts_label}\n"
            "← → prev/next   ↑ ↓ ±10   Home/End   Q quit   drag=rotate   scroll=zoom",
            color="white",
            fontsize=9,
        )
        self.fig.set_facecolor("#1a1a1a")
        self.fig.canvas.draw_idle()


def main() -> int:
    args = parse_args()

    if args.session_dir:
        session_dir = Path(args.session_dir).expanduser().resolve()
        pcd_dir = session_dir / "lidar" / "frames"
        if not pcd_dir.exists():
            sys.exit(f"LiDAR frames folder not found: {pcd_dir}")
    else:
        pcd_dir = Path(args.pcd_dir).expanduser().resolve()
        if not pcd_dir.exists():
            sys.exit(f"Directory not found: {pcd_dir}")

    files = find_pcd_files(pcd_dir)
    print(f"Found {len(files)} PCD frames in {pcd_dir}")

    viewer = LidarViewer(
        files=files,
        max_points=max(1, args.max_points),
        point_size=max(0.1, args.point_size),
    )
    plt.show()
    del viewer
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
