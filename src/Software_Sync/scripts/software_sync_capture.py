#!/usr/bin/env python3
"""Software-sync LiDAR + stereo capture orchestrator."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2

from tis_camera import TisCamera

LIDAR_FRAME_RE = re.compile(r"Finish save (\d+) frame to (pcd|lvx) file\.", re.IGNORECASE)
LIDAR_PCD_INDEX_RE = re.compile(r"_frame_(\d+)\.pcd$")
SUPPORTED_LIDAR_FORMATS = ("pcd", "lvx")
DEFAULT_LIDAR_FRAME_RATE = 20.0


@dataclass
class SessionPaths:
    root: Path
    lidar_raw: Path
    lidar_frames: Path
    lidar_lvx: Path
    stereo_root: Path
    metadata_dir: Path
    logs_dir: Path


@dataclass
class CameraEntry:
    serial: str
    image_prefix: str
    trigger_config: dict[str, Any]
    directory: Path
    camera: TisCamera


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    software_sync_root = script_dir.parent

    parser = argparse.ArgumentParser(
        description="Capture software-synchronized LiDAR frames and stereo images."
    )
    parser.add_argument("--frames", type=int, required=True, help="Number of synchronized frame pairs to capture")
    parser.add_argument(
        "--config",
        default=str(software_sync_root / "config" / "stereo_software_trigger.json"),
        help="Path to stereo camera JSON config",
    )
    parser.add_argument(
        "--output-root",
        default=str(software_sync_root / "captures"),
        help="Root folder where per-session capture folders are created",
    )
    parser.add_argument(
        "--lidar-binary",
        default="",
        help="Path to lidar_lvx_sample binary (defaults to auto-detection)",
    )
    parser.add_argument(
        "--lidar-format",
        choices=SUPPORTED_LIDAR_FORMATS,
        default="pcd",
        help="LiDAR output format passed to lidar_lvx_sample (pcd or lvx)",
    )
    parser.add_argument(
        "--lidar-frame-rate",
        type=float,
        default=DEFAULT_LIDAR_FRAME_RATE,
        help="Expected LiDAR frame rate used for runtime estimate (default: 20)",
    )
    parser.add_argument(
        "--lidar-time-margin-sec",
        type=int,
        default=2,
        help="Extra seconds added to estimated LiDAR runtime",
    )
    parser.add_argument(
        "--camera-timeout-sec",
        type=float,
        default=0.35,
        help="Timeout per camera when waiting for a software-triggered frame",
    )
    parser.add_argument(
        "--lidar-code",
        default="",
        help="Optional Livox broadcast code list for -c/--code",
    )
    parser.add_argument(
        "--lidar-param",
        action="store_true",
        help="Pass --param to lidar_lvx_sample",
    )
    parser.add_argument(
        "--lidar-log",
        action="store_true",
        help="Pass --log to lidar_lvx_sample",
    )
    parser.add_argument(
        "--show-video",
        action="store_true",
        help="Show live ximagesink previews for each camera",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as infile:
        return json.load(infile)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def create_session_paths(output_root: Path) -> SessionPaths:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    session_root = output_root / f"session_{timestamp}"

    lidar_raw = session_root / "lidar" / "raw"
    lidar_frames = session_root / "lidar" / "frames"
    lidar_lvx = session_root / "lidar" / "lvx"
    stereo_root = session_root / "stereo"
    metadata_dir = session_root / "metadata"
    logs_dir = session_root / "logs"

    for path in [lidar_raw, lidar_frames, lidar_lvx, stereo_root, metadata_dir, logs_dir]:
        path.mkdir(parents=True, exist_ok=True)

    return SessionPaths(
        root=session_root,
        lidar_raw=lidar_raw,
        lidar_frames=lidar_frames,
        lidar_lvx=lidar_lvx,
        stereo_root=stereo_root,
        metadata_dir=metadata_dir,
        logs_dir=logs_dir,
    )


def detect_lidar_binary(provided_path: str, repo_root: Path) -> Path:
    if provided_path:
        lidar_path = Path(provided_path).expanduser().resolve()
        if not lidar_path.exists():
            raise FileNotFoundError(f"LiDAR binary does not exist: {lidar_path}")
        if not os.access(lidar_path, os.X_OK):
            raise PermissionError(f"LiDAR binary is not executable: {lidar_path}")
        return lidar_path

    candidates = [
        repo_root / "src" / "Software_Sync" / "lidar" / "bin" / "lidar_lvx_sample",
        Path("/home/daniel/Livox-SDK/build/sample/lidar_lvx_file/lidar_lvx_sample"),
    ]
    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate

    raise FileNotFoundError(
        "Could not auto-detect lidar_lvx_sample. Pass --lidar-binary /path/to/lidar_lvx_sample."
    )


def validate_lidar_binary_supports_format(lidar_binary: Path, lidar_format: str) -> None:
    try:
        result = subprocess.run(
            [str(lidar_binary), "--help"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5,
            check=False,
        )
    except Exception as error:  # pylint: disable=broad-except
        raise RuntimeError(f"Unable to check LiDAR binary help output: {error}") from error

    help_text = (result.stdout or "").lower()
    if "format" not in help_text or lidar_format.lower() not in help_text:
        raise RuntimeError(
            f"Selected LiDAR binary does not advertise '--format {lidar_format}' support. "
            "Use the Software_Sync copy at src/Software_Sync/lidar/bin/lidar_lvx_sample "
            "or pass a compatible binary with --lidar-binary."
        )


def merge_dicts(base: dict[str, Any], override: dict[str, Any] | None) -> dict[str, Any]:
    merged = dict(base)
    if override:
        merged.update(override)
    return merged


def init_cameras(
    config: dict[str, Any],
    session_paths: SessionPaths,
    show_video: bool,
) -> list[CameraEntry]:
    camera_cfgs = config.get("cameras", [])
    if len(camera_cfgs) < 2:
        raise ValueError("Config must include at least two cameras for stereo capture")

    global_trigger = config.get("trigger", {})
    entries: list[CameraEntry] = []

    for camera_cfg in camera_cfgs:
        serial = camera_cfg.get("serial")
        image_prefix = camera_cfg.get("imageprefix")
        if not serial or not image_prefix:
            raise ValueError("Each camera entry must include 'serial' and 'imageprefix'")

        camera_output_dir = session_paths.stereo_root / image_prefix
        camera_output_dir.mkdir(parents=True, exist_ok=True)

        cam = TisCamera(
            serial=serial,
            width=int(camera_cfg["width"]),
            height=int(camera_cfg["height"]),
            framerate=str(camera_cfg["framerate"]),
            pixel_format=str(camera_cfg["pixelformat"]),
            image_prefix=image_prefix,
            show_video=show_video,
        )
        cam.start()
        cam.apply_properties(camera_cfg.get("properties", []))

        trigger_cfg = merge_dicts(global_trigger, camera_cfg.get("trigger"))
        cam.configure_software_trigger(trigger_cfg)

        entries.append(
            CameraEntry(
                serial=serial,
                image_prefix=image_prefix,
                trigger_config=trigger_cfg,
                directory=camera_output_dir,
                camera=cam,
            )
        )

    return entries


def stop_cameras(entries: list[CameraEntry]) -> None:
    for entry in entries:
        try:
            entry.camera.set_trigger_mode(False, entry.trigger_config)
        except Exception as error:  # pylint: disable=broad-except
            print(f"[WARN] Failed to disable trigger mode for {entry.image_prefix}: {error}")
        try:
            entry.camera.stop()
        except Exception as error:  # pylint: disable=broad-except
            print(f"[WARN] Failed to stop camera {entry.image_prefix}: {error}")


def build_lidar_command(
    lidar_binary: Path,
    frames: int,
    lidar_frame_rate: float,
    lidar_time_margin_sec: int,
    lidar_format: str,
    lidar_code: str,
    lidar_log: bool,
    lidar_param: bool,
) -> list[str]:
    estimated_seconds = max(1, int(math.ceil(frames / lidar_frame_rate)) + int(lidar_time_margin_sec))

    command = [
        str(lidar_binary),
        "--time",
        str(estimated_seconds),
        "--format",
        lidar_format,
    ]
    if lidar_code:
        command.extend(["--code", lidar_code])
    if lidar_log:
        command.append("--log")
    if lidar_param:
        command.append("--param")
    return command


def terminate_process(proc: subprocess.Popen[str], timeout_sec: float = 3.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout_sec)


def capture_stereo_pair(
    camera_entries: list[CameraEntry],
    frame_number: int,
    camera_timeout_sec: float,
) -> dict[str, Any]:
    trigger_time_ns = time.time_ns()
    for entry in camera_entries:
        command_property = entry.trigger_config.get("command_property", "TriggerSoftware")
        entry.camera.trigger(command_property)

    record: dict[str, Any] = {
        "frame_index": frame_number,
        "trigger_time_ns": trigger_time_ns,
    }
    for entry in camera_entries:
        image, capture_time_ns = entry.camera.snap_image(camera_timeout_sec)
        image_filename = f"{entry.image_prefix}_{frame_number:06d}_{capture_time_ns}.png"
        image_path = entry.directory / image_filename

        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f"Failed to save image for {entry.image_prefix}: {image_path}")

        record[f"{entry.image_prefix}_serial"] = entry.serial
        record[f"{entry.image_prefix}_capture_time_ns"] = capture_time_ns
        record[f"{entry.image_prefix}_file"] = str(image_path.relative_to(entry.directory.parents[1]))

    return record


def write_csv(records: list[dict[str, Any]], path: Path) -> None:
    if not records:
        return

    fieldnames: list[str] = []
    for row in records:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in records:
            writer.writerow(row)


def map_pcd_files_to_records(session_paths: SessionPaths, records: list[dict[str, Any]]) -> None:
    file_by_lidar_raw_index: dict[int, Path] = {}
    for pcd_file in sorted(session_paths.lidar_raw.glob("*.pcd")):
        match = LIDAR_PCD_INDEX_RE.search(pcd_file.name)
        if not match:
            continue
        file_by_lidar_raw_index[int(match.group(1))] = pcd_file

    for record in records:
        lidar_raw_index = record.get("lidar_frame_raw_index")
        if lidar_raw_index is None:
            record["lidar_file"] = ""
            continue

        source_file = file_by_lidar_raw_index.get(int(lidar_raw_index))
        if source_file is None or not source_file.exists():
            record["lidar_file"] = ""
            continue

        target_name = f"lidar_{int(record['frame_index']):06d}.pcd"
        target_file = session_paths.lidar_frames / target_name
        shutil.move(str(source_file), target_file)
        record["lidar_file"] = str(target_file.relative_to(session_paths.root))


def collect_lvx_files(session_paths: SessionPaths) -> list[str]:
    moved_files: list[str] = []
    for index, lvx_file in enumerate(sorted(session_paths.lidar_raw.glob("*.lvx")), start=1):
        target_name = "lidar_capture.lvx" if index == 1 else f"lidar_capture_{index:02d}.lvx"
        target_file = session_paths.lidar_lvx / target_name
        shutil.move(str(lvx_file), target_file)
        moved_files.append(str(target_file.relative_to(session_paths.root)))
    return moved_files


def main() -> int:
    args = parse_args()
    if args.frames <= 0:
        print("--frames must be > 0", file=sys.stderr)
        return 1

    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[2]
    config_path = Path(args.config).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()

    if not config_path.exists():
        print(f"Config not found: {config_path}", file=sys.stderr)
        return 1

    config = load_json(config_path)
    session_paths = create_session_paths(output_root)
    print(f"Session directory: {session_paths.root}")

    metadata_csv_path = session_paths.metadata_dir / "frame_metadata.csv"
    summary_json_path = session_paths.metadata_dir / "session_summary.json"
    lidar_stdout_log_path = session_paths.logs_dir / "lidar_stdout.log"

    started_utc = utc_now_iso()
    camera_entries: list[CameraEntry] = []
    lidar_proc: subprocess.Popen[str] | None = None
    records: list[dict[str, Any]] = []
    status = "failed"
    error_message = ""
    lidar_binary: Path | None = None
    lidar_command: list[str] = []
    lvx_files: list[str] = []

    try:
        lidar_binary = detect_lidar_binary(args.lidar_binary, repo_root)
        validate_lidar_binary_supports_format(lidar_binary, args.lidar_format)
        lidar_command = build_lidar_command(
            lidar_binary=lidar_binary,
            frames=args.frames,
            lidar_frame_rate=args.lidar_frame_rate,
            lidar_time_margin_sec=args.lidar_time_margin_sec,
            lidar_format=args.lidar_format,
            lidar_code=args.lidar_code,
            lidar_log=args.lidar_log,
            lidar_param=args.lidar_param,
        )

        camera_entries = init_cameras(config, session_paths, show_video=args.show_video)
        print("Stereo cameras initialized in software trigger mode.")

        print(f"Starting LiDAR capture: {' '.join(lidar_command)}")
        lidar_proc = subprocess.Popen(  # pylint: disable=consider-using-with
            lidar_command,
            cwd=session_paths.lidar_raw,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        captured_frames = 0
        with lidar_stdout_log_path.open("w", encoding="utf-8") as log_file:
            while True:
                line = lidar_proc.stdout.readline() if lidar_proc.stdout else ""
                if line == "":
                    if lidar_proc.poll() is not None:
                        break
                    continue

                log_file.write(line)
                log_file.flush()
                print(line, end="")

                match = LIDAR_FRAME_RE.search(line)
                if not match:
                    continue

                lidar_raw_index = int(match.group(1))
                lidar_line_format = match.group(2).lower()
                if lidar_line_format != args.lidar_format.lower():
                    continue
                captured_frames += 1
                lidar_event_time_ns = time.time_ns()

                frame_record = capture_stereo_pair(
                    camera_entries=camera_entries,
                    frame_number=captured_frames,
                    camera_timeout_sec=args.camera_timeout_sec,
                )
                frame_record["lidar_frame_raw_index"] = lidar_raw_index
                frame_record["lidar_event_time_ns"] = lidar_event_time_ns
                records.append(frame_record)

                print(f"Captured synchronized frame {captured_frames}/{args.frames}")

                if captured_frames >= args.frames:
                    terminate_process(lidar_proc)
                    break

        if lidar_proc.poll() is None:
            terminate_process(lidar_proc)

        if len(records) != args.frames:
            raise RuntimeError(
                f"Requested {args.frames} frames but captured {len(records)} synchronized frames"
            )

        if args.lidar_format == "pcd":
            map_pcd_files_to_records(session_paths, records)
        else:
            lvx_files = collect_lvx_files(session_paths)
            lidar_file_value = lvx_files[0] if lvx_files else ""
            for record in records:
                record["lidar_file"] = lidar_file_value

        write_csv(records, metadata_csv_path)
        status = "ok"

    except Exception as error:  # pylint: disable=broad-except
        error_message = str(error)
        print(f"ERROR: {error_message}", file=sys.stderr)

    finally:
        if lidar_proc is not None and lidar_proc.poll() is None:
            terminate_process(lidar_proc)
        stop_cameras(camera_entries)

        ended_utc = utc_now_iso()
        summary = {
            "status": status,
            "error": error_message,
            "started_utc": started_utc,
            "ended_utc": ended_utc,
            "frames_requested": args.frames,
            "frames_captured": len(records),
            "session_dir": str(session_paths.root),
            "config_path": str(config_path),
            "lidar_binary": str(lidar_binary) if lidar_binary else "",
            "lidar_format": args.lidar_format,
            "lidar_lvx_files": lvx_files,
            "lidar_command": lidar_command,
            "metadata_csv": str(metadata_csv_path),
            "lidar_stdout_log": str(lidar_stdout_log_path),
        }
        with summary_json_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2)

        if status == "ok":
            print("Capture complete.")
            print(f"Metadata CSV: {metadata_csv_path}")
            print(f"Session summary: {summary_json_path}")
            return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
