#!/usr/bin/env python3
"""
Stereo capture using The Imaging Source cameras in hardware trigger mode.

Run:
    python3 stereo_hw_trigger_capture.py --config cameras_stereo_hw.json
"""

import argparse
import csv
import ctypes
import json
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

import cv2
import gi

import CAMERA
import TIS


gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GObject", "2.0")
gi.require_version("GLib", "2.0")

from gi.repository import GLib, Gst


_STOP = False


class TcamMetaReader:
    def __init__(self):
        self._lib = None
        self._buffer_size = 640
        self._buffer = ctypes.create_string_buffer(self._buffer_size)

        try:
            self._lib = ctypes.CDLL("libtcamgststatistics.so")
            self._lib.tcam_statistics_get_structure.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
            self._lib.tcam_statistics_get_structure.restype = ctypes.c_bool
        except OSError:
            self._lib = None
            print("Warning: libtcamgststatistics.so not found. Metadata export disabled.")

    def read(self, gst_buffer):
        if self._lib is None:
            return {}

        meta = gst_buffer.get_meta("TcamStatisticsMetaApi")
        if not meta:
            return {}

        ret = self._lib.tcam_statistics_get_structure(hash(meta), self._buffer, self._buffer_size)
        if not ret:
            return {}

        structure_string = ctypes.string_at(self._buffer).decode("utf-8")
        parsed = Gst.Structure.from_string(structure_string)
        if not parsed or parsed[0] is None:
            return {}

        structure = parsed[0]
        metadata = {}

        def _save_field(field_id, value, user_data):
            del user_data
            name = GLib.quark_to_string(field_id)
            metadata[f"meta_{name}"] = value
            return True

        structure.foreach(_save_field, None)
        return metadata


class CaptureContext:
    def __init__(self, output_dir, save_metadata):
        self.output_dir = output_dir
        self.save_metadata = save_metadata
        self.lock = Lock()
        self.frame_counts = {}
        self.records = []
        self.meta_reader = TcamMetaReader()

    def next_frame_index(self, camera_name):
        with self.lock:
            current = self.frame_counts.get(camera_name, 0) + 1
            self.frame_counts[camera_name] = current
            return current

    def add_record(self, record):
        if not self.save_metadata:
            return
        with self.lock:
            self.records.append(record)


def _signal_handler(sig, frame):
    del sig, frame
    global _STOP
    _STOP = True


def _merge_trigger_config(global_config, camera_config):
    merged = dict(global_config or {})
    merged.update(camera_config or {})
    return merged


def _build_session_directory(base_output_dir):
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    session_dir = base_output_dir / f"capture_{timestamp}"
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


def _load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as infile:
        return json.load(infile)


def _write_metadata_csv(metadata_path, records):
    if not records:
        print("No metadata records to write.")
        return

    fieldnames = []
    for record in records:
        for key in record.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with open(metadata_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in records:
            writer.writerow(row)


def on_new_image(camera, userdata):
    context = userdata
    image = camera.get_image()
    if image is None:
        return

    frame_time_ns = time.time_ns()
    camera_name = camera.imageprefix
    frame_index = context.next_frame_index(camera_name)

    image_file = f"{camera_name}_{frame_index:06d}_{frame_time_ns}.png"
    image_path = context.output_dir / image_file

    if not cv2.imwrite(str(image_path), image):
        print(f"[{camera_name}] Failed to save image: {image_path}")
        return

    sample = camera.appsink.get_property("last-sample")
    metadata = {}
    if sample:
        gst_buffer = sample.get_buffer()
        metadata = context.meta_reader.read(gst_buffer)

    record = {
        "camera": camera_name,
        "serial": camera.serialnumber,
        "frame_index": frame_index,
        "unix_time_ns": frame_time_ns,
        "image_file": image_file,
    }
    record.update(metadata)
    context.add_record(record)

    if frame_index % 100 == 0:
        print(f"[{camera_name}] Captured {frame_index} frames")


def _validate_camera_config(camera_cfg):
    required = ["serial", "pixelformat", "width", "height", "framerate", "imageprefix"]
    missing = [field for field in required if field not in camera_cfg]
    if missing:
        raise ValueError(f"Camera config missing fields: {missing}")


def main():
    parser = argparse.ArgumentParser(description="Stereo hardware-trigger capture")
    parser.add_argument(
        "--config",
        default="cameras_stereo_hw.json",
        help="Path to JSON capture config",
    )
    parser.add_argument(
        "--output-dir",
        default="captures",
        help="Directory where image session folder will be created",
    )
    parser.add_argument(
        "--no-metadata",
        action="store_true",
        help="Disable metadata CSV generation",
    )
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file does not exist: {config_path}")

    config = _load_config(config_path)
    camera_configs = config.get("cameras", [])
    if len(camera_configs) < 2:
        raise ValueError("Config must include at least 2 cameras for stereo capture")

    for camera_cfg in camera_configs:
        _validate_camera_config(camera_cfg)

    base_output_dir = Path(args.output_dir).expanduser().resolve()
    session_dir = _build_session_directory(base_output_dir)
    print(f"Saving captures to: {session_dir}")

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    context = CaptureContext(session_dir, save_metadata=(not args.no_metadata))
    global_trigger = config.get("trigger", {})

    cameras = []

    try:
        for camera_cfg in camera_configs:
            camera = CAMERA.CAMERA(camera_cfg.get("properties", []), camera_cfg["imageprefix"])
            camera.open_device(
                camera_cfg["serial"],
                camera_cfg["width"],
                camera_cfg["height"],
                camera_cfg["framerate"],
                TIS.SinkFormats[camera_cfg["pixelformat"]],
                False,
            )
            camera.set_image_callback(on_new_image, context)
            cameras.append((camera, camera_cfg))

        for camera, camera_cfg in cameras:
            trigger_cfg = _merge_trigger_config(global_trigger, camera_cfg.get("trigger", {}))
            camera.set_trigger_mode(False, trigger_cfg)
            if not camera.start_pipeline():
                raise RuntimeError(f"Failed to start pipeline for camera {camera.imageprefix}")
            camera.apply_properties()
            camera.configure_hardware_trigger(trigger_cfg)

        print("Cameras armed. Waiting for Arduino hardware trigger pulses...")
        while not _STOP:
            time.sleep(0.1)

    finally:
        print("Stopping cameras...")
        for camera, camera_cfg in cameras:
            trigger_cfg = _merge_trigger_config(global_trigger, camera_cfg.get("trigger", {}))
            try:
                camera.set_trigger_mode(False, trigger_cfg)
            except Exception as error:  # pylint: disable=broad-except
                print(f"[{camera.imageprefix}] Failed to disable trigger mode: {error}")
            camera.stop_pipeline()

        if context.save_metadata:
            metadata_path = session_dir / "capture_metadata.csv"
            _write_metadata_csv(metadata_path, context.records)
            print(f"Metadata written to: {metadata_path}")

        print("Capture finished.")


if __name__ == "__main__":
    main()
