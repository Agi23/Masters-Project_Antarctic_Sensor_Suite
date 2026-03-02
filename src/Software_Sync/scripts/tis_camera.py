#!/usr/bin/env python3
"""Minimal The Imaging Source camera wrapper for software-trigger capture."""

from __future__ import annotations

import re
import time
from typing import Any, Sequence

import gi
import numpy as np

gi.require_version("Gst", "1.0")
gi.require_version("Tcam", "1.0")

from gi.repository import GLib, Gst, Tcam  # noqa: F401  pylint: disable=unused-import

_GST_INITIALIZED = False


def _ensure_gst_initialized() -> None:
    global _GST_INITIALIZED
    if _GST_INITIALIZED:
        return
    try:
        if not Gst.is_initialized():
            Gst.init(())
    except Exception:  # pylint: disable=broad-except
        Gst.init(())
    _GST_INITIALIZED = True


class TisCamera:
    """Camera controller with property + software trigger helpers."""

    def __init__(
        self,
        serial: str,
        width: int,
        height: int,
        framerate: str,
        pixel_format: str,
        image_prefix: str,
        show_video: bool = False,
    ) -> None:
        _ensure_gst_initialized()

        self.serial = serial
        self.width = int(width)
        self.height = int(height)
        self.framerate = str(framerate)
        self.pixel_format = str(pixel_format)
        self.image_prefix = image_prefix
        self.pipeline = None
        self.source = None
        self.appsink = None

        self._create_pipeline(show_video)
        self.source.set_property("serial", self.serial)
        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.get_state(5 * Gst.SECOND)

    def _create_pipeline(self, show_video: bool) -> None:
        pipeline_str = "tcambin name=source ! capsfilter name=caps"
        if show_video:
            pipeline_str += " ! tee name=t"
            pipeline_str += " t. ! queue ! videoconvert ! ximagesink"
            pipeline_str += " t. ! queue ! appsink name=sink"
        else:
            pipeline_str += " ! appsink name=sink"

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as error:
            raise RuntimeError(f"Failed to create pipeline for {self.image_prefix}: {error}") from error

        self.source = self.pipeline.get_by_name("source")
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("max-buffers", 5)
        self.appsink.set_property("drop", True)
        self.appsink.set_property("emit-signals", False)
        self.appsink.set_property("enable-last-sample", False)

    def _set_caps(self) -> None:
        caps = Gst.Caps.from_string(
            (
                f"video/x-raw,format={self.pixel_format},"
                f"width={self.width},height={self.height},framerate={self.framerate}"
            )
        )
        caps_filter = self.pipeline.get_by_name("caps")
        caps_filter.set_property("caps", caps)

    def start(self) -> None:
        self._set_caps()
        self.pipeline.set_state(Gst.State.PLAYING)
        state_change = self.pipeline.get_state(5 * Gst.SECOND)
        if state_change[1] != Gst.State.PLAYING:
            raise RuntimeError(f"[{self.image_prefix}] Failed to enter PLAYING state")

    def stop(self) -> None:
        if self.pipeline is None:
            return
        self.pipeline.set_state(Gst.State.NULL)

    @staticmethod
    def _normalize_value(value: Any) -> Any:
        if not isinstance(value, str):
            return value

        lowered = value.strip().lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
        if re.fullmatch(r"[-+]?\d+", value.strip()):
            return int(value.strip())
        if re.fullmatch(r"[-+]?\d*\.\d+", value.strip()):
            return float(value.strip())
        return value

    @staticmethod
    def _normalize_enum_token(value: Any) -> str:
        return re.sub(r"[^a-z0-9]", "", str(value).lower())

    def _set_property(self, property_name: str, value: Any) -> None:
        base_property = self.source.get_tcam_property(property_name)
        base_property.set_value(self._normalize_value(value))

    def _try_set_enum_alias(self, property_name: str, value: Any) -> bool:
        if not isinstance(value, str):
            return False
        try:
            base_property = self.source.get_tcam_property(property_name)
            if not hasattr(base_property, "get_enum_entries"):
                return False
            entries = list(base_property.get_enum_entries())
            wanted = self._normalize_enum_token(value)
            for entry in entries:
                if self._normalize_enum_token(entry) == wanted:
                    base_property.set_value(entry)
                    return True
        except Exception:  # pylint: disable=broad-except
            return False
        return False

    def set_property_with_fallback(self, property_name: str | None, values: Any, required: bool = False) -> bool:
        if not property_name or values is None:
            return True

        candidates = values if isinstance(values, list) else [values]
        for candidate in candidates:
            try:
                self._set_property(property_name, candidate)
                return True
            except Exception:  # pylint: disable=broad-except
                if self._try_set_enum_alias(property_name, candidate):
                    return True

        if required:
            raise RuntimeError(
                f"[{self.image_prefix}] Failed to set '{property_name}' with values {candidates}"
            )
        return False

    def apply_properties(self, properties: Sequence[dict[str, Any]] | None) -> None:
        for prop in properties or []:
            name = prop.get("property")
            if not name:
                continue
            self.set_property_with_fallback(name, prop.get("value"), required=False)

    def set_trigger_mode(self, enabled: bool, trigger_config: dict[str, Any]) -> None:
        mode_property = trigger_config.get("mode_property", "TriggerMode")
        mode_value = trigger_config.get("mode_on", "On") if enabled else trigger_config.get("mode_off", "Off")
        self.set_property_with_fallback(mode_property, mode_value, required=enabled)

    def configure_software_trigger(self, trigger_config: dict[str, Any]) -> None:
        self.set_property_with_fallback(
            trigger_config.get("selector_property", "TriggerSelector"),
            trigger_config.get("selector", ["Frame Start", "FrameStart"]),
            required=False,
        )
        self.set_property_with_fallback(
            trigger_config.get("source_property", "TriggerSource"),
            trigger_config.get("source", ["Software"]),
            required=False,
        )
        self.set_property_with_fallback(
            trigger_config.get("activation_property", "TriggerActivation"),
            trigger_config.get("activation", ["Rising Edge", "Level High"]),
            required=False,
        )

        delay_property = trigger_config.get("delay_property")
        if delay_property and "delay" in trigger_config:
            self.set_property_with_fallback(delay_property, trigger_config.get("delay"), required=False)

        self.set_trigger_mode(True, trigger_config)

    def trigger(self, command_property: str) -> None:
        base_property = self.source.get_tcam_property(command_property)
        base_property.set_command()

    @staticmethod
    def _convert_to_numpy(data: bytes, caps: Gst.Caps) -> np.ndarray:
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        fmt = structure.get_value("format")

        if fmt == "GRAY8":
            return np.ndarray((height, width), buffer=data, dtype=np.uint8).copy()
        if fmt == "GRAY16_LE":
            return np.ndarray((height, width), buffer=data, dtype=np.uint16).copy()
        if fmt in ("BGRx", "BGRA"):
            image = np.ndarray((height, width, 4), buffer=data, dtype=np.uint8).copy()
            return image[:, :, :3]
        if fmt == "BGR":
            return np.ndarray((height, width, 3), buffer=data, dtype=np.uint8).copy()

        raise RuntimeError(f"Unsupported pixel format from appsink: {fmt}")

    def snap_image(self, timeout_seconds: float) -> tuple[np.ndarray, int]:
        timeout_ns = int(timeout_seconds * Gst.SECOND)
        sample = self.appsink.emit("try-pull-sample", timeout_ns)
        if sample is None:
            raise TimeoutError(f"[{self.image_prefix}] Timed out waiting for triggered frame")

        buffer = sample.get_buffer()
        if buffer is None:
            raise RuntimeError(f"[{self.image_prefix}] No GstBuffer in sample")

        capture_time_ns = time.time_ns()
        data = buffer.extract_dup(0, buffer.get_size())
        image = self._convert_to_numpy(data, sample.get_caps())
        return image, capture_time_ns
