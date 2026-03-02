#!/usr/bin/env python3
"""
Quick recovery utility for TIS cameras.

Use this to return cameras to a safe live-view state after trigger experiments.
"""

import argparse

import gi


gi.require_version("Gst", "1.0")
gi.require_version("Tcam", "1.0")

from gi.repository import Gst, Tcam  # noqa: F401


def set_if_exists(camera, property_name, value):
    try:
        prop = camera.get_tcam_property(property_name)
        prop.set_value(value)
        print(f"  set {property_name} = {value}")
        return True
    except Exception as err:  # pylint: disable=broad-except
        print(f"  skip {property_name}: {err}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Recover camera from bad trigger/exposure state")
    parser.add_argument("--serial", action="append", required=True, help="Camera serial (repeat for multiple)")
    parser.add_argument("--manual", action="store_true", help="Use manual exposure/gain instead of auto")
    parser.add_argument("--exposure-us", type=float, default=4000.0, help="Manual exposure time in us")
    parser.add_argument("--gain-db", type=float, default=0.0, help="Manual gain in dB")
    args = parser.parse_args()

    Gst.init(())

    for serial in args.serial:
        print(f"{serial}:")
        camera = Gst.ElementFactory.make("tcambin")
        camera.set_property("serial", serial)
        camera.set_state(Gst.State.READY)

        try:
            set_if_exists(camera, "TriggerMode", "Off")

            if args.manual:
                set_if_exists(camera, "ExposureAuto", "Off")
                set_if_exists(camera, "GainAuto", "Off")
                set_if_exists(camera, "ExposureTime", args.exposure_us)
                set_if_exists(camera, "Gain", args.gain_db)
            else:
                set_if_exists(camera, "ExposureAuto", "Continuous")
                set_if_exists(camera, "GainAuto", "Continuous")

        finally:
            camera.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
