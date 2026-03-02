#!/usr/bin/env python3
"""
List trigger/line-related properties for a TIS camera.
Use this if your model needs different TriggerSource/Activation enum values.
"""

import argparse

import gi


gi.require_version("Tcam", "1.0")
gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")

from gi.repository import GLib, Gst, Tcam


def _matches_trigger_name(property_name):
    lowered = property_name.lower()
    return "trigger" in lowered or "line" in lowered


def _safe_call(func, default="<error>"):
    try:
        return func()
    except GLib.Error as err:
        return f"<error: {err.message}>"
    except Exception as err:  # pylint: disable=broad-except
        return f"<error: {err}>"


def main():
    parser = argparse.ArgumentParser(description="Inspect trigger-related camera properties")
    parser.add_argument("--serial", default=None, help="Camera serial; default camera is used if omitted")
    args = parser.parse_args()

    Gst.init(())

    camera = Gst.ElementFactory.make("tcambin")
    if args.serial:
        camera.set_property("serial", args.serial)

    camera.set_state(Gst.State.READY)

    try:
        names = camera.get_tcam_property_names()
        selected = sorted(name for name in names if _matches_trigger_name(name))

        if not selected:
            print("No trigger/line properties found.")
            return

        for name in selected:
            try:
                base = camera.get_tcam_property(name)
            except GLib.Error as err:
                print(f"{name}: <error: {err.message}>")
                continue

            ptype = base.get_property_type()
            print(f"\n{name}")
            print(f"  type: {ptype}")
            print(f"  available: {_safe_call(base.is_available)}  locked: {_safe_call(base.is_locked)}")

            if ptype == Tcam.PropertyType.ENUMERATION:
                print(f"  value: {_safe_call(base.get_value)}")
                entries = _safe_call(lambda: list(base.get_enum_entries()))
                print(f"  entries: {entries}")
            elif ptype == Tcam.PropertyType.BOOLEAN:
                print(f"  value: {_safe_call(base.get_value)}")
            elif ptype == Tcam.PropertyType.INTEGER:
                value = _safe_call(base.get_value)
                range_value = _safe_call(base.get_range)
                if isinstance(range_value, tuple) and len(range_value) == 3:
                    minimum, maximum, step = range_value
                    print(f"  value: {value}  range: [{minimum}, {maximum}] step={step}")
                else:
                    print(f"  value: {value}  range: {range_value}")
            elif ptype == Tcam.PropertyType.COMMAND:
                print("  value: <command>")
            else:
                print(f"  value: {_safe_call(base.get_value)}")
    finally:
        camera.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
