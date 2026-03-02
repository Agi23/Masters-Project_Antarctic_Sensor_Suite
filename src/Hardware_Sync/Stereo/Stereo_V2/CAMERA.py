import re

import TIS


class CAMERA(TIS.TIS):
    """
    Camera helper around TIS.TIS with property + hardware-trigger helpers.
    """

    def __init__(self, properties, imageprefix):
        super().__init__()
        self.properties = properties or []
        self.imageprefix = imageprefix

    @staticmethod
    def _normalize_value(value):
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
    def _normalize_enum_token(value):
        return re.sub(r"[^a-z0-9]", "", str(value).lower())

    def _try_set_enum_by_alias(self, property_name, alias_value):
        if not isinstance(alias_value, str):
            return False

        try:
            baseproperty = self.source.get_tcam_property(property_name)
        except Exception:  # pylint: disable=broad-except
            return False

        if not hasattr(baseproperty, "get_enum_entries"):
            return False

        try:
            entries = list(baseproperty.get_enum_entries())
        except Exception:  # pylint: disable=broad-except
            return False

        wanted = self._normalize_enum_token(alias_value)
        for entry in entries:
            if self._normalize_enum_token(entry) == wanted:
                try:
                    baseproperty.set_value(entry)
                    return True
                except Exception:  # pylint: disable=broad-except
                    return False
        return False

    def _set_property_safe(self, property_name, value, required=False):
        try:
            self.set_property(property_name, self._normalize_value(value))
            return True
        except Exception as error:  # pylint: disable=broad-except
            if self._try_set_enum_by_alias(property_name, value):
                return True
            msg = f"[{self.imageprefix}] Failed to set '{property_name}' to '{value}': {error}"
            if required:
                raise RuntimeError(msg) from error
            print(msg)
            return False

    def apply_properties(self):
        """
        Apply static camera properties from config in order.
        """
        for prop in self.properties:
            name = prop.get("property")
            if not name:
                continue
            self._set_property_safe(name, prop.get("value"), required=False)

    def _set_with_fallback_values(self, property_name, values, required=False):
        if not property_name:
            return True

        if values is None:
            return True

        if isinstance(values, list):
            for candidate in values:
                if self._set_property_safe(property_name, candidate, required=False):
                    return True
            if required:
                raise RuntimeError(
                    f"[{self.imageprefix}] None of fallback values worked for '{property_name}': {values}"
                )
            return False

        return self._set_property_safe(property_name, values, required=required)

    def set_trigger_mode(self, enabled, trigger_config):
        trigger_config = trigger_config or {}
        mode_property = trigger_config.get("mode_property", "TriggerMode")
        mode_on = trigger_config.get("mode_on", "On")
        mode_off = trigger_config.get("mode_off", "Off")
        mode_value = mode_on if enabled else mode_off
        return self._set_property_safe(mode_property, mode_value, required=enabled)

    def configure_hardware_trigger(self, trigger_config):
        """
        Configure external hardware trigger and enable TriggerMode.

        Expected keys in trigger_config:
        - selector_property, selector
        - source_property, source
        - activation_property, activation
        - delay_property, delay
        - mode_property, mode_on, mode_off
        """
        trigger_config = trigger_config or {}

        self._set_with_fallback_values(
            trigger_config.get("selector_property", "TriggerSelector"),
            trigger_config.get("selector", "Frame Start"),
            required=False,
        )

        self._set_with_fallback_values(
            trigger_config.get("source_property", "TriggerSource"),
            trigger_config.get("source", ["Line0", "Line1"]),
            required=False,
        )

        self._set_with_fallback_values(
            trigger_config.get("activation_property", "TriggerActivation"),
            trigger_config.get("activation", ["Rising Edge", "Level High"]),
            required=False,
        )

        delay_property = trigger_config.get("delay_property")
        if delay_property and "delay" in trigger_config:
            self._set_property_safe(delay_property, trigger_config.get("delay"), required=False)

        self.set_trigger_mode(True, trigger_config)
