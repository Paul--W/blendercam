"""Fabex 'custom_settings_utils.py'

Persistence for Custom Settings — saves to JSON on every change,
restores on startup so settings survive without saving the .blend file.
"""

import json
from pathlib import Path

import bpy

from .logging_utils import log

_SETTINGS_FILE = Path(__file__).parent.parent / "_user_custom_settings.json"

# Guard flag — suppresses save + recalc during bulk loading
_loading = False

# All Custom Settings property names on CAM_MACHINE_Properties
_CUSTOM_SETTINGS_KEYS = [
    "use_job_start_height",
    "job_start_height",
    "use_rapids",
    "feedrate_rapid",
    "show_rest_machining",
    "default_safe_height",
    "use_g64_trajectory",
    "g64_tolerance",
    "stock_material",
    "machine_stiffness",
    "spindle_power_watts",
    "auto_calc_stepover",
    "auto_calc_layer_height",
    "auto_calc_rpm",
    "auto_calc_feedrate",
    "surface_finish_quality",
    "tool_material",
    "plunge_rate_factor",
    "spindle_min",
    "spindle_max",
    "spindle_default",
    "default_exact_mode",
    "default_use_opencamlib",
    "default_use_modifiers",
    "default_reduce_path_points",
    "default_path_threshold",
    "terrain_aware_rapids",
    "terrain_clearance",
    "verbose_logging",
]


def save_custom_settings(self, context):
    """Update callback — saves all Custom Settings to JSON."""
    if _loading:
        return

    import time as _time
    _t0 = _time.time()

    try:
        machine = context.scene.cam_machine
        data = {}
        for key in _CUSTOM_SETTINGS_KEYS:
            val = getattr(machine, key, None)
            if val is not None:
                if isinstance(val, float):
                    data[key] = val
                elif isinstance(val, bool):
                    data[key] = val
                elif isinstance(val, int):
                    data[key] = val
                elif isinstance(val, str):
                    data[key] = val
                else:
                    data[key] = str(val)
        _SETTINGS_FILE.write_text(json.dumps(data, indent=2))
    except Exception as e:
        log.debug(f"[Custom Settings] Save failed: {e}")

    _elapsed = _time.time() - _t0
    log.info(f"[Custom Settings] Saved in {_elapsed*1000:.1f}ms")
    # No auto-recalc here — only save_and_recalc triggers it.


def save_custom_settings_and_recalc(self, context):
    """Update callback for calc-related settings (material, stiffness, etc.).

    Saves to JSON AND triggers auto-recalculation on the active operation.
    Use this for properties that affect cutting parameters.
    """
    save_custom_settings(self, context)
    if _loading:
        return  # bulk load in progress — single calc happens after all properties are set
    try:
        machine = context.scene.cam_machine
        scene = context.scene
        if scene.cam_operations:
            op = scene.cam_operations[scene.cam_active_operation]
            from .cutting_params import auto_calculate_for_operation
            result = auto_calculate_for_operation(op, machine)
            if result is None:
                log.warning("[Custom Settings] auto_calculate returned None (reentrancy blocked?)")
            # Update rest machining layer height if rest machining is active
            if getattr(op, "use_rest_machining", False):
                from ..strategies.rest_machining import suggest_layer_height
                suggest_layer_height(op, machine)
        else:
            log.debug("[Custom Settings] No operations — skipping recalc")
    except Exception as e:
        log.warning(f"[Custom Settings] Recalc failed: {e}")


def load_custom_settings():
    """Restore Custom Settings from JSON into scene.cam_machine.

    Called from on_blender_startup after scene is available.
    """
    if not _SETTINGS_FILE.exists():
        return

    global _loading
    try:
        data = json.loads(_SETTINGS_FILE.read_text())
        machine = bpy.context.scene.cam_machine

        _loading = True
        for key, val in data.items():
            if key in _CUSTOM_SETTINGS_KEYS and hasattr(machine, key):
                try:
                    setattr(machine, key, val)
                except Exception:
                    pass  # skip incompatible values silently
        _loading = False

        log.debug(f"[Custom Settings] Loaded {len(data)} settings from {_SETTINGS_FILE.name}")

        # Single calc after all properties are set
        try:
            scene = bpy.context.scene
            if scene.cam_operations:
                op = scene.cam_operations[scene.cam_active_operation]
                from .cutting_params import auto_calculate_for_operation
                auto_calculate_for_operation(op, machine)
        except Exception:
            pass
    except Exception as e:
        _loading = False
        log.debug(f"[Custom Settings] Load failed: {e}")
