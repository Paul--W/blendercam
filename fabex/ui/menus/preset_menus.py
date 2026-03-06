"""Fabex 'preset_menus.py'

Operators and Menus for CAM Machine, Cutter and Operation Presets.
"""

import json
from pathlib import Path

import bpy
from bpy.types import Menu

from ... import __package__ as base_package

_OCL_PREFS_FILE = Path(__file__).parent.parent.parent / "_user_ocl_prefs.json"
_remembered = {"use_exact": True, "use_opencamlib": False}
try:
    if _OCL_PREFS_FILE.exists():
        _remembered.update(json.loads(_OCL_PREFS_FILE.read_text()))
except Exception:
    pass


class CAM_CUTTER_MT_presets(Menu):
    bl_label = "Cutter Presets"
    preset_subdir = "cam_cutters"
    preset_operator = "script.execute_preset"
    # draw = Menu.draw_preset

    name = bl_label
    filepath = bpy.utils.preset_find(name, preset_subdir, display_name=True, ext=".py")

    # @classmethod
    # def post_cb(cls, context, filepath):
    #     addon_prefs = context.preferences.addons["bl_ext.user_default.fabex"].preferences
    #     addon_prefs.default_machine_preset = filepath
    #     bpy.ops.wm.save_userpref()

    def draw(self, context):
        scene = context.scene

        layout = self.layout
        layout.prop_menu_enum(scene, "idcwoodcraft", text="IDC Woodcraft")
        layout.prop_menu_enum(scene, "cadence", text="Cadence")
        layout.prop_menu_enum(scene, "user_cutter", text="User")
        layout.operator("wm.call_menu", text="View Full List").name = "CAM_CUTTER_MT_presets"


class CAM_OPERATION_MT_presets(Menu):
    bl_label = "Operation Presets"
    preset_subdir = "cam_operations"
    preset_operator = "script.execute_preset"
    # draw = Menu.draw_preset

    name = bl_label
    filepath = bpy.utils.preset_find(name, preset_subdir, display_name=True, ext=".py")

    @classmethod
    def reset_cb(cls, context, filepath):
        """Called before preset is executed — snapshot current OCL/exact settings."""
        s = context.scene
        if s.cam_operations:
            op = s.cam_operations[s.cam_active_operation]
            _remembered["use_exact"] = op.optimisation.use_exact
            _remembered["use_opencamlib"] = op.optimisation.use_opencamlib

    @classmethod
    def post_cb(cls, context, filepath):
        """Called after preset is executed — restore the snapshotted settings."""
        s = context.scene
        if s.cam_operations:
            op = s.cam_operations[s.cam_active_operation]
            op.optimisation.use_exact = _remembered["use_exact"]
            op.optimisation.use_opencamlib = _remembered["use_opencamlib"]

    def draw(self, context):
        scene = context.scene

        layout = self.layout
        layout.prop_menu_enum(scene, "finishing", text="Finishing")
        layout.prop_menu_enum(scene, "roughing", text="Roughing")
        layout.prop_menu_enum(scene, "user_operation", text="User")
        layout.operator("wm.call_menu", text="View Full List").name = "CAM_OPERATION_MT_presets"


class CAM_MACHINE_MT_presets(Menu):
    bl_label = "Machine Presets"
    preset_subdir = "cam_machines"
    preset_operator = "script.execute_preset"

    name = bl_label
    filepath = bpy.utils.preset_find(name, preset_subdir, display_name=True, ext=".py")

    @classmethod
    def post_cb(cls, context, filepath):
        addon_prefs = context.preferences.addons[base_package].preferences
        addon_prefs.default_machine_preset = filepath
        bpy.ops.wm.save_userpref()

    def draw(self, context):
        scene = context.scene

        layout = self.layout
        layout.prop_menu_enum(scene, "avidcnc", text="AvidCNC")
        layout.prop_menu_enum(scene, "carbide3d", text="Carbide3D")
        layout.prop_menu_enum(scene, "cnc4all", text="CNC4ALL")
        layout.prop_menu_enum(scene, "inventables", text="Inventables")
        layout.prop_menu_enum(scene, "millright", text="MillRight")
        layout.prop_menu_enum(scene, "onefinity", text="Onefinity")
        layout.prop_menu_enum(scene, "ooznest", text="Ooznest")
        layout.prop_menu_enum(scene, "sienci", text="Sienci")
        layout.prop_menu_enum(scene, "user_machine", text="User")
        layout.operator("wm.call_menu", text="View Full List").name = "CAM_MACHINE_MT_presets"
