"""Fabex 'machine.py'

'CAM Machine' panel in Properties > Render
"""

import bpy
from bpy.types import Panel

from ...utilities.cutting_params import get_last_calc_display
from .parent_panel import CAMParentPanel


class CAM_MACHINE_Panel(CAMParentPanel, Panel):
    """CAM Machine Panel"""

    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "render"

    bl_label = "[ Machine ]"
    bl_idname = "FABEX_PT_CAM_MACHINE"
    panel_interface_level = 0
    always_show_panel = True

    def __init__(self, *args, **kwargs):
        Panel.__init__(self, *args, **kwargs)
        CAMParentPanel.__init__(self, *args, **kwargs)
        super().__init__(*args, **kwargs)

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        # Presets
        if self.level >= 1:
            row = layout.row(align=True)
            row.menu("CAM_MACHINE_MT_presets", text=bpy.types.CAM_MACHINE_MT_presets.bl_label)
            row.operator(
                "render.cam_preset_machine_add",
                text="",
                icon="ADD",
            )
            row.operator(
                "render.cam_preset_machine_add",
                text="",
                icon="REMOVE",
            ).remove_active = True

        box = layout.box()
        col = box.column(align=True)
        col.scale_y = 1.2
        # Post Processor
        if self.machine.post_processor == "USER":
            row = col.row(align=True)
            row.prop(self.machine, "post_processor")
            row.operator("fabex.edit_user_post_processor", text="Edit", icon="GREASEPENCIL")
        else:
            col.prop(self.machine, "post_processor")
        # Units
        row = col.row(align=True)
        row.prop(context.scene.cam_machine, "unit_system")

        # # Collet Size
        # if self.level >= 2:
        #     box.prop(self.machine, "collet_size")

        # Working Area
        box.prop(self.machine, "working_area")

        # Supplemental Axis
        if self.level >= 3:
            row = box.row(align=True)
            row.prop(self.machine, "axis_4")
            row.prop(self.machine, "axis_5")

        # Position Definitions
        if self.level >= 2:
            layout.use_property_split = False
            header, panel = layout.panel("position_definitions", default_closed=True)
            header.prop(self.machine, "use_position_definitions", text="Position Definitions")
            if panel:
                panel.enabled = self.machine.use_position_definitions
                col = panel.column()
                col.use_property_split = True
                col.prop(self.machine, "starting_position")
                col.prop(self.machine, "mtc_position")
                col.prop(self.machine, "ending_position")

        layout.use_property_split = True
        # Feedrates
        if self.level >= 1:
            f_unit = "in" if self.machine.unit_system == "INCHES" else "mm"
            header, panel = layout.panel(idname="feedrate", default_closed=True)
            header.label(text=f"Feedrate ({f_unit}/min)")
            if panel:
                panel.prop(self.machine, "feedrate_default", text="Default")
                col = panel.column(align=True)
                col.prop(self.machine, "feedrate_min", text="Minimum")
                col.prop(self.machine, "feedrate_max", text="Maximum")

        # Spindle Speeds
        # TODO: spindle default and feedrate default should become part of the cutter definition...
        header, panel = layout.panel(idname="spindle", default_closed=True)
        header.label(text="Spindle Speed (RPM)")
        if panel:
            panel.prop(self.machine, "spindle_default", text="Default")
            col = panel.column(align=True)
            col.prop(self.machine, "spindle_min", text="Minimum")
            col.prop(self.machine, "spindle_max", text="Maximum")
            panel.prop(self.machine, "spindle_start_time", text="Start Delay (seconds)")
            # Spindle Slow Start
            if self.level >= 2:
                subheader, subpanel = panel.panel(idname="slow_start", default_closed=True)
                subheader.label(text="Slow Start (Ramp-Up) (*EXPERIMENTAL, grbl only*)")
                if subpanel:
                    col = subpanel.column(align=True)
                    subpanel.use_property_split = True
                    col.prop(self.machine, "spindle_slow_start_enable", text="Slow Start")
                    subcol = col.column(align=True)
                    subcol.enabled = self.machine.spindle_slow_start_enable
                    subcol.prop(self.machine, "spindle_slow_start_steps", text="Steps")
                    subcol.prop(
                        self.machine,
                        "spindle_slow_start_skip_threshold",
                        text="Skip Small Increase (RPM)",
                    )
                    subcol.prop(
                        self.machine, "spindle_slow_start_total_time", text="Total Time (sec)"
                    )

        # Gcode Options
        if self.level >= 1:
            header, panel = layout.panel(idname="gcode", default_closed=True)
            header.label(text="Machine G-code")
            if panel:
                panel.use_property_split = False
                col = panel.column()
                # Tool Options
                if self.level >= 2:
                    col.prop(self.machine, "output_tool_definitions")
                    subheader, subpanel = col.panel(idname="tool_change", default_closed=False)
                    subheader.prop(self.machine, "output_tool_change")
                    if subpanel:
                        subpanel.enabled = self.machine.output_tool_change
                        subpanel.prop(self.machine, "output_G43_on_tool_change")

                # Block Numbers
                if self.level >= 2:
                    subheader, subpanel = col.panel(idname="block_numbers", default_closed=True)
                    subheader.prop(self.machine, "output_block_numbers")
                    if subpanel:
                        subpanel.enabled = self.machine.output_block_numbers
                        subpanel.use_property_split = True
                        column = subpanel.column(align=True)
                        column.prop(self.machine, "start_block_number")
                        column.prop(self.machine, "block_number_increment")

                # Split Files
                if self.level >= 2:
                    subheader, subpanel = col.panel(idname="split", default_closed=False)
                    subheader.prop(self.machine, "eval_splitting")
                    if subpanel:
                        subpanel.enabled = self.machine.eval_splitting
                        subpanel.use_property_split = True
                        subpanel.prop(self.machine, "split_limit")

            # Hourly Rate
            layout.prop(
                self.machine,
                "hourly_rate",
                text="Price ($/hour)",
            )

        # ── Custom Settings ──────────────────────────────────────────
        header, panel = layout.panel(idname="custom_settings", default_closed=True)
        header.label(text="Custom Settings")
        if panel:
            d_unit = "in" if self.machine.unit_system == "INCHES" else "mm"
            panel.use_property_split = False

            # Job Start Height
            col = panel.column(align=True)
            col.prop(self.machine, "use_job_start_height")
            subcol = col.column()
            subcol.use_property_split = True
            subcol.enabled = self.machine.use_job_start_height
            subcol.prop(self.machine, "job_start_height", text=f"Job Start Height ({d_unit})")

            # Default Safe Height
            panel.use_property_split = True
            col = panel.column(align=True)
            col.prop(self.machine, "default_safe_height", text=f"Default Safe Height ({d_unit})")

            # Machine Stiffness
            col = panel.column(align=True)
            col.prop(self.machine, "machine_stiffness")

            # Spindle Power
            col = panel.column(align=True)
            col.prop(self.machine, "spindle_power_watts")

            # Use Rapids
            col = panel.column(align=True)
            col.use_property_split = False
            col.prop(self.machine, "use_rapids")
            subcol = col.column()
            subcol.use_property_split = True
            subcol.enabled = self.machine.use_rapids
            f_unit = "in" if self.machine.unit_system == "INCHES" else "mm"
            subcol.prop(self.machine, "feedrate_rapid", text=f"Rapid ({f_unit}/min)")

            panel.separator()

            # Operation Defaults
            col = panel.column(align=True)
            col.use_property_split = False
            col.prop(self.machine, "default_exact_mode")
            col.prop(self.machine, "default_use_opencamlib")
            col.prop(self.machine, "default_use_modifiers")
            col.prop(self.machine, "default_reduce_path_points")
            subcol = col.column()
            subcol.use_property_split = True
            subcol.enabled = self.machine.default_reduce_path_points
            subcol.prop(self.machine, "default_path_threshold")

            # G64 Trajectory
            col = panel.column(align=True)
            col.use_property_split = False
            col.prop(self.machine, "use_g64_trajectory")
            subcol = col.column()
            subcol.use_property_split = True
            subcol.enabled = self.machine.use_g64_trajectory
            subcol.prop(self.machine, "g64_tolerance", text=f"G64 Tolerance ({d_unit})")

            panel.separator()

            # Path Control
            col = panel.column(align=True)
            col.use_property_split = False
            col.prop(self.machine, "show_rest_machining")
            col.prop(self.machine, "terrain_aware_rapids")
            if self.machine.terrain_aware_rapids:
                col.prop(self.machine, "terrain_clearance")
            col.prop(self.machine, "verbose_logging")

            panel.separator()

            # Auto-Calculate
            subheader, subpanel = panel.panel(
                idname="auto_calculate", default_closed=True
            )
            subheader.label(text="Auto-Calculate Feeds and Speeds")
            if subpanel:
                subpanel.use_property_split = True
                subpanel.prop(self.machine, "stock_material")
                subpanel.prop(self.machine, "tool_material")
                subpanel.prop(self.machine, "surface_finish_quality")
                subpanel.prop(self.machine, "plunge_rate_factor")

                subpanel.separator()

                # Refresh display cache only when the active operation
                # or machine settings change — NOT on every panel redraw.
                # The cache is updated by update callbacks (update_offset_image,
                # update_chipload, etc.) which call auto_calculate with
                # write_back=True.  The display just reads the cached result.

                # Auto-calc toggles with warning indicators
                calc_lines, calc_warnings = get_last_calc_display()

                col = subpanel.column(align=True)
                col.use_property_split = False

                # Auto Stepover
                row = col.row(align=True)
                if calc_warnings.get("engagement"):
                    row.alert = True
                row.prop(self.machine, "auto_calc_stepover")
                if calc_warnings.get("engagement"):
                    row.label(text="", icon='ERROR')

                # Auto Layer Height
                col.prop(self.machine, "auto_calc_layer_height")

                # Auto RPM
                row = col.row(align=True)
                row.prop(self.machine, "auto_calc_rpm")

                # Auto Feedrate
                row = col.row(align=True)
                if calc_warnings.get("power"):
                    row.alert = True
                row.prop(self.machine, "auto_calc_feedrate")
                if calc_warnings.get("power"):
                    row.label(text="", icon='ERROR')

                # Calculation Details (collapsible)
                if calc_lines:
                    subpanel.separator()
                    det_header, det_panel = subpanel.panel(
                        idname="calc_details", default_closed=True
                    )
                    det_header.label(text="Calculation Details")
                    det_header.operator("fabex.copy_calc_details", text="", icon='COPYDOWN')
                    if det_panel:
                        for line in calc_lines:
                            if line.startswith("\u2500"):
                                det_panel.separator(factor=0.5)
                                row = det_panel.row()
                                row.scale_y = 0.7
                                row.label(text=line)
                            elif line.startswith("WARNING"):
                                row = det_panel.row()
                                row.scale_y = 0.7
                                row.alert = True
                                row.label(text=line, icon='ERROR')
                            elif line.startswith("NOTE"):
                                row = det_panel.row()
                                row.scale_y = 0.7
                                row.label(text=line, icon='INFO')
                            else:
                                row = det_panel.row()
                                row.scale_y = 0.7
                                row.label(text=line)
