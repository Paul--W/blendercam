"""Fabex 'calc_ops.py'

Operators for the cutting parameter calculation UI.
"""

import bpy
from bpy.types import Operator


class FABEX_OT_copy_calc_details(Operator):
    """Copy calculation details to the system clipboard"""

    bl_idname = "fabex.copy_calc_details"
    bl_label = "Copy Calculation Details"

    def execute(self, context):
        from ..utilities.cutting_params import get_last_calc_display

        lines, _ = get_last_calc_display()
        if not lines:
            self.report({"INFO"}, "No calculation details available")
            return {"CANCELLED"}

        text = "\n".join(lines)
        context.window_manager.clipboard = text
        self.report({"INFO"}, "Calculation details copied to clipboard")
        return {"FINISHED"}
