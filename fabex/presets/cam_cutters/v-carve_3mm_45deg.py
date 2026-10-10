import bpy

d = bpy.context.scene.cam_operations[bpy.context.scene.cam_active_operation]

d.cutter_type = "VCARVE"
d.cutter_diameter = 0.003
d.cutter_length = 0.025
d.cutter_tip_angle = 45.0
