"""Fabex 'machine_settings.py'


All CAM machine properties.
"""

from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
)
from bpy.types import PropertyGroup

from ..constants import PRECISION
from ..utilities.custom_settings_utils import (
    save_custom_settings,
    save_custom_settings_and_recalc,
)
from ..utilities.machine_utils import (
    update_machine,
    update_unit_system,
)


def _update_machine_and_save(self, context):
    """Chain update_machine + save_custom_settings for Job Start Height."""
    update_machine(self, context)
    save_custom_settings(self, context)


class CAM_MACHINE_Properties(PropertyGroup):
    """stores all data for machines"""

    path_color: FloatVectorProperty(
        name="Path Color",
        description="Color of the CAM_path object in the viewport",
        size=4,
        default=(0.0, 1.0, 0.0, 1.0),
        subtype="COLOR",
    )

    wire_color: FloatVectorProperty(
        name="Wire Color",
        description="Color of the CAM_Machine box in the viewport",
        size=4,
        default=(1.0, 1.0, 0.0, 1.0),
        subtype="COLOR",
    )

    unit_system: EnumProperty(
        name="Units",
        items=(
            (
                "INCHES",
                "Inches (in)",
                "Dimensions use Inches (Imperial)",
            ),
            (
                "MILLIMETERS",
                "Millimeters (mm)",
                "Dimensions use Millimeters (Metric)",
            ),
        ),
        update=update_unit_system,
    )
    # name = StringProperty(name="Machine Name", default="Machine")
    post_processor: EnumProperty(
        name="Post Processor",
        items=(
            (
                "ISO",
                "Iso",
                "Exports standardized gcode ISO 6983 (RS-274)",
            ),
            (
                "MACH3",
                "Mach3",
                "Default mach3",
            ),
            (
                "EMC",
                "LinuxCNC - EMC2",
                "Linux based CNC control software - formally EMC2",
            ),
            (
                "FADAL",
                "Fadal",
                "Fadal VMC",
            ),
            (
                "GRBL",
                "grbl",
                "Optimized gcode for grbl firmware on Arduino with cnc shield",
            ),
            (
                "HEIDENHAIN",
                "Heidenhain",
                "Heidenhain",
            ),
            (
                "HEIDENHAIN530",
                "Heidenhain530",
                "Heidenhain530",
            ),
            (
                "TNC151",
                "Heidenhain TNC151",
                "Post Processor for the Heidenhain TNC151 machine",
            ),
            (
                "SIEGKX1",
                "Sieg KX1",
                "Sieg KX1",
            ),
            (
                "HM50",
                "Hafco HM-50",
                "Hafco HM-50",
            ),
            (
                "CENTROID",
                "Centroid M40",
                "Centroid M40",
            ),
            (
                "ANILAM",
                "Anilam Crusader M",
                "Anilam Crusader M",
            ),
            (
                "GRAVOS",
                "Gravos",
                "Gravos",
            ),
            (
                "WIN-PC",
                "WinPC-NC",
                "German CNC by Burkhard Lewetz",
            ),
            (
                "SHOPBOT MTC",
                "ShopBot MTC",
                "ShopBot MTC",
            ),
            (
                "LYNX_OTTER_O",
                "Lynx Otter o",
                "Lynx Otter o",
            ),
            (
                "USER",
                "User Custom",
                "Customizable User Post Processor (based on ISO)",
            ),
        ),
        description="Post Processor",
        default="MACH3",
    )
    # position definitions:
    use_position_definitions: BoolProperty(
        name="Use Position Definitions",
        description="Define own positions for op start, toolchange, ending position",
        default=False,
    )
    starting_position: FloatVectorProperty(
        name="Start Position",
        default=(0, 0, 0),
        unit="LENGTH",
        precision=PRECISION,
        subtype="XYZ",
        update=update_machine,
    )
    mtc_position: FloatVectorProperty(
        name="MTC Position",
        default=(0, 0, 0),
        unit="LENGTH",
        precision=PRECISION,
        subtype="XYZ",
        update=update_machine,
    )
    ending_position: FloatVectorProperty(
        name="End Position",
        default=(0, 0, 0),
        unit="LENGTH",
        precision=PRECISION,
        subtype="XYZ",
        update=update_machine,
    )
    use_job_start_height: BoolProperty(
        name="Use Job Start Height (Machine Z)",
        description="Raise Z to a safe height before any XY move at job start, "
        "guaranteeing clearance over toolsetters and fixtures",
        default=False,
        update=_update_machine_and_save,
    )
    job_start_height: FloatProperty(
        name="Job Start Height",
        description="Z height the spindle retracts to before moving to the XY start position. "
        "Must be high enough to clear all fixtures and toolsetters",
        default=0.05,
        min=0.0,
        max=10.0,
        precision=3,
        update=_update_machine_and_save,
    )
    working_area: FloatVectorProperty(
        name="Work Area",
        default=(0.500, 0.500, 0.100),
        unit="LENGTH",
        precision=PRECISION,
        subtype="XYZ",
        update=update_machine,
    )
    feedrate_min: FloatProperty(
        name="Feedrate Minimum /min",
        default=0.0,
        min=0.00001,
        max=320000,
        precision=0,
    )
    feedrate_max: FloatProperty(
        name="Feedrate Maximum /min",
        default=2,
        min=0.00001,
        max=320000,
        precision=0,
    )
    feedrate_default: FloatProperty(
        name="Feedrate Default /min",
        default=1.5,
        min=0.00001,
        max=320000,
        precision=0,
    )
    feedrate_rapid: FloatProperty(
        name="Rapids /min",
        description="Rapid traverse speed. Output as a comment in the G-code header and "
        "as an F word on G00 moves",
        default=5.0,
        min=0.00001,
        max=320000,
        precision=0,
        update=save_custom_settings,
    )
    hourly_rate: FloatProperty(
        name="Price per Hour",
        default=100,
        min=0.005,
        precision=2,
    )

    # UNSUPPORTED:
    spindle_min: FloatProperty(
        name="Spindle Speed Minimum RPM",
        default=5000,
        min=0.00001,
        max=320000,
        precision=1,
    )
    spindle_max: FloatProperty(
        name="Spindle Speed Maximum RPM",
        default=30000,
        min=0.00001,
        max=320000,
        precision=1,
    )
    spindle_default: FloatProperty(
        name="Spindle Speed Default RPM",
        default=15000,
        min=0.00001,
        max=320000,
        precision=1,
    )
    spindle_start_time: FloatProperty(
        name="Spindle Start Delay Seconds",
        description="Wait for the spindle to start spinning before starting "
        "the feeds , in seconds",
        default=0,
        min=0.0000,
        max=320000,
        precision=1,
    )

    # Spindle Slow Start Settings
    spindle_slow_start_enable: BoolProperty(
        name="Spindle Slow Start Enable",
        description="Enable gradual spindle speed ramping from minimum to target speed",
        default=False,
    )
    spindle_slow_start_steps: IntProperty(
        name="Slow Start Steps",
        description="Number of intermediate speed steps when ramping from minimum to target spindle speed. More steps = smoother acceleration",
        default=8,
        min=2,
        max=20,
    )
    spindle_slow_start_skip_threshold: FloatProperty(
        name="Skip Threshold (RPM)",
        description="If target speed is within this RPM of the minimum speed, skip slow start and go directly to target",
        default=500,
        min=0,
        max=5000,
        precision=0,
    )

    spindle_slow_start_total_time: FloatProperty(
        name="Total Ramp Time (seconds)",
        description="Total time in seconds to ramp from minimum to target speed, distributed equally across all steps",
        default=2.0,
        min=0.1,
        max=10.0,
        precision=1,
    )

    axis_4: BoolProperty(
        name="4th Axis",
        description="Machine has 4th axis",
        default=0,
    )
    axis_5: BoolProperty(
        name="5th Axis",
        description="Machine has 5th axis",
        default=0,
    )
    eval_splitting: BoolProperty(
        name="Split Files",
        description="Split gcode file with large number of operations",
        default=True,
    )  # split large files
    split_limit: IntProperty(
        name="Operations per File",
        description="Split files with larger number of operations than this",
        min=1000,
        max=20000000,
        default=800000,
    )
    collet_size: FloatProperty(
        name="Collet Size",
        description="Collet size for collision detection",
        default=33,
        min=0.00001,
        max=320000,
        precision=PRECISION,
        unit="LENGTH",
    )

    # post processor options

    output_block_numbers: BoolProperty(
        name="Output Block Numbers",
        description="Output block numbers ie N10 at start of line",
        default=False,
    )

    start_block_number: IntProperty(
        name="Start Block Number",
        description="The starting block number ie 10",
        default=10,
    )

    block_number_increment: IntProperty(
        name="Block Number Increment",
        description="How much the block number should " "increment for the next line",
        default=10,
    )

    output_tool_definitions: BoolProperty(
        name="Output Tool Definitions",
        description="Output tool definitions",
        default=True,
    )

    output_tool_change: BoolProperty(
        name="Output Tool Change Commands",
        description="Output tool change commands ie: Tn M06",
        default=True,
    )

    output_G43_on_tool_change: BoolProperty(
        name="Output G43 on Tool Change",
        description="Output G43 on tool change line",
        default=False,
    )

    # ── Custom Settings ──────────────────────────────────────────────

    # Path Control
    use_rapids: BoolProperty(
        name="Use Rapids",
        description="Enable G0 rapid moves in generated G-code",
        default=True,
        update=save_custom_settings,
    )
    show_rest_machining: BoolProperty(
        name="Show Rest Machining",
        description="Show the rest machining option in operation panels",
        default=False,
        update=save_custom_settings,
    )
    default_safe_height: FloatProperty(
        name="Default Safe Height",
        description="Default safe retract height for new operations",
        default=0.05,
        min=0.0,
        max=1.0,
        precision=3,
        update=save_custom_settings,
    )
    use_g64_trajectory: BoolProperty(
        name="G64 Trajectory",
        description="Output G64 continuous path mode for smoother motion",
        default=False,
        update=save_custom_settings,
    )
    g64_tolerance: FloatProperty(
        name="G64 Tolerance",
        description="Path blending tolerance for G64 (P value). "
        "Smaller = more accurate, larger = smoother motion",
        default=0.0001,
        min=0.000001,
        max=0.01,
        precision=PRECISION,
        update=save_custom_settings,
    )

    # Stock Material & Machine
    stock_material: EnumProperty(
        name="Stock Material",
        items=(
            ("SOFTWOOD", "Softwood", "Pine, cedar, balsa, etc."),
            ("HARDWOOD", "Hardwood", "Oak, maple, walnut, cherry, etc."),
            ("SOFT_PLASTICS", "Soft Plastics", "HDPE, nylon, UHMW, etc."),
            ("HARD_PLASTICS", "Hard Plastics", "Acrylic, polycarbonate, Delrin, etc."),
            ("ALUMINIUM_CAST", "Aluminium (Cast)", "Cast aluminium alloys"),
            ("ALUMINIUM_6061", "Aluminium (6061)", "6061-T6 and similar wrought alloys"),
            ("ALUMINIUM_7075", "Aluminium (7075)", "7075-T6 and similar high-strength alloys"),
            ("BRASS_SOFT", "Brass (Soft)", "Free-cutting brass, 360 alloy"),
            ("BRASS_MEDIUM", "Brass (Medium)", "Yellow brass, cartridge brass"),
            ("BRASS_HARD", "Brass (Hard)", "Naval brass, high-strength alloys"),
            ("BRONZE", "Bronze", "Bronze alloys"),
            ("CARBON_STEEL", "Carbon Steel", "Mild and carbon steel"),
            ("STAINLESS_303", "Stainless (303)", "Free-machining stainless steel"),
            ("STAINLESS_304", "Stainless (304)", "General-purpose stainless steel"),
            ("STAINLESS_316", "Stainless (316)", "Marine-grade stainless steel"),
            ("TOOL_STEEL", "Tool Steel", "Hardened tool and die steel"),
        ),
        description="Stock material — used for calculating optimal cutting parameters",
        default="HARDWOOD",
        update=save_custom_settings_and_recalc,
    )
    machine_stiffness: IntProperty(
        name="Machine Stiffness",
        description="Machine rigidity on a scale of 1-10. "
        "1 = very flexible (engraving machine), "
        "10 = very rigid (can mill steel at high feedrates without chatter)",
        default=5,
        min=1,
        max=10,
        update=save_custom_settings_and_recalc,
    )

    # Machine Power
    spindle_power_watts: FloatProperty(
        name="Spindle Power (watts)",
        description="Rated spindle power in watts. Used for power warning when "
        "calculated cutting power exceeds machine capability. "
        "Common: 800W (small router), 1500W (mid), 2200W (large)",
        default=800,
        min=50,
        max=50000,
        precision=0,
        update=save_custom_settings_and_recalc,
    )

    # Auto-Calculate
    auto_calc_stepover: BoolProperty(
        name="Auto Stepover",
        description="Automatically calculate optimal stepover when a tool is chosen or edited",
        default=False,
        update=save_custom_settings_and_recalc,
    )
    auto_calc_layer_height: BoolProperty(
        name="Auto Layer Height",
        description="Automatically calculate optimal layer height when a tool is chosen or edited",
        default=False,
        update=save_custom_settings_and_recalc,
    )
    auto_calc_rpm: BoolProperty(
        name="Auto RPM",
        description="Automatically calculate optimal spindle RPM from surface speed tables "
        "based on material and tool material. Clamped to spindle min/max",
        default=False,
        update=save_custom_settings_and_recalc,
    )
    auto_calc_feedrate: BoolProperty(
        name="Auto Feedrate",
        description="Automatically calculate optimal feedrate from chip load, flutes, "
        "and RPM with chip thinning compensation",
        default=False,
        update=save_custom_settings_and_recalc,
    )
    surface_finish_quality: EnumProperty(
        name="Surface Finish",
        items=(
            ("FINE", "Fine", "Scallop height 0.0001\" (2.5 μm) — best surface finish"),
            ("STANDARD", "Standard", "Scallop height 0.0005\" (12.7 μm) — good balance"),
            ("DRAFT", "Draft", "Scallop height 0.001\" (25.4 μm) — faster machining"),
        ),
        description="Surface finish quality preset — sets scallop height for ball nose stepover calculation",
        default="STANDARD",
        update=save_custom_settings_and_recalc,
    )
    tool_material: EnumProperty(
        name="Tool Material",
        items=(
            ("HSS", "HSS", "High Speed Steel — lower surface speeds, more forgiving"),
            ("CARBIDE", "Carbide", "Tungsten Carbide — higher surface speeds, more rigid"),
        ),
        description="Tool material — determines surface speed for RPM calculation",
        default="CARBIDE",
        update=save_custom_settings_and_recalc,
    )
    plunge_rate_factor: FloatProperty(
        name="Plunge Rate Factor",
        description="Plunge feed = horizontal feed × this factor. "
        "Plunge engages only the tip, so typically 50% of horizontal feed",
        default=0.5,
        min=0.1,
        max=1.0,
        precision=2,
        update=save_custom_settings_and_recalc,
    )

    # Operation Defaults
    default_exact_mode: BoolProperty(
        name="Exact Mode",
        description="Default value for 'Use Exact Mode' on new operations",
        default=True,
        update=save_custom_settings,
    )
    default_use_opencamlib: BoolProperty(
        name="Use OpenCamLib",
        description="Default value for 'Use OpenCamLib' on new operations",
        default=False,
        update=save_custom_settings,
    )
    default_use_modifiers: BoolProperty(
        name="Use Mesh Modifiers",
        description="Default value for 'Use Mesh Modifiers' on new operations",
        default=True,
        update=save_custom_settings,
    )
    default_reduce_path_points: BoolProperty(
        name="Reduce Path Points",
        description="Default value for 'Reduce Path Points' on new operations",
        default=True,
        update=save_custom_settings,
    )
    default_path_threshold: FloatProperty(
        name="Reduction Threshold (μm)",
        description="Default path point reduction threshold for new operations. "
        "Example: 2.54 μm = 0.00254 mm = 0.0001\"",
        default=0.2,
        min=0.000000001,
        max=1000,
        precision=20,
        update=save_custom_settings,
    )

    # Terrain-Aware Rapids (experimental)
    terrain_aware_rapids: BoolProperty(
        name="Terrain Aware Rapids",
        description="Experimental: fly just above stock surface between cuts "
        "instead of retracting to full safe height. Uses Z-map from roughing "
        "pass to determine terrain height. Only affects rest machining",
        default=False,
        update=save_custom_settings,
    )
    terrain_clearance: FloatProperty(
        name="Terrain Clearance (mm)",
        description="Height above stock surface for terrain-aware rapid traverses. "
        "Applied on top of the dilated (worst-case) Z-map surface",
        default=3.0,
        min=2.0,
        max=20.0,
        precision=1,
        update=save_custom_settings,
    )

    # Debugging
    verbose_logging: BoolProperty(
        name="Verbose Logging",
        description="Log detailed per-layer and per-island information during "
        "rest machining path calculation",
        default=False,
        update=save_custom_settings,
    )
