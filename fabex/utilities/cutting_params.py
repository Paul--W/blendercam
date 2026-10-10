"""Fabex 'cutting_params.py'

Calculation engine for optimal cutting parameters — stepover, layer height,
feedrate, chip load, engagement angle, RPM, plunge rate, and power estimation.

Single source of truth: called by auto-calc callbacks, [i] popup operator,
log output, and calculation details panel.

All internal calculations use MILLIMETERS (matching Machinery's Handbook and
FreeCAD Feeds & Speeds addon).  Blender stores unit="LENGTH" properties in
meters; conversion happens only at the boundary in auto_calculate_for_operation.
Display adapts to the machine's unit_system (INCHES or MILLIMETERS).

Power estimation formulas from Machinery's Handbook (28th ed.) via FreeCAD
Feeds & Speeds addon (GPL). Tool length derating based on cantilever beam
deflection model (L³/D⁴).
"""

from math import acos, degrees, pi, sqrt

from .logging_utils import log


# ── Unified Material Data Table ──────────────────────────────────────────
# Consolidates chip load, stepover, layer height, surface speed, and power
# constant (Kp) per material into a single structure.
# Chip load base values are mm/tooth for a 3.175mm (1/8") reference tool.
_MATERIAL_DATA = {
    # ── Wood (use_max_rpm: always run at spindle max for clean cuts) ──
    "SOFTWOOD": {
        "chip_load_base": 0.10,             # mm/tooth for 3.175mm ref tool
        "stepover_rough": 0.60,             # ae/D fraction
        "stepover_finish": 0.20,            # ae/D fraction
        "layer_height_rough": 0.80,         # ap/D fraction
        "surface_speed_hss": 225,           # m/min (FreeCAD; reference only)
        "surface_speed_carbide": 255,       # m/min (FreeCAD; reference only)
        "use_max_rpm": True,                # wood: max RPM for clean cuts
        "Kp": 0.5,                          # power constant
    },
    "HARDWOOD": {
        "chip_load_base": 0.08,
        "stepover_rough": 0.50,
        "stepover_finish": 0.18,
        "layer_height_rough": 0.60,
        "surface_speed_hss": 145,           # m/min (FreeCAD; reference only)
        "surface_speed_carbide": 275,       # m/min (FreeCAD; reference only)
        "use_max_rpm": True,                # wood: max RPM for clean cuts
        "Kp": 0.75,
    },
    # ── Plastics ──
    "SOFT_PLASTICS": {
        "chip_load_base": 0.10,             # similar to softwood
        "stepover_rough": 0.55,
        "stepover_finish": 0.20,
        "layer_height_rough": 0.70,
        "surface_speed_hss": 225,           # m/min (FreeCAD)
        "surface_speed_carbide": 255,       # m/min (FreeCAD)
        "Kp": 0.5,
    },
    "HARD_PLASTICS": {
        "chip_load_base": 0.08,             # similar to hardwood
        "stepover_rough": 0.50,
        "stepover_finish": 0.18,
        "layer_height_rough": 0.60,
        "surface_speed_hss": 225,           # m/min (FreeCAD)
        "surface_speed_carbide": 275,       # m/min (FreeCAD)
        "Kp": 0.75,
    },
    # ── Aluminium ──
    "ALUMINIUM_6061": {
        "chip_load_base": 0.076,
        "stepover_rough": 0.45,
        "stepover_finish": 0.15,
        "layer_height_rough": 0.50,
        "surface_speed_hss": 175,           # m/min (FreeCAD)
        "surface_speed_carbide": 395,       # m/min (FreeCAD)
        "Kp": 0.9,
    },
    "ALUMINIUM_7075": {
        "chip_load_base": 0.064,            # slightly harder than 6061
        "stepover_rough": 0.40,
        "stepover_finish": 0.15,
        "layer_height_rough": 0.45,
        "surface_speed_hss": 175,           # m/min (FreeCAD)
        "surface_speed_carbide": 395,       # m/min (FreeCAD)
        "Kp": 0.9,
    },
    "ALUMINIUM_CAST": {
        "chip_load_base": 0.089,            # softer, more forgiving
        "stepover_rough": 0.50,
        "stepover_finish": 0.18,
        "layer_height_rough": 0.55,
        "surface_speed_hss": 175,           # m/min (FreeCAD)
        "surface_speed_carbide": 395,       # m/min (FreeCAD)
        "Kp": 0.68,
    },
    # ── Brass ──
    "BRASS_SOFT": {
        "chip_load_base": 0.064,
        "stepover_rough": 0.45,
        "stepover_finish": 0.15,
        "layer_height_rough": 0.45,
        "surface_speed_hss": 125,           # m/min (FreeCAD)
        "surface_speed_carbide": 300,       # m/min (FreeCAD)
        "Kp": 0.68,
    },
    "BRASS_MEDIUM": {
        "chip_load_base": 0.051,
        "stepover_rough": 0.40,
        "stepover_finish": 0.12,
        "layer_height_rough": 0.40,
        "surface_speed_hss": 175,           # m/min (FreeCAD)
        "surface_speed_carbide": 350,       # m/min (FreeCAD)
        "Kp": 1.36,
    },
    "BRASS_HARD": {
        "chip_load_base": 0.038,
        "stepover_rough": 0.35,
        "stepover_finish": 0.10,
        "layer_height_rough": 0.35,
        "surface_speed_hss": 200,           # m/min (FreeCAD)
        "surface_speed_carbide": 395,       # m/min (FreeCAD)
        "Kp": 2.27,
    },
    # ── Bronze ──
    "BRONZE": {
        "chip_load_base": 0.051,
        "stepover_rough": 0.35,
        "stepover_finish": 0.12,
        "layer_height_rough": 0.35,
        "surface_speed_hss": 40,
        "surface_speed_carbide": 120,
        "Kp": 1.2,
    },
    # ── Steel ──
    "CARBON_STEEL": {
        "chip_load_base": 0.025,
        "stepover_rough": 0.25,
        "stepover_finish": 0.10,
        "layer_height_rough": 0.20,
        "surface_speed_hss": 35,            # m/min (FreeCAD)
        "surface_speed_carbide": 120,       # m/min (FreeCAD)
        "Kp": 1.88,
    },
    "STAINLESS_303": {
        "chip_load_base": 0.025,
        "stepover_rough": 0.25,
        "stepover_finish": 0.10,
        "layer_height_rough": 0.20,
        "surface_speed_hss": 25,            # m/min (FreeCAD)
        "surface_speed_carbide": 85,        # m/min (FreeCAD)
        "Kp": 2.07,
    },
    "STAINLESS_304": {
        "chip_load_base": 0.020,
        "stepover_rough": 0.20,
        "stepover_finish": 0.08,
        "layer_height_rough": 0.18,
        "surface_speed_hss": 10,            # m/min (FreeCAD)
        "surface_speed_carbide": 37.5,      # m/min (FreeCAD)
        "Kp": 2.07,
    },
    "STAINLESS_316": {
        "chip_load_base": 0.018,
        "stepover_rough": 0.18,
        "stepover_finish": 0.08,
        "layer_height_rough": 0.15,
        "surface_speed_hss": 7.5,           # m/min (FreeCAD)
        "surface_speed_carbide": 25,        # m/min (FreeCAD)
        "Kp": 2.07,
    },
    "TOOL_STEEL": {
        "chip_load_base": 0.020,
        "stepover_rough": 0.20,
        "stepover_finish": 0.08,
        "layer_height_rough": 0.15,
        "surface_speed_hss": 12,            # m/min (FreeCAD)
        "surface_speed_carbide": 45,        # m/min (FreeCAD)
        "Kp": 1.88,
    },
}

# ── Power Constant (C) — Machinery's Handbook Table 2 ───────────────────
# Chipload (mm/tooth) → power constant. Smaller chips = less efficient.
_POWER_C_KEYS = [
    0.02, 0.05, 0.07, 0.10, 0.12, 0.15, 0.18, 0.20,
    0.22, 0.25, 0.28, 0.30, 0.33, 0.35, 0.38, 0.40,
    0.45, 0.50, 0.55, 0.60, 0.70, 0.75, 0.80, 0.90,
    1.00, 1.50,
]
_POWER_C_VALS = [
    1.70, 1.40, 1.30, 1.25, 1.20, 1.15, 1.11, 1.08,
    1.06, 1.04, 1.01, 1.00, 0.98, 0.97, 0.95, 0.94,
    0.92, 0.90, 0.88, 0.87, 0.84, 0.83, 0.82, 0.80,
    0.78, 0.72,
]

# ── Ball nose roughing stepover (fraction of R) ─────────────────────────────
# Based on Alain's recommendation: R/3 for standard roughing.
# Expressed as ae = R × fraction.  R/4=tight scallops, R/3=normal, R/2=fast.
_BALL_ROUGH_STEPOVER = {
    "FINE": 0.25,           # R/4 — tighter scallops, less finishing work
    "STANDARD": 1.0 / 3.0,  # R/3 — Alain's recommendation
    "DRAFT": 0.50,          # R/2 — fast roughing, heavier finishing
}

# ── Ball nose finishing stepover (fraction of R) ────────────────────────────
# Based on Alain's recommendation: R/5 for standard finishing.
# Scales naturally with tool size (unlike fixed scallop height).
_BALL_FINISH_STEPOVER = {
    "FINE": 1.0 / 6.0,      # R/6 — finest finish
    "STANDARD": 1.0 / 5.0,  # R/5 — Alain's recommendation
    "DRAFT": 0.25,           # R/4 — faster finishing, visible scallops
}

# Finishing layer height fraction of diameter (flat end mill)
_FINISHING_LAYER_FRACTION = 0.10

# Reference tool diameter for chip load scaling: 3.175mm (1/8")
_REF_DIAMETER = 3.175

# Power estimation constants (Machinery's Handbook)
_TOOL_WEAR_FACTOR = 1.1    # pg 1048 — 10% wear margin
_MACHINE_EFFICIENCY = 0.80  # pg 1049 — belt/spindle losses

# ── Calculation Display Cache ────────────────────────────────────────────
_last_calc_lines = []       # list of display strings for UI panel
_last_calc_warnings = {}    # dict of warning flags for UI


def get_last_calc_display():
    """Return cached display lines and warnings for the panel UI."""
    return _last_calc_lines, _last_calc_warnings


# ── Helper Functions ─────────────────────────────────────────────────────

def _interpolate(x, x_list, y_list):
    """Linear interpolation between table entries."""
    if x <= x_list[0]:
        return y_list[0]
    if x >= x_list[-1]:
        return y_list[-1]
    for i in range(len(x_list) - 1):
        if x_list[i] <= x <= x_list[i + 1]:
            t = (x - x_list[i]) / (x_list[i + 1] - x_list[i])
            return y_list[i] + t * (y_list[i + 1] - y_list[i])
    return y_list[-1]


def stiffness_factor(stiffness):
    """Convert stiffness 1-10 to a 0.30–1.00 multiplier."""
    return 0.3 + (stiffness - 1) * 0.7 / 9


def tool_length_factor(cutter_length, cutter_diameter, material_kp=0.75):
    """Derating factor based on tool stick-out to diameter ratio and material.

    Deflection scales with L³/D⁴ — longer tools chatter sooner.
    The geometric penalty (from L/D ratio) is scaled by material hardness
    via Kp (power constant).  Hardwood (Kp=0.75) is the baseline — softer
    materials reduce the penalty, harder materials amplify it.

    Returns a factor in [0.40, 1.0].
    """
    if cutter_diameter <= 0 or cutter_length <= 0:
        return 1.0
    ld_ratio = cutter_length / cutter_diameter
    if ld_ratio <= 3.0:
        return 1.0
    # Geometric penalty: 0 at L/D=3, 0.60 at L/D=7
    geometric_penalty = min(0.60, (ld_ratio - 3.0) * 0.15)
    # Scale penalty by material hardness (Kp relative to hardwood)
    _KP_HARDWOOD = 0.75
    material_scale = material_kp / _KP_HARDWOOD if _KP_HARDWOOD > 0 else 1.0
    penalty = geometric_penalty * material_scale
    return max(0.40, 1.0 - penalty)


def chip_thinning_factor(ae, D):
    """Chip thinning compensation factor for flat end mills.

    When stepover (ae) < D/2, the effective chip is thinner than nominal.
    Feedrate should be increased by this factor to maintain proper chip load.

    Returns:
        float: CTF >= 1.0, capped at 3.0
    """
    if ae >= D / 2:
        return 1.0
    if ae <= 0 or D <= 0:
        return 1.0
    ctf = D / (2 * ae)
    return min(ctf, 3.0)


def engagement_angle(ae, R):
    """Calculate engagement angle in degrees.

    angle = arccos(1 - ae/R)  where R = D/2
    """
    if R <= 0 or ae <= 0:
        return 0.0
    ratio = 1.0 - ae / R
    ratio = max(-1.0, min(1.0, ratio))  # clamp for floating point
    return degrees(acos(ratio))


def calculate_rpm(material, tool_material, cutter_diameter_mm,
                  spindle_min=None, spindle_max=None):
    """Calculate optimal RPM.

    For wood (use_max_rpm=True): use machine maximum RPM.
    Higher RPM = cleaner cuts, less tear out.

    For metals: RPM = (1000 × surface_speed) / (π × D_mm),
    clamped to spindle min/max to protect the tool edge.

    Returns:
        tuple: (rpm, surface_speed, was_clamped, clamp_reason, ideal_rpm)
    """
    mat_data = _MATERIAL_DATA.get(material, _MATERIAL_DATA["HARDWOOD"])
    speed_key = "surface_speed_carbide" if tool_material == "CARBIDE" else "surface_speed_hss"
    surface_speed = mat_data[speed_key]

    D_mm = cutter_diameter_mm
    if D_mm <= 0:
        return (15000, surface_speed, False, "", 15000)

    # Wood: use max RPM for clean cuts, no surface speed limit
    if mat_data.get("use_max_rpm") and spindle_max:
        rpm = spindle_max
        # Calculate surface speed for display (informational)
        surface_speed = (pi * D_mm * rpm) / 1000.0
        return (rpm, surface_speed, False, "", rpm)

    # Metals: calculate from surface speed tables
    ideal_rpm = (1000.0 * surface_speed) / (pi * D_mm)
    rpm = ideal_rpm

    was_clamped = False
    clamp_reason = ""
    if spindle_max and rpm > spindle_max:
        rpm = spindle_max
        was_clamped = True
        clamp_reason = "limited to machine maximum"
    elif spindle_min and rpm < spindle_min:
        rpm = spindle_min
        was_clamped = True
        clamp_reason = "raised to machine minimum"

    return (rpm, surface_speed, was_clamped, clamp_reason, ideal_rpm)


def calculate_power(ae_mm, ap_mm, feedrate_mm_min, chip_load_mm,
                    material, spindle_power_watts=None):
    """Calculate cutting power estimation.

    All inputs in mm / mm/min.  MRR formula: Q = (ae × ap × feed) / 60000 cm³/s.

    Returns:
        dict with keys: mrr_mm3, power_c, Kp, Q_cm3s, Pc_kw, Pm_kw, hp, watts,
        power_warning (bool), power_warning_msg (str)
    """
    mat_data = _MATERIAL_DATA.get(material, _MATERIAL_DATA["HARDWOOD"])
    Kp = mat_data["Kp"]

    # MRR in mm³/min, then to cm³/s
    mrr_mm3 = ae_mm * ap_mm * feedrate_mm_min
    Q = mrr_mm3 / 60000.0  # cm³/s

    # Power constant C — interpolated from chipload (mm/tooth)
    C = _interpolate(chip_load_mm, _POWER_C_KEYS, _POWER_C_VALS)

    # Power at cutter (kW)
    Pc = Kp * C * Q * _TOOL_WEAR_FACTOR

    # Power at motor (kW)
    Pm = Pc / _MACHINE_EFFICIENCY

    # Conversions
    hp = Pm * 1.341
    watts = hp * 745.69

    power_warning = False
    power_warning_msg = ""
    if spindle_power_watts and watts > spindle_power_watts:
        power_warning = True
        power_warning_msg = (
            f"Estimated {watts:.0f}W exceeds spindle rating of "
            f"{spindle_power_watts:.0f}W"
        )

    return {
        "mrr_mm3": mrr_mm3,
        "power_c": C,
        "Kp": Kp,
        "Q_cm3s": Q,
        "Pc_kw": Pc,
        "Pm_kw": Pm,
        "hp": hp,
        "watts": watts,
        "power_warning": power_warning,
        "power_warning_msg": power_warning_msg,
    }


def calculate_cutting_params(
    cutter_type,
    cutter_diameter_mm,
    cutter_flutes,
    cutter_length_mm,
    material,
    stiffness,
    spindle_rpm,
    is_roughing,
    surface_finish="STANDARD",
    feedrate_max_mm=None,
    tool_material="CARBIDE",
    spindle_min=None,
    spindle_max=None,
    auto_calc_rpm=False,
    plunge_rate_factor=0.5,
    spindle_power_watts=None,
    unit_system="INCHES",
    shank_diameter_mm=None,
    ball_radius_mm=None,
):
    """Calculate optimal cutting parameters.

    All dimensional inputs and outputs are in MILLIMETERS.

    Args:
        cutter_type: "END", "BALLNOSE", "BALLCONE", etc.
        cutter_diameter_mm: Tool diameter in mm
        cutter_flutes: Number of flutes
        cutter_length_mm: Tool stick-out length in mm
        material: One of the _MATERIAL_DATA keys
        stiffness: Machine stiffness 1-10
        spindle_rpm: Spindle speed (used when auto_calc_rpm is False)
        is_roughing: True for roughing, False for finishing
        surface_finish: "FINE", "STANDARD", or "DRAFT"
        feedrate_max_mm: Machine max feedrate in mm/min (optional)
        tool_material: "HSS" or "CARBIDE"
        spindle_min: Minimum spindle RPM for clamping
        spindle_max: Maximum spindle RPM for clamping
        auto_calc_rpm: If True, derive RPM from surface speed tables
        plunge_rate_factor: Plunge feed = horizontal feed × this factor
        spindle_power_watts: Machine spindle power for warning
        unit_system: "INCHES" or "MILLIMETERS" — controls display only
        shank_diameter_mm: Shank diameter in mm (for L/D deflection calc).
            Defaults to cutter_diameter_mm if not provided.

    Returns:
        dict with all calculated values in mm, intermediate values,
        and warning flags.
    """
    D = cutter_diameter_mm
    is_ball = cutter_type in ("BALLNOSE", "BALLCONE")
    # Ball radius: BALLCONE has an independent ball tip (may differ from body
    # diameter).  BALLNOSE always has R = D/2 by definition.
    if cutter_type == "BALLCONE" and ball_radius_mm and ball_radius_mm > 0:
        R = ball_radius_mm
    else:
        R = D / 2.0
    # Deflection uses shank diameter (the rigid part), not the tip.
    # Shank is never smaller than cutter diameter (straight end mills).
    if shank_diameter_mm and shank_diameter_mm > 0:
        shank_D = max(shank_diameter_mm, D)
    else:
        shank_D = D
    mat_data = _MATERIAL_DATA.get(material, _MATERIAL_DATA["HARDWOOD"])
    sf = stiffness_factor(stiffness)
    tlf = tool_length_factor(cutter_length_mm, shank_D, material_kp=mat_data["Kp"])
    ef = sf * tlf  # effective factor

    # ── RPM ────────────────────────────────────────────────────────
    rpm_clamped = False
    rpm_clamp_reason = ""
    surface_speed = 0
    ideal_rpm = 0
    if auto_calc_rpm and D > 0:
        rpm, surface_speed, rpm_clamped, rpm_clamp_reason, ideal_rpm = calculate_rpm(
            material, tool_material, D, spindle_min, spindle_max
        )
    else:
        rpm = spindle_rpm

    # ── Chip Load (mm/tooth) ──────────────────────────────────────
    base_cl = mat_data["chip_load_base"]
    # Scale with tool diameter (sqrt scaling from 3.175mm reference)
    dia_scale = sqrt(D / _REF_DIAMETER) if D > 0 else 1.0
    scaled_cl = base_cl * dia_scale * ef

    # ── Stepover (mm) ─────────────────────────────────────────────
    # Surface finish scales roughing stepover: DRAFT=100%, STANDARD=80%, FINE=60%
    _ROUGHING_FINISH_SCALE = {"FINE": 0.60, "STANDARD": 0.80, "DRAFT": 1.0}
    rough_scale = _ROUGHING_FINISH_SCALE.get(surface_finish, 0.80)

    if is_ball and not is_roughing:
        # Ball nose finishing: fraction of radius.
        # R/5 standard per Alain's recommendation — scales naturally
        # with tool size unlike fixed scallop height.
        ball_frac = _BALL_FINISH_STEPOVER.get(surface_finish,
                                               _BALL_FINISH_STEPOVER["STANDARD"])
        ae = R * ball_frac
        stepover_frac = ae / D if D > 0 else 0
        stepover_method = "ball_finishing"
    elif is_ball and is_roughing:
        # Ball nose roughing: fraction of radius, not diameter.
        # R/3 standard per Alain's recommendation — much tighter than
        # flat end mill roughing (which uses % of D).
        ball_frac = _BALL_ROUGH_STEPOVER.get(surface_finish,
                                              _BALL_ROUGH_STEPOVER["STANDARD"])
        ae = R * ball_frac * ef
        stepover_frac = ae / D if D > 0 else 0
        stepover_method = "ball_roughing"
    else:
        # Flat end mill
        if is_roughing:
            stepover_frac = mat_data["stepover_rough"] * rough_scale
        else:
            stepover_frac = mat_data["stepover_finish"]
        ae = D * stepover_frac * ef
        stepover_method = "roughing" if is_roughing else "finishing"

    # Safety clamp: never exceed material's maximum roughing engagement
    ae_max = D * mat_data["stepover_rough"]
    if ae > ae_max:
        ae = ae_max
        stepover_frac = mat_data["stepover_rough"]
        stepover_method += "_clamped"

    # ── Engagement ─────────────────────────────────────────────────
    # Ball cutters: percentage of tip diameter (same as info panel).
    # Flat end mills: angular engagement arccos(1 - ae/R).
    if is_ball:
        eng_deg = 100.0 * ae / (2.0 * R) if R > 0 else 0.0  # % of tip Ø
    else:
        eng_deg = engagement_angle(ae, R)

    # ── Chip Thinning ──────────────────────────────────────────────
    ctf = chip_thinning_factor(ae, D)

    # ── Feedrate (mm/min) ─────────────────────────────────────────
    base_feedrate = scaled_cl * cutter_flutes * rpm
    adjusted_feedrate = base_feedrate * ctf
    feedrate_was_clamped = False
    if feedrate_max_mm and adjusted_feedrate > feedrate_max_mm:
        adjusted_feedrate = feedrate_max_mm
        feedrate_was_clamped = True

    # ── Layer Height (mm) ─────────────────────────────────────────
    if is_roughing:
        lh_frac = mat_data["layer_height_rough"]
        layer_height = D * lh_frac * ef
        layer_height_method = "roughing"
    elif is_ball:
        # Ball nose finishing: layer height based on tip diameter (2×R),
        # NOT stepover.  Stepover controls XY scallop (surface finish);
        # layer height controls Z depth per pass (limited by cutter
        # strength and material hardness, not surface finish).
        # Base: 75% of tip diameter, scaled by material hardness.
        _BALL_FINISH_FRAC = 0.75  # base fraction of tip diameter
        _HARDWOOD_FRAC = 0.60     # baseline: hardwood layer_height_rough
        lh_frac = mat_data["layer_height_rough"]
        material_factor = min(1.2, lh_frac / _HARDWOOD_FRAC)
        tip_diameter = 2.0 * R  # ball tip diameter (may differ from D for BALLCONE)
        layer_height = tip_diameter * _BALL_FINISH_FRAC * material_factor
        layer_height_method = "ball_finish"
    else:
        # Flat end mill finishing: light passes
        lh_frac = _FINISHING_LAYER_FRACTION
        layer_height = D * lh_frac * ef
        layer_height_method = "finishing"

    # ── Power Estimation & feedrate-for-power limiting ────────────
    power = calculate_power(
        ae, layer_height, adjusted_feedrate,
        scaled_cl, material, spindle_power_watts
    )
    feedrate_power_limited = False
    pre_power_feedrate = adjusted_feedrate
    if power["power_warning"] and spindle_power_watts:
        # Scale feedrate so Pm exactly equals spindle rating.
        # Pm is proportional to feedrate (through MRR), so:
        scale = spindle_power_watts / power["watts"]
        adjusted_feedrate = adjusted_feedrate * scale
        feedrate_power_limited = True
        # Recalculate power with the reduced feedrate
        power = calculate_power(
            ae, layer_height, adjusted_feedrate,
            scaled_cl, material, spindle_power_watts
        )

    # ── Plunge Rate (mm/min) ──────────────────────────────────────
    plunge_rate = adjusted_feedrate * plunge_rate_factor

    # ── L/D warning (uses shank diameter for deflection) ────────────
    # Threshold depends on material hardness and operation type:
    #   Finishing in soft materials: forces are minimal, L/D 8 is fine
    #   Roughing in metals: higher forces, warn earlier at L/D 5
    ld_ratio = cutter_length_mm / shank_D if shank_D > 0 else 0
    _hard_materials = {
        "CARBON_STEEL", "STAINLESS_303", "STAINLESS_304", "STAINLESS_316",
        "TOOL_STEEL", "BRASS_SOFT", "BRASS_MEDIUM", "BRASS_HARD", "BRONZE",
    }
    if is_roughing and material in _hard_materials:
        ld_threshold = 5.0
    elif is_roughing:
        ld_threshold = 6.0
    elif material in _hard_materials:
        ld_threshold = 7.0
    else:
        ld_threshold = 8.0
    ld_warning = ld_ratio > ld_threshold

    # ── Engagement warning ─────────────────────────────────────────
    # The safety clamp above ensures ae never exceeds the material's
    # maximum roughing engagement.  Warn only if somehow exceeded
    # (shouldn't happen with the clamp, but defensive).
    if is_ball:
        eng_limit = 100.0 * mat_data["stepover_rough"] + 5.0  # small margin for float
        engagement_warning = eng_deg > eng_limit
    else:
        engagement_warning = eng_deg > 90.0  # degrees

    result = {
        "cutter_type": cutter_type,
        "cutter_diameter_mm": D,
        "ball_radius_mm": R if is_ball else D / 2.0,
        "shank_diameter_mm": shank_D,
        "cutter_flutes": cutter_flutes,
        "cutter_length_mm": cutter_length_mm,
        "ld_ratio": ld_ratio,
        "material": material,
        "tool_material": tool_material,
        "stiffness": stiffness,
        "stiffness_factor": sf,
        "tool_length_factor": tlf,
        "effective_factor": ef,
        "dia_scale": dia_scale,
        "spindle_rpm": rpm,
        "auto_calc_rpm": auto_calc_rpm,
        "surface_speed": surface_speed,
        "rpm_clamped": rpm_clamped,
        "rpm_clamp_reason": rpm_clamp_reason,
        "ideal_rpm": ideal_rpm,
        "use_max_rpm": mat_data.get("use_max_rpm", False),
        "is_roughing": is_roughing,
        "surface_finish": surface_finish,
        "base_chip_load": base_cl,
        "scaled_chip_load": scaled_cl,
        "stepover_mm": ae,
        "stepover_method": stepover_method,
        "stepover_frac": stepover_frac,
        "engagement_deg": eng_deg,
        "engagement_warning": engagement_warning,
        "ctf": ctf,
        "base_feedrate_mm": base_feedrate,
        "adjusted_feedrate_mm": adjusted_feedrate,
        "feedrate_was_clamped": feedrate_was_clamped,
        "feedrate_power_limited": feedrate_power_limited,
        "pre_power_feedrate_mm": pre_power_feedrate,
        "plunge_rate_mm": plunge_rate,
        "plunge_rate_factor": plunge_rate_factor,
        "layer_height_mm": layer_height,
        "layer_height_method": layer_height_method,
        "layer_height_frac": lh_frac,
        "layer_height_material_factor": material_factor if layer_height_method == "ball_finish" else 1.0,
        "ld_warning": ld_warning,
        "unit_system": unit_system,
        **power,
    }

    # ── Update display cache ───────────────────────────────────────
    _update_display_cache(result)

    return result


# ── Display helpers ─────────────────────────────────────────────────────

def _fmt_len(mm_val, unit_system):
    """Format a length value for display in the user's unit system.

    Returns e.g. '0.0591"' for INCHES or '1.500mm' for MILLIMETERS,
    with the alternate unit in parentheses.
    """
    in_val = mm_val / 25.4
    if unit_system == "INCHES":
        return f'{in_val:.4f}" ({mm_val:.3f}mm)'
    else:
        return f'{mm_val:.3f}mm ({in_val:.4f}")'


def _fmt_feed(mm_min_val, unit_system):
    """Format a feedrate for display."""
    in_min_val = mm_min_val / 25.4
    if unit_system == "INCHES":
        return f"{in_min_val:.1f} in/min ({mm_min_val:.0f} mm/min)"
    else:
        return f"{mm_min_val:.0f} mm/min ({in_min_val:.1f} in/min)"


def _fmt_cl(mm_val, unit_system):
    """Format a chip load for display."""
    in_val = mm_val / 25.4
    if unit_system == "INCHES":
        return f'{in_val:.5f}"/tooth ({mm_val:.4f} mm/tooth)'
    else:
        return f'{mm_val:.4f} mm/tooth ({in_val:.5f}"/tooth)'


def _update_display_cache(params):
    """Build display lines and warning flags from calculation result."""
    global _last_calc_lines, _last_calc_warnings
    p = params
    D = p["cutter_diameter_mm"]
    R = D / 2.0
    D_in = D / 25.4
    ae = p["stepover_mm"]
    lh = p["layer_height_mm"]
    u = p["unit_system"]
    op_type = "Roughing" if p["is_roughing"] else "Finishing"

    lines = []
    warnings = {
        "power": p.get("power_warning", False) and not p.get("feedrate_power_limited", False),
        "engagement": p.get("engagement_warning", False),
        "ld_ratio": p.get("ld_warning", False),
    }

    # Tool info — show diameter in user's preferred unit first
    ball_R = p.get("ball_radius_mm", D / 2.0)
    ball_R_in = ball_R / 25.4
    is_ballcone = p["cutter_type"] == "BALLCONE"

    if u == "INCHES":
        dia_str = f'\u00d8{D_in:.4f}" ({D:.2f}mm)'
        len_str = f'L={p["cutter_length_mm"] / 25.4:.3f}" ({p["cutter_length_mm"]:.1f}mm)'
    else:
        dia_str = f'\u00d8{D:.2f}mm ({D_in:.4f}")'
        len_str = f'L={p["cutter_length_mm"]:.1f}mm ({p["cutter_length_mm"] / 25.4:.3f}")'

    shank_D = p["shank_diameter_mm"]
    shank_in = shank_D / 25.4
    if u == "INCHES":
        shank_str = f'{shank_in:.4f}" ({shank_D:.2f}mm)'
    else:
        shank_str = f'{shank_D:.2f}mm ({shank_in:.4f}")'

    if is_ballcone:
        lines.append(f"Tool: {p['cutter_type']}, {p['cutter_flutes']}fl")
    else:
        lines.append(f"Tool: {p['cutter_type']} {dia_str}, {p['cutter_flutes']}fl")
    if is_ballcone:
        if u == "INCHES":
            lines.append(f"  Ball Radius: {ball_R_in:.4f}\" ({ball_R:.3f}mm)")
        else:
            lines.append(f"  Ball Radius: {ball_R:.3f}mm ({ball_R_in:.4f}\")")
    lines.append(f"  Shank: {shank_str}")
    lines.append(
        f"  {len_str} (Length/Shank={p['ld_ratio']:.1f})"
    )
    lines.append(
        f"  Deflection factor: {p['tool_length_factor']:.2f}"
        f" (1.0=rigid, 0.4=max flex)"
    )
    if p["ld_warning"]:
        lines.append(f"WARNING: High stick-out/shank ratio ({p['ld_ratio']:.1f}) \u2014 tool deflection risk")

    # Material & factors
    lines.append(
        f"Material: {p['material']} ({p['tool_material']})"
    )
    lines.append(
        f"Machine Stiffness: {p['stiffness']}/10"
    )
    lines.append(
        f"Effective factor: {p['stiffness_factor']:.2f} x {p['tool_length_factor']:.2f} "
        f"= {p['effective_factor']:.2f}"
    )
    lines.append(f"Operation: {op_type}")

    # RPM
    lines.append("── RPM ──")
    if p["auto_calc_rpm"]:
        actual = p["spindle_rpm"]
        ideal = p["ideal_rpm"]
        use_max = p.get("use_max_rpm", False)
        ss = p["surface_speed"]
        sfm = ss * 3.2808  # m/min → surface feet/min
        if u == "INCHES":
            ss_str = f"Cutter surface speed: {sfm:.0f} SFM ({ss:.0f} m/min)"
        else:
            ss_str = f"Cutter surface speed: {ss:.0f} m/min ({sfm:.0f} SFM)"
        if use_max:
            lines.append(f"RPM = {actual:.0f} (max — best for wood)")
            lines.append(ss_str)
        elif p["rpm_clamped"]:
            lines.append(
                f"{ss_str} ({p['material']}, {p['tool_material']})"
            )
            lines.append(f"Ideal RPM: {ideal:.0f}")
            if actual > ideal:
                lines.append(
                    f"RPM = {actual:.0f} (machine minimum)"
                )
            else:
                lines.append(
                    f"RPM = {actual:.0f} (machine maximum)"
                )
        else:
            lines.append(
                f"{ss_str} ({p['material']}, {p['tool_material']})"
            )
            lines.append(f"RPM = {actual:.0f}")
    else:
        lines.append(f"RPM = {p['spindle_rpm']:.0f} (manual)")

    # Stepover
    lines.append("── Stepover ──")
    method = p.get("stepover_method", "")
    frac = p.get("stepover_frac", 0)
    if method == "ball_finishing":
        ball_R = p.get("ball_radius_mm", p["cutter_diameter_mm"] / 2.0)
        r_ratio = ae / ball_R if ball_R > 0 else 0
        lines.append(
            f"Ball finishing: R/{1/r_ratio:.0f} ({p['surface_finish']})"
            if r_ratio > 0 else "Ball finishing stepover"
        )
    elif method == "ball_roughing":
        # Show as fraction of R for ball cutters
        ball_R = p.get("ball_radius_mm", p["cutter_diameter_mm"] / 2.0)
        r_ratio = ae / ball_R if ball_R > 0 else 0
        lines.append(
            f"Ball roughing: R/{1/r_ratio:.0f}"
            f" \u00d7 {p['effective_factor']:.2f} eff. factor"
            if r_ratio > 0 else "Ball roughing stepover"
        )
    elif method == "roughing":
        lines.append(
            f"Roughing stepover: {frac:.0%} of \u00d8"
            f" \u00d7 {p['effective_factor']:.2f} eff. factor"
        )
    elif method == "finishing":
        lines.append(
            f"Finishing stepover: {frac:.0%} of \u00d8"
            f" \u00d7 {p['effective_factor']:.2f} eff. factor"
        )
    lines.append(f"Stepover: {_fmt_len(ae, u)}")

    # Layer Height
    lines.append("── Layer Height ──")
    lh_method = p.get("layer_height_method", "")
    lh_frac = p.get("layer_height_frac", 0)
    if lh_method == "roughing":
        lines.append(
            f"Roughing depth: {lh_frac:.0%} of \u00d8"
            f" \u00d7 {p['effective_factor']:.2f} eff. factor"
        )
    elif lh_method == "ball_finish":
        mf = p.get("layer_height_material_factor", 1.0)
        if abs(mf - 1.0) < 0.01:
            lines.append("75% of tip diameter")
        else:
            lines.append(f"75% of tip \u00d8 \u00d7 {mf:.2f} material")
    elif lh_method == "finishing":
        lines.append(
            f"Finishing depth: {lh_frac:.0%} of \u00d8"
            f" \u00d7 {p['effective_factor']:.2f} eff. factor"
        )
    lines.append(f"Layer Height: {_fmt_len(lh, u)}")

    # Engagement
    lines.append("\u2500\u2500 Engagement \u2500\u2500")
    eng = p["engagement_deg"]
    is_ball_display = p["cutter_type"] in ("BALLNOSE", "BALLCONE")
    if is_ball_display:
        # eng is % of tip diameter
        if eng > 50:
            label = "HEAVY"
        elif eng > 35:
            label = "moderate"
        elif eng > 20:
            label = "light"
        else:
            label = "very light"
        lines.append(f"{eng:.1f}% of tip \u00d8 ({label})")
        if p["engagement_warning"]:
            lines.append(f"WARNING: High engagement ({eng:.1f}% > 50%)")
    else:
        if eng > 90:
            label = "HEAVY"
        elif eng > 60:
            label = "moderate"
        elif eng > 30:
            label = "light"
        else:
            label = "very light"
        lines.append(f"Angle = {eng:.1f}\u00b0 ({label})")
        if p["engagement_warning"]:
            lines.append(f"WARNING: Heavy engagement ({eng:.1f}\u00b0 > 90\u00b0)")

    # Chip Load
    lines.append("\u2500\u2500 Chip Load \u2500\u2500")
    lines.append(f"Optimal: {_fmt_cl(p['base_chip_load'], u)}")
    # Build scaling explanation
    ds = p["dia_scale"]
    ef = p["effective_factor"]
    if abs(ds - 1.0) > 0.01:
        lines.append(
            f"Scaled: {_fmt_cl(p['scaled_chip_load'], u)}"
            f" (diameter x{ds:.2f}, stiffness x{ef:.2f})"
        )
    elif abs(ef - 1.0) > 0.01:
        lines.append(
            f"Scaled: {_fmt_cl(p['scaled_chip_load'], u)}"
            f" (stiffness x{ef:.2f})"
        )
    else:
        lines.append(f"Scaled: {_fmt_cl(p['scaled_chip_load'], u)}")

    # Feedrate
    lines.append("── Feedrate ──")
    lines.append(f"Optimal: {_fmt_feed(p['base_feedrate_mm'], u)}")
    if p["ctf"] > 1.0:
        lines.append(f"CTF = {p['ctf']:.2f} (Chip Thinning Factor)")
        lines.append(
            f"Increase feedrate by {p['ctf']:.2f}: stepover < 50% of cutter dia.)"
        )
    if p["feedrate_power_limited"]:
        if p["feedrate_was_clamped"]:
            lines.append(
                f"Adjusted: {_fmt_feed(p['pre_power_feedrate_mm'], u)}"
                " (machine maximum)"
            )
        else:
            lines.append(
                f"Adjusted: {_fmt_feed(p['pre_power_feedrate_mm'], u)}"
            )
        lines.append(
            f"Power limited: {_fmt_feed(p['adjusted_feedrate_mm'], u)}"
            f" (reduced for {p['watts']:.0f}W spindle)"
        )
    elif p["feedrate_was_clamped"]:
        lines.append(
            f"Adjusted: {_fmt_feed(p['adjusted_feedrate_mm'], u)}"
            " (machine maximum)"
        )
    else:
        lines.append(f"Adjusted: {_fmt_feed(p['adjusted_feedrate_mm'], u)}")

    # Plunge Rate
    lines.append("── Plunge Rate ──")
    lines.append(
        f"{_fmt_feed(p['adjusted_feedrate_mm'], u).split(' (')[0]} x "
        f"{p['plunge_rate_factor']:.2f} = "
        f"{_fmt_feed(p['plunge_rate_mm'], u)}"
    )

    # Power
    lines.append("── Power ──")
    mrr_in3 = p["mrr_mm3"] / 16387.064
    lines.append(
        f"Material Removal Rate = {p['mrr_mm3']:.0f} mm\u00b3/min ({mrr_in3:.4f} in\u00b3/min)"
    )
    lines.append(
        f"Power Constant = {p['power_c']:.2f}, "
        f"Material Power Factor = {p['Kp']:.2f}"
    )
    lines.append(
        f"Power at Motor = {p['Pm_kw']:.3f} kW ({p['watts']:.0f}W, {p['hp']:.2f} HP)"
    )
    if p.get("feedrate_power_limited"):
        lines.append(f"Feedrate reduced to stay within {p['watts']:.0f}W spindle")
    elif p["power_warning"]:
        lines.append(f"WARNING: {p['power_warning_msg']}")
    else:
        lines.append(f"Within spindle rating")

    _last_calc_lines = lines
    _last_calc_warnings = warnings


def log_cutting_params(params):
    """Log a calculation result in readable format."""
    p = params
    D = p["cutter_diameter_mm"]
    R = D / 2.0
    D_in = D / 25.4
    L = p["cutter_length_mm"]
    op_type = "Roughing" if p["is_roughing"] else "Finishing"
    ae = p["stepover_mm"]
    lh = p["layer_height_mm"]

    log.info("[Calc] \u2550\u2550\u2550 Optimal Cutting Parameters \u2550\u2550\u2550")
    log.info(
        f"[Calc]   Tool: {p['cutter_type']} "
        f"\u00d8{D:.3f}mm ({D_in:.4f}\"), {p['cutter_flutes']} flutes, "
        f"L={L:.1f}mm (L/D={p['ld_ratio']:.1f}, TLF={p['tool_length_factor']:.2f})"
    )
    if p["ld_warning"]:
        log.info(f"[Calc]   \u26a0 Tool deflection warning: L/D={p['ld_ratio']:.1f} > 5.0")
    log.info(
        f"[Calc]   Material: {p['material']} ({p['tool_material']}), "
        f"Stiffness: {p['stiffness']}/10 (factor: {p['stiffness_factor']:.2f})"
    )
    log.info(
        f"[Calc]   Effective factor: {p['stiffness_factor']:.2f} "
        f"\u00d7 {p['tool_length_factor']:.2f} = {p['effective_factor']:.2f}"
    )
    log.info(f"[Calc]   Operation: {op_type}")

    # RPM
    log.info("[Calc] \u2500\u2500\u2500 RPM \u2500\u2500\u2500")
    if p["auto_calc_rpm"]:
        log.info(
            f"[Calc]   Cutter surface speed: {p['surface_speed']} m/min "
            f"({p['material']}, {p['tool_material']})"
        )
        log.info(
            f"[Calc]   RPM = (1000 \u00d7 {p['surface_speed']}) / "
            f"(\u03c0 \u00d7 {D:.3f}) = {p['spindle_rpm']:.0f}"
        )
        if p["rpm_clamped"]:
            log.info(f"[Calc]   \u26a0 {p['rpm_clamp_reason']}")
    else:
        log.info(f"[Calc]   RPM: {p['spindle_rpm']:.0f} (manual/default)")

    # Stepover
    log.info("[Calc] \u2500\u2500\u2500 Stepover \u2500\u2500\u2500")
    if p["stepover_method"].startswith("ball_"):
        ball_R = p.get("ball_radius_mm", D / 2.0)
        r_ratio = ae / ball_R if ball_R > 0 else 0
        label = "roughing" if p["is_roughing"] else "finishing"
        r_denom = f"R/{1/r_ratio:.0f}" if r_ratio > 0 else "?"
        log.info(f"[Calc]   Ball {label}: {r_denom} ({p['surface_finish']})")
        log.info(f"[Calc]   ae = {ae:.4f}mm ({ae / 25.4:.5f}\")")
    else:
        log.info(f"[Calc]   ae = {ae:.4f}mm ({ae / 25.4:.5f}\")")

    # Engagement
    log.info("[Calc] \u2500\u2500\u2500 Engagement \u2500\u2500\u2500")
    eng = p["engagement_deg"]
    is_ball_log = p["cutter_type"] in ("BALLNOSE", "BALLCONE")
    if is_ball_log:
        log.info(f"[Calc]   {eng:.1f}% of tip \u00d8")
    else:
        log.info(f"[Calc]   angle = {eng:.1f}\u00b0")

    # Chip Load
    log.info("[Calc] \u2500\u2500\u2500 Chip Load \u2500\u2500\u2500")
    log.info(
        f"[Calc]   Base: {p['base_chip_load']:.4f} mm/tooth "
        f"({p['material']}, 3.175mm ref)"
    )
    log.info(
        f"[Calc]   Scaled: {p['base_chip_load']:.4f} \u00d7 "
        f"{sqrt(D / _REF_DIAMETER) if D > 0 else 0:.3f} \u00d7 {p['effective_factor']:.2f} "
        f"= {p['scaled_chip_load']:.4f} mm/tooth"
    )

    # Feedrate
    log.info("[Calc] \u2500\u2500\u2500 Feedrate \u2500\u2500\u2500")
    log.info(
        f"[Calc]   Base: {p['scaled_chip_load']:.4f} \u00d7 "
        f"{p['cutter_flutes']} flutes \u00d7 {p['spindle_rpm']:.0f} RPM "
        f"= {p['base_feedrate_mm']:.0f} mm/min"
    )
    if p["ctf"] > 1.0:
        log.info(
            f"[Calc]   Chip thinning: ae={ae:.4f} < D/2={R:.3f} "
            f"\u2192 CTF = {p['ctf']:.2f}"
        )
        log.info(
            f"[Calc]   Adjusted: {p['base_feedrate_mm']:.0f} \u00d7 {p['ctf']:.2f} "
            f"= {p['adjusted_feedrate_mm']:.0f} mm/min"
        )
    else:
        log.info(f"[Calc]   No chip thinning (ae \u2265 D/2)")
    if p["feedrate_was_clamped"]:
        log.info(f"[Calc]   \u26a0 Feedrate clamped to machine maximum")

    # Plunge Rate
    log.info("[Calc] \u2500\u2500\u2500 Plunge Rate \u2500\u2500\u2500")
    log.info(
        f"[Calc]   {p['adjusted_feedrate_mm']:.0f} \u00d7 {p['plunge_rate_factor']:.2f} "
        f"= {p['plunge_rate_mm']:.0f} mm/min"
    )

    # Layer Height
    log.info("[Calc] \u2500\u2500\u2500 Layer Height \u2500\u2500\u2500")
    log.info(f"[Calc]   ap = {lh:.4f}mm ({lh / 25.4:.5f}\")")

    # Power
    log.info("[Calc] \u2500\u2500\u2500 Power \u2500\u2500\u2500")
    mrr_in3 = p["mrr_mm3"] / 16387.064
    log.info(
        f"[Calc]   MRR = {p['mrr_mm3']:.0f} mm\u00b3/min "
        f"({mrr_in3:.4f} in\u00b3/min)"
    )
    log.info(
        f"[Calc]   C = {p['power_c']:.2f} (chipload {p['scaled_chip_load']:.4f} mm/tooth)"
    )
    log.info(
        f"[Calc]   Pc = {p['Kp']:.2f} \u00d7 {p['power_c']:.2f} \u00d7 "
        f"{p['Q_cm3s']:.4f} \u00d7 {_TOOL_WEAR_FACTOR} = {p['Pc_kw']:.3f} kW"
    )
    log.info(
        f"[Calc]   Pm = {p['Pc_kw']:.3f} / {_MACHINE_EFFICIENCY} "
        f"= {p['Pm_kw']:.3f} kW ({p['watts']:.0f}W, {p['hp']:.2f} HP)"
    )
    if p["power_warning"]:
        log.info(f"[Calc]   \u26a0 POWER WARNING: {p['power_warning_msg']}")
        log.info(f"[Calc]   Consider: reduce stepover, layer height, or feedrate")
    else:
        log.info(f"[Calc]   \u2713 Within spindle rating")

    log.info("[Calc] \u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550"
             "\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550"
             "\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550"
             "\u2550\u2550\u2550\u2550\u2550")


_auto_calc_running = False
_auto_calc_count = 0


def auto_calculate_for_operation(operation, machine, write_back=True):
    """Run auto-calculation for an operation using machine settings.

    Reads cutter info from the operation, material/stiffness from machine,
    calculates optimal values, and optionally applies stepover, layer height,
    RPM, and/or feedrate if the corresponding auto_calc toggles are enabled.

    Args:
        write_back: If True, write calculated values to the operation
            properties (stepover, layer height, RPM, feedrate).
            If False, only update the display cache and log — used when
            switching between operations to refresh Calculation Details
            without overwriting the operation's existing values.

    Unit handling:
        Blender stores all unit="LENGTH" properties in METERS internally.
        The calculation engine works in MILLIMETERS.
        Machine feedrate properties (no unit="LENGTH") are in user display
        units (in/min or mm/min depending on unit_system).
        Conversions happen here — nowhere else.
    """
    global _auto_calc_running, _auto_calc_count
    if _auto_calc_running:
        return None
    _auto_calc_running = True
    _auto_calc_count += 1
    import traceback
    caller = traceback.extract_stack(limit=3)[0]
    log.info(
        f"[AutoCalc] #{_auto_calc_count} write_back={write_back} "
        f"from {caller.filename.split('fabex')[-1]}:{caller.lineno}"
    )
    try:
        return _auto_calculate_impl(operation, machine, write_back)
    finally:
        _auto_calc_running = False


def _auto_calculate_impl(operation, machine, write_back):
    m = machine

    # ── Unit conversion constants ──────────────────────────────────
    _M_TO_MM = 1000.0       # Blender meters → mm
    _MM_TO_M = 0.001        # mm → Blender meters

    # Gather inputs (operation props with unit="LENGTH" are in meters)
    cutter_type = operation.cutter_type
    D_mm = operation.cutter_diameter * _M_TO_MM
    flutes = operation.cutter_flutes
    cutter_length_mm = operation.cutter_length * _M_TO_MM
    # Sanity clamp: old files had 25m stick-out; Blender clamps to property max (152.4mm/6")
    # Treat anything >= 152mm (6") as uninitialized and reset to 25.4mm (1")
    if cutter_length_mm >= 152.0:
        cutter_length_mm = 25.4
        operation.cutter_length = cutter_length_mm * _MM_TO_M
    shank_mm = getattr(operation, "shank_diameter", 0) * _M_TO_MM
    # Default shank: >= 3mm cutter uses cutter dia, < 3mm uses 3mm
    if shank_mm <= 0 or abs(shank_mm - 3.175) < 0.01:
        shank_mm = D_mm if D_mm >= 3.0 else 3.0
    rpm = operation.spindle_rpm
    is_roughing = operation.skin > 0

    # Machine feedrate_max has no unit="LENGTH", stored in user display units
    if m.unit_system == "INCHES":
        feedrate_max_mm = m.feedrate_max * 25.4   # in/min → mm/min
    else:
        feedrate_max_mm = m.feedrate_max           # already mm/min

    # Ball radius for BALLCONE cutters (ball tip smaller than body diameter)
    ball_r_mm = getattr(operation, "ball_radius", 0) * _M_TO_MM

    params = calculate_cutting_params(
        cutter_type=cutter_type,
        cutter_diameter_mm=D_mm,
        cutter_flutes=flutes,
        cutter_length_mm=cutter_length_mm,
        material=m.stock_material,
        stiffness=m.machine_stiffness,
        spindle_rpm=rpm,
        is_roughing=is_roughing,
        surface_finish=m.surface_finish_quality,
        feedrate_max_mm=feedrate_max_mm,
        tool_material=m.tool_material,
        spindle_min=m.spindle_min,
        spindle_max=m.spindle_max,
        auto_calc_rpm=m.auto_calc_rpm,
        plunge_rate_factor=m.plunge_rate_factor,
        spindle_power_watts=m.spindle_power_watts,
        unit_system=m.unit_system,
        shank_diameter_mm=shank_mm,
        ball_radius_mm=ball_r_mm,
    )

    # ── Apply results (convert mm back to meters for Blender) ──────
    if write_back:
        if m.auto_calc_stepover:
            operation.distance_between_paths = params["stepover_mm"] * _MM_TO_M

        if m.auto_calc_layer_height:
            operation.stepdown = params["layer_height_mm"] * _MM_TO_M

        if m.auto_calc_rpm:
            operation.spindle_rpm = params["spindle_rpm"]

        if m.auto_calc_feedrate:
            operation.feedrate = params["adjusted_feedrate_mm"] * _MM_TO_M
            operation.plunge_feedrate = params["plunge_rate_factor"] * 100.0

    # Log the calculation
    log_cutting_params(params)

    return params
