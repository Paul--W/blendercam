"""
Verify Fabex BALLCONE cutter geometry against analytic ground truth and OCL.

Convention used by Fabex `get_cutter_array` (BALLNOSE, verified):
    z = sqrt(R^2 - r^2) - R       (z=0 at axis, z=-R at sphere equator)
    => z is the negative of "height above tip" (sphere oriented as if center is at
       z=0 and the tip is below... but offset_image uses np.nanmax(source + car)
       so car is consistently subtractive of available height).

We only need to compare OLD-code vs NEW-code vs analytic-truth on the SAME
sign convention. We pick "z positive going up, tip at z=0, sphere center at
z=R above tip" as ground truth, and flip Fabex's signs to compare.

cutter_tip_angle is in DEGREES, full-included angle (per VCARVE in same file).
half-angle from axis: alpha = radians(cutter_tip_angle) / 2.
"""

from math import sin, cos, tan, radians, sqrt, acos, pi
import numpy as np


def fabex_old_car_value(r, ball_r, cutter_r, cutter_tip_angle_deg):
    """OLD code from operation_utils.py:790 (pre-fix)."""
    angle = radians(cutter_tip_angle_deg) / 2
    if r > cutter_r:
        return None
    conedepth = (cutter_r - ball_r) / tan(angle)
    Ball_R = ball_r / cos(angle)
    D_ofset = ball_r * tan(angle)
    s = tan(pi / 2 - angle)
    z = -(r - ball_r) * s - Ball_R + D_ofset
    if r <= ball_r:
        z = sin(acos(r / Ball_R)) * Ball_R - Ball_R
    return z


def fabex_new_car_value(r, ball_r, cutter_r, cutter_tip_angle_deg):
    """NEW code (current working tree, the 'fix')."""
    angle = radians(cutter_tip_angle_deg) / 2
    if r > cutter_r:
        return None
    r_tangent = ball_r * sin(angle)
    z_tangent = -ball_r * (1.0 - cos(angle))
    cone_slope = 1.0 / tan(angle)
    if r <= r_tangent:
        z = sqrt(ball_r * ball_r - r * r) - ball_r
    else:
        z = z_tangent - (r - r_tangent) * cone_slope
    return z


def truth_car_value_R_eq_ball_r(r, ball_r, cutter_r, cutter_tip_angle_deg):
    """Analytic truth assuming `ball_radius` IS the actual sphere radius
    (matching OCL's ball-diameter convention).

    Sphere of radius R=ball_r tangent to cone of half-angle alpha (from axis).
    Tip-zeroed (tip at z=0, sphere center at z=R above tip).
    Tangent point: (R*cos(alpha), R*(1-sin(alpha))).
    Cone: z = z_tangent + (r - r_tangent)/tan(alpha).
    Then NEGATED to match Fabex's car sign convention.
    """
    alpha = radians(cutter_tip_angle_deg) / 2
    R = ball_r
    if r > cutter_r:
        return None
    r_tangent = R * cos(alpha)
    z_tangent_phys = R * (1.0 - sin(alpha))
    if r <= r_tangent:
        z_phys = R - sqrt(R * R - r * r)
    else:
        z_phys = z_tangent_phys + (r - r_tangent) / tan(alpha)
    return -z_phys  # flip to Fabex's car sign convention


def truth_car_value_ball_r_is_tangent_radius(r, ball_r, cutter_r, cutter_tip_angle_deg):
    """Analytic truth assuming `ball_radius` is the TANGENT radius (the OLD
    code's implicit interpretation): actual sphere radius is ball_r/cos(alpha).
    Tip-zeroed and sign-flipped to Fabex car convention.
    """
    alpha = radians(cutter_tip_angle_deg) / 2
    R = ball_r / cos(alpha)
    if r > cutter_r:
        return None
    r_tangent = R * cos(alpha)  # = ball_r
    z_tangent_phys = R * (1.0 - sin(alpha))
    if r <= r_tangent:
        z_phys = R - sqrt(R * R - r * r)
    else:
        z_phys = z_tangent_phys + (r - r_tangent) / tan(alpha)
    return -z_phys


def ocl_ballcone_height_at_r(r, ball_diam, cutter_diam, alpha_axis):
    """Replica of OCL BallConeCutter::height(r), per compositecutter.cpp:328.
    alpha_axis is the cone half-angle measured from the AXIS (radians).
    Returns the cutter surface height above the tip (POSITIVE going up).
    """
    radius1 = ball_diam / 2.0
    radius2 = cutter_diam / 2.0
    rcontact = radius1 * cos(alpha_axis)  # tangent radius
    height1 = radius1 - sqrt(radius1 * radius1 - rcontact * rcontact)
    cone_offset = -((rcontact / tan(alpha_axis)) - height1)
    if r > radius2:
        return None
    if r <= rcontact:
        # ball region
        return radius1 - sqrt(radius1 * radius1 - r * r)
    else:
        # cone region: ConeCutter height(r) = r/tan(alpha_axis), with cone_offset
        return r / tan(alpha_axis) + cone_offset


def compare_at_angles(test_cases):
    print(f"{'Tool':<32} {'r/R':<6} {'OLD':<10} {'NEW':<10} {'truth_R=ball':<12} {'truth_R=tan':<12} {'OCL':<10}")
    print("-" * 100)
    for label, ball_r, cutter_r, angle_deg in test_cases:
        # Sample at several radial positions
        sample_radii = [0.0, 0.25 * ball_r, 0.5 * ball_r, 0.75 * ball_r, ball_r,
                        1.5 * ball_r if 1.5 * ball_r <= cutter_r else cutter_r,
                        cutter_r]
        sample_radii = sorted(set(sample_radii))
        alpha_axis = radians(angle_deg) / 2
        for r in sample_radii:
            old = fabex_old_car_value(r, ball_r, cutter_r, angle_deg)
            new = fabex_new_car_value(r, ball_r, cutter_r, angle_deg)
            t_sphere = truth_car_value_R_eq_ball_r(r, ball_r, cutter_r, angle_deg)
            t_tan = truth_car_value_ball_r_is_tangent_radius(r, ball_r, cutter_r, angle_deg)
            ocl_phys = ocl_ballcone_height_at_r(r, 2 * ball_r, 2 * cutter_r, alpha_axis)
            ocl = -ocl_phys if ocl_phys is not None else None
            r_norm = r / ball_r
            def fmt(v):
                if v is None:
                    return "  -      "
                return f"{v:>+9.5f}"
            print(f"{label:<32} {r_norm:<6.3f} {fmt(old)} {fmt(new)} {fmt(t_sphere)}   {fmt(t_tan)}   {fmt(ocl)}")
        print()


def rms_errors(test_cases, n=200):
    print(f"{'Tool':<32} {'OLD vs sphere':<14} {'NEW vs sphere':<14} {'OLD vs tan':<14} {'NEW vs tan':<14} {'OCL vs sphere':<14}")
    print("-" * 110)
    for label, ball_r, cutter_r, angle_deg in test_cases:
        rs = np.linspace(0.001, cutter_r * 0.999, n)
        old_arr = np.array([fabex_old_car_value(r, ball_r, cutter_r, angle_deg) for r in rs])
        new_arr = np.array([fabex_new_car_value(r, ball_r, cutter_r, angle_deg) for r in rs])
        ts_arr = np.array([truth_car_value_R_eq_ball_r(r, ball_r, cutter_r, angle_deg) for r in rs])
        tt_arr = np.array([truth_car_value_ball_r_is_tangent_radius(r, ball_r, cutter_r, angle_deg) for r in rs])
        alpha_axis = radians(angle_deg) / 2
        ocl_arr = np.array([-ocl_ballcone_height_at_r(r, 2 * ball_r, 2 * cutter_r, alpha_axis) for r in rs])
        rms_old_s = np.sqrt(np.mean((old_arr - ts_arr) ** 2))
        rms_new_s = np.sqrt(np.mean((new_arr - ts_arr) ** 2))
        rms_old_t = np.sqrt(np.mean((old_arr - tt_arr) ** 2))
        rms_new_t = np.sqrt(np.mean((new_arr - tt_arr) ** 2))
        rms_ocl_s = np.sqrt(np.mean((ocl_arr - ts_arr) ** 2))
        print(f"{label:<32} {rms_old_s:>12.5f}   {rms_new_s:>12.5f}   {rms_old_t:>12.5f}   {rms_new_t:>12.5f}   {rms_ocl_s:>12.5f}")


if __name__ == "__main__":
    # Production ballcone cutters live in the 6-20 deg included-angle range.
    # Largest tool found on Amazon: 15.2 deg included.
    # So we evaluate only in that production range.
    # Test cases: (label, ball_radius_mm, cutter_radius_mm, cutter_tip_angle_deg)
    cases = [
        ("20deg, R=0.5, cR=3.175",         0.5,  3.175, 20.0),
        ("15.2deg (Amazon largest)",       0.5,  3.175, 15.2),
        ("12deg, R=0.25, cR=3.175",        0.25, 3.175, 12.0),
        ("10deg, R=0.5, cR=3.175",         0.5,  3.175, 10.0),
        ("8deg, R=0.25, cR=3.175",         0.25, 3.175, 8.0),
        ("6deg, R=0.1,  cR=3.175",         0.1,  3.175, 6.0),
    ]
    print("=== Pointwise sample values (sign-flipped to Fabex car convention) ===\n")
    compare_at_angles(cases)
    print("\n=== RMS errors over r in (0, cutter_r) ===\n")
    rms_errors(cases)
    print("\n=== Max abs error over r in (0, cutter_r) ===\n")
    print(f"{'Tool':<32} {'OLD vs OCL (mm)':<18} {'OLD vs OCL (um)':<18}")
    print("-" * 70)
    for label, ball_r, cutter_r, angle_deg in cases:
        rs = np.linspace(0.001, cutter_r * 0.999, 500)
        old_arr = np.array([fabex_old_car_value(r, ball_r, cutter_r, angle_deg) for r in rs])
        alpha_axis = radians(angle_deg) / 2
        ocl_arr = np.array([-ocl_ballcone_height_at_r(r, 2 * ball_r, 2 * cutter_r, alpha_axis) for r in rs])
        max_err_mm = float(np.max(np.abs(old_arr - ocl_arr)))
        print(f"{label:<32} {max_err_mm:<18.6f} {max_err_mm*1000:<18.2f}")
