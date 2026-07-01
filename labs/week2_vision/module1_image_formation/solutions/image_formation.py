"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

This is a CONCEPT lab — it does not need the simulator.
Run it directly:
    python3 image_formation.py
It prints PASS/FAIL for each question's self-check.
"""

import numpy as np


# ── Q1: Perspective Projection ──────────────────────────────────────────────────────
def project_perspective(point_cam, f):
    """
    Project a 3D point expressed in CAMERA coordinates onto the image plane.

    Pinhole model:  x = f * X / Z ,  y = f * Y / Z
    Args:  point_cam = (X, Y, Z) in meters, f = focal length in meters.
    Returns: (x, y) image-plane coordinates in meters.
    """
    X, Y, Z = point_cam
    x = f * X / Z
    y = f * Y / Z
    return (x, y)


# ── Q2: Conversion to Pixels ────────────────────────────────────────────────────────
def meters_to_pixels(x, y, pixel_size, principal_point):
    """
    Convert image-plane coordinates (meters) to pixel coordinates.

    u = x / pixel_size + cx ,  v = y / pixel_size + cy
    Args:  pixel_size = width of one pixel in meters, principal_point = (cx, cy).
    Returns: (u, v) in pixels.
    """
    cx, cy = principal_point
    u = x / pixel_size + cx
    v = y / pixel_size + cy
    return (u, v)


# ── Q3: Intrinsic Matrix ────────────────────────────────────────────────────────────
def intrinsic_matrix(fx, fy, cx, cy):
    """
    Build the 3x3 camera intrinsic matrix K.

        [ fx  0  cx ]
        [  0 fy  cy ]
        [  0  0   1 ]
    """
    return np.array([[fx, 0.0, cx],
                     [0.0, fy, cy],
                     [0.0, 0.0, 1.0]])


# ── Q4: Point Projection with Known Pose ────────────────────────────────────────────
def project_world_point(K, R, t, point_world):
    """
    Project a 3D WORLD point to pixels given the camera pose.

        p_cam   = R @ point_world + t      (world -> camera)
        p_homog = K @ p_cam                (camera -> image, homogeneous)
        (u, v)  = (p_homog[0]/p_homog[2], p_homog[1]/p_homog[2])
    Returns: (u, v) in pixels.
    """
    p_cam = R @ np.asarray(point_world, dtype=float) + np.asarray(t, dtype=float)
    p_homog = K @ p_cam
    u = p_homog[0] / p_homog[2]
    v = p_homog[1] / p_homog[2]
    return (u, v)


# ── Q5: Radial Distortion ───────────────────────────────────────────────────────────
def apply_radial_distortion(x, y, k1, k2):
    """
    Apply radial (barrel/pincushion) distortion to a normalized image point.

        r^2    = x^2 + y^2
        factor = 1 + k1*r^2 + k2*r^4
        x_d    = x * factor ,  y_d = y * factor
    Returns: (x_d, y_d).
    """
    r2 = x * x + y * y
    factor = 1.0 + k1 * r2 + k2 * r2 * r2
    return (x * factor, y * factor)


# ── Self-check ──────────────────────────────────────────────────────────────────────
def _check():
    passed = 0
    total = 0

    def expect(name, got, want):
        nonlocal passed, total
        total += 1
        ok = np.allclose(np.asarray(got, dtype=float), np.asarray(want, dtype=float),
                         atol=1e-6)
        passed += ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {np.round(got, 4)}")

    # Q1: a point 2m right, 1m up, 4m forward, f=0.05 -> (0.025, 0.0125)
    expect("Q1 project_perspective", project_perspective((2.0, 1.0, 4.0), 0.05),
           (0.025, 0.0125))
    # Q2: x=0.025m, pixel 5e-5 m, cx=320 -> u = 500 + 320 = 820
    expect("Q2 meters_to_pixels", meters_to_pixels(0.025, 0.0125, 5e-5, (320, 240)),
           (820.0, 490.0))
    # Q3 intrinsic matrix
    expect("Q3 intrinsic_matrix", intrinsic_matrix(600, 600, 320, 240),
           [[600, 0, 320], [0, 600, 240], [0, 0, 1]])
    # Q4: identity pose, point on optical axis at Z=2 -> principal point
    K = intrinsic_matrix(600, 600, 320, 240)
    expect("Q4 project_world_point (axis)",
           project_world_point(K, np.eye(3), np.array([0.0, 0.0, 2.0]),
                               np.array([0.0, 0.0, 0.0])),
           (320.0, 240.0))
    expect("Q4 project_world_point (offset)",
           project_world_point(K, np.eye(3), np.array([0.0, 0.0, 0.0]),
                               np.array([1.0, 0.5, 2.0])),
           (620.0, 390.0))
    # Q5: zero distortion leaves the point unchanged
    expect("Q5 radial (k=0)", apply_radial_distortion(0.3, -0.2, 0.0, 0.0), (0.3, -0.2))
    # Q5: r^2 = 0.25, k1=0.1 -> factor 1.025
    expect("Q5 radial (k1=0.1)", apply_radial_distortion(0.3, -0.4, 0.1, 0.0),
           (0.3 * 1.025, -0.4 * 1.025))

    print(f"\n{passed}/{total} checks passed.")
    return passed == total


if __name__ == "__main__":
    print("Week 2 · Module 1 — Image Formation (SOLUTION)\n")
    _check()
