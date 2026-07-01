"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

This is a CONCEPT lab — it does not need the simulator.
Fill in the five functions below, then run it directly:
    python3 image_formation.py
It prints PASS/FAIL for each question's self-check.

A completed reference lives in ../solutions/image_formation.py
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
    ##################################
    #### START PUT CODE HERE #########
    # Apply the pinhole equations and return (x, y).
    x = 0.0  # YOUR CODE HERE
    y = 0.0  # YOUR CODE HERE
    ###### END PUT CODE HERE #########
    ##################################
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
    ##################################
    #### START PUT CODE HERE #########
    u = 0.0  # YOUR CODE HERE
    v = 0.0  # YOUR CODE HERE
    ###### END PUT CODE HERE #########
    ##################################
    return (u, v)


# ── Q3: Intrinsic Matrix ────────────────────────────────────────────────────────────
def intrinsic_matrix(fx, fy, cx, cy):
    """
    Build the 3x3 camera intrinsic matrix K.

        [ fx  0  cx ]
        [  0 fy  cy ]
        [  0  0   1 ]
    """
    ##################################
    #### START PUT CODE HERE #########
    K = np.eye(3)  # YOUR CODE HERE
    ###### END PUT CODE HERE #########
    ##################################
    return K


# ── Q4: Point Projection with Known Pose ────────────────────────────────────────────
def project_world_point(K, R, t, point_world):
    """
    Project a 3D WORLD point to pixels given the camera pose.

        p_cam   = R @ point_world + t      (world -> camera)
        p_homog = K @ p_cam                (camera -> image, homogeneous)
        (u, v)  = (p_homog[0]/p_homog[2], p_homog[1]/p_homog[2])
    Returns: (u, v) in pixels.
    """
    ##################################
    #### START PUT CODE HERE #########
    u = 0.0  # YOUR CODE HERE
    v = 0.0  # YOUR CODE HERE
    ###### END PUT CODE HERE #########
    ##################################
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
    ##################################
    #### START PUT CODE HERE #########
    factor = 1.0  # YOUR CODE HERE (replace with 1 + k1*r^2 + k2*r^4)
    ###### END PUT CODE HERE #########
    ##################################
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

    expect("Q1 project_perspective", project_perspective((2.0, 1.0, 4.0), 0.05),
           (0.025, 0.0125))
    expect("Q2 meters_to_pixels", meters_to_pixels(0.025, 0.0125, 5e-5, (320, 240)),
           (820.0, 490.0))
    expect("Q3 intrinsic_matrix", intrinsic_matrix(600, 600, 320, 240),
           [[600, 0, 320], [0, 600, 240], [0, 0, 1]])
    K = intrinsic_matrix(600, 600, 320, 240)
    expect("Q4 project_world_point (axis)",
           project_world_point(K, np.eye(3), np.array([0.0, 0.0, 2.0]),
                               np.array([0.0, 0.0, 0.0])),
           (320.0, 240.0))
    expect("Q4 project_world_point (offset)",
           project_world_point(K, np.eye(3), np.array([0.0, 0.0, 0.0]),
                               np.array([1.0, 0.5, 2.0])),
           (620.0, 390.0))
    expect("Q5 radial (k=0)", apply_radial_distortion(0.3, -0.2, 0.0, 0.0), (0.3, -0.2))
    expect("Q5 radial (k1=0.1)", apply_radial_distortion(0.3, -0.4, 0.1, 0.0),
           (0.3 * 1.025, -0.4 * 1.025))

    print(f"\n{passed}/{total} checks passed.")
    return passed == total


if __name__ == "__main__":
    print("Week 2 · Module 1 — Image Formation\n")
    _check()
