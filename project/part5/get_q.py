import numpy as np
import cv2

K = np.array([[9.32543671e+02, 0.0,              3.78228036e+02],
              [0.0,              1.00484929e+03, 2.76842723e+02],
              [0.0,              0.0,             1.0]])

dist_coeffs = np.array([[-0.29927287, -0.38567896, -0.04231542, -0.00156885, 1.00625209]])

def pixel_to_cam_ray(u, v):
    """
    u, v: pixel coordinates
    returns: 3x1 direction vector in camera frame (unit length)
    """
    pts = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)
    undist = cv2.undistortPoints(pts, K, dist_coeffs)   # -> (1,1,2) in normalized image coords

    x_n, y_n = undist[0, 0]
    d_cam = np.array([[x_n], [y_n], [1.0]])  # direction in camera frame

    # normalize (optional but nice)
    d_cam = d_cam / np.linalg.norm(d_cam)
    return d_cam

# === FILL THESE FROM YOUR CALIBRATION ===
R_0C = np.eye(3)  # 3x3
p_0C = np.array([[0.0], [0.0], [0.0]])  # 3x1


def pixel_to_Pd(u, v, Z_base):
    """
    u, v   : pixel coordinates
    Z_base : known Z of the object in base frame (meters)
    returns: 3x1 Pd (position in base frame)
    """
    # Ray in camera frame
    d_cam = pixel_to_cam_ray(u, v)

    # Ray in base frame
    d_base = R_0C @ d_cam       # 3x1
    d_base = d_base / np.linalg.norm(d_base)

    # Camera origin in base frame
    p_cam_base = p_0C           # 3x1

    # Avoid divide-by-zero if ray parallel to plane
    if abs(d_base[2, 0]) < 1e-6:
        raise ValueError("Ray is (almost) parallel to Z = const plane in base frame")

    t = (Z_base - p_cam_base[2, 0]) / d_base[2, 0]

    Pd = p_cam_base + t * d_base   # 3x1
    return Pd


def main():
    robot = build_robot()  # your function that constructs rox.Robot

    # Desired orientation (could be fixed for now)
    Rd = np.eye(3)

    # Example: pixel of detected object
    u_desired = 400.0
    v_desired = 250.0

    # Known Z in BASE frame (e.g. plane of table)
    Z_base = 0.10  # meters

    Pd = pixel_to_Pd(u_desired, v_desired, Z_base)  # 3x1

    q0 = np.array([25, 50, 75, 30, 30]) * math.pi / 180.0
    tol = np.array([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]]).T
    Nmax = 200
    alpha = 0.1

    q = jacobian_inverse(robot, q0, Rd, Pd, Nmax, alpha, tol)

    q_deg = q * 180.0 / math.pi
    print("q (rad):", q)
    print("q (deg):", q_deg)


def main():
    robot = build_robot()  # your function that constructs rox.Robot

    # Desired orientation (could be fixed for now)
    Rd = np.eye(3)

    # Example: pixel of detected object
    u_desired = 400.0
    v_desired = 250.0

    # Known Z in BASE frame (e.g. plane of table)
    Z_base = 0.10  # meters

    Pd = pixel_to_Pd(u_desired, v_desired, Z_base)  # 3x1

    q0 = np.array([25, 50, 75, 30, 30]) * math.pi / 180.0
    tol = np.array([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]]).T
    Nmax = 200
    alpha = 0.1

    q = jacobian_inverse(robot, q0, Rd, Pd, Nmax, alpha, tol)

    q_deg = q * 180.0 / math.pi
    print("q (rad):", q)
    print("q (deg):", q_deg)


