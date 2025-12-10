import math
import numpy as np
import cv2
import general_robotics_toolbox as rox

# -------------------------------
# Camera intrinsics & distortion
# -------------------------------
K = np.array([
    [9.32543671e+02, 0.0,              3.78228036e+02],
    [0.0,              1.00484929e+03, 2.76842723e+02],
    [0.0,              0.0,             1.0]
])

dist_coeffs = np.array(
    [[-0.29927287, -0.38567896, -0.04231542, -0.00156885, 1.00625209]]
)


def pixel_to_cam_ray(u, v):
    """
    Convert pixel coordinates (u, v) to a unit ray direction in the camera frame.
    """
    pts = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)
    undist = cv2.undistortPoints(pts, K, dist_coeffs)  # -> (1,1,2) normalized coords

    x_n, y_n = undist[0, 0]
    d_cam = np.array([[x_n], [y_n], [1.0]])  # direction in camera frame
    d_cam /= np.linalg.norm(d_cam)
    return d_cam  # 3x1


# -------------------------------
# Robot model: camera at joint 5
# -------------------------------
def build_robot_and_cam_robot():
    """
    Returns:
        robot:     full robot with tool at frame T (after joint 5)
        robot_cam: robot whose tool coincides with frame 5 (camera frame)
    """
    ex = np.array([1, 0, 0])
    ey = np.array([0, 1, 0])
    ez = np.array([0, 0, 1])

    # Link lengths [m]
    l0 = 0.061   # base to servo 1
    l1 = 0.0435  # servo 1 to servo 2
    l2 = 0.08285 # servo 2 to servo 3
    l3 = 0.08285 # servo 3 to servo 4
    l4 = 0.07385 # servo 4 to servo 5
    l5 = 0.05457 # servo 5 to gripper

    # Position vectors (from frame i-1 to i, expressed in frame i-1)
    P01 = (l0 + l1) * ez
    P12 = np.zeros(3,)
    P23 = l2 * ex
    P34 = -l3 * ez
    P45 = np.zeros(3,)
    P5T = -(l4 + l5) * ex

    # Rotation axes
    H = np.array([ez, -1*ey, -1*ey, -1*ey, -1*ex]).T  # shape (3,5)

    # P for full robot (tool frame T)
    P_full = np.array([P01, P12, P23, P34, P45, P5T]).T  # shape (3,6)

    # joint types: 0 = revolute
    joint_type = [0, 0, 0, 0, 0]

    # Robot with EE at tool frame T
    robot = rox.Robot(H, P_full, joint_type)

    # Robot for camera: tool coincides with frame 5
    # -> same H, but last link from 5 to tool is zero
    P_cam = np.array([P01, P12, P23, P34, P45, np.zeros(3,)]).T
    robot_cam = rox.Robot(H, P_cam, joint_type)

    return robot, robot_cam


def get_camera_pose_in_base(robot_cam, q):
    """
    Get camera pose in base frame given current joint angles q.
    Returns:
        R_0C: 3x3 rotation matrix from camera frame to base
        p_0C: 3x1 position of camera origin in base frame
    """
    H_0C = rox.fwdkin(robot_cam, q)
    R_0C = H_0C.R
    p_0C = np.array(H_0C.p).reshape(3, 1)
    return R_0C, p_0C


def pixel_to_Pd_base(u, v, Z_base, robot_cam, q_current):
    """
    Convert a pixel (u, v) and known Z-plane in base frame into a 3x1 target
    position Pd in the robot base frame.

    Args:
        u, v       : pixel coordinates of the object
        Z_base     : known Z coordinate of the object in base frame [m]
        robot_cam  : robot model with camera at tool
        q_current  : current joint angles (5,) in radians

    Returns:
        Pd: 3x1 position of object in base frame
    """
    q_current = np.asarray(q_current).reshape(5,)
    d_cam = pixel_to_cam_ray(u, v)         # 3x1 in camera frame
    R_0C, p_0C = get_camera_pose_in_base(robot_cam, q_current)

    # Ray direction in base frame
    d_base = R_0C @ d_cam
    d_base /= np.linalg.norm(d_base)

    # Intersect with plane z = Z_base
    if abs(d_base[2, 0]) < 1e-6:
        raise ValueError("Ray is nearly parallel to Z = const plane in base frame.")

    t = (Z_base - p_0C[2, 0]) / d_base[2, 0]
    Pd = p_0C + t * d_base  # 3x1

    return Pd


# -------------------------------
# Jacobian-based IK
# -------------------------------
def jacobian_inverse(robot, q0, Rd, Pd, Nmax, alpha, tol):
    """
    Iterative Jacobian-based inverse kinematics.

    Args:
        robot : rox.Robot
        q0    : initial guess (5,) in radians
        Rd    : desired 3x3 rotation matrix (base -> EE)
        Pd    : desired 3x1 position (base frame)
        Nmax  : max iterations
        alpha : step size
        tol   : 6x1 tolerances [roll, pitch, yaw, x, y, z]

    Returns:
        q: final joint angles (5,) in radians
    """
    q0 = np.asarray(q0).reshape(-1)
    n = len(q0)

    q = np.zeros((n, Nmax + 1))
    q[:, 0] = q0
    iternum = 0

    # Initial FK
    H = rox.fwdkin(robot, q[:, 0])
    R = H.R
    P = np.array([[H.p[0]], [H.p[1]], [H.p[2]]])

    dR = R @ Rd.T
    r = np.array(rox.R2rpy(dR))[None]  # shape (1,3)
    dX = np.concatenate((r.T, P - Pd))  # shape (6,1)

    # Iterate until all error components are within tolerance
    while (np.abs(dX) > tol).any():
        if iternum < Nmax:
            H = rox.fwdkin(robot, q[:, iternum])
            R = H.R
            P = np.array([[H.p[0]], [H.p[1]], [H.p[2]]])

            dR = R @ Rd.T
            r = np.array(rox.R2rpy(dR))[None]
            dX = np.concatenate((r.T, P - Pd))

            Jq = rox.robotjacobian(robot, q[:, iternum])
            j = np.linalg.pinv(Jq) @ dX  # 5x1

            q[:, iternum + 1] = q[:, iternum] - (alpha * j).T
            iternum += 1
        else:
            print("Warning: IK reached max iterations without converging.")
            break

    return q[:, iternum]


# -------------------------------
# Main: pixel -> q (1x5)
# -------------------------------
def main():
    # Build robot models
    robot, robot_cam = build_robot_and_cam_robot()

    # Current joint angles at moment of image capture (in degrees)
    q0_deg = np.array([25, 50, 75, 30, 30])
    q0 = q0_deg * math.pi / 180.0  # convert to radians

    # Example: pixel coordinates of detected object
    u_desired = 400.0  # replace with your detected pixel x
    v_desired = 250.0  # replace with your detected pixel y

    # Known plane height of object in BASE frame (e.g. table height)
    Z_base = 0.10  # [m] — adjust to your setup

    # Compute desired EE position in base frame from pixel
    Pd = pixel_to_Pd_base(u_desired, v_desired, Z_base, robot_cam, q0)

    # Desired orientation: keep current orientation of the EE
    H_current = rox.fwdkin(robot, q0)
    Rd = H_current.R

    # IK parameters
    tol = np.array([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]]).T
    Nmax = 200
    alpha = 0.1

    # Solve IK
    q_sol = jacobian_inverse(robot, q0, Rd, Pd, Nmax, alpha, tol)

    # Format outputs
    q_sol_deg = q_sol * 180.0 / math.pi
    q_row = q_sol.reshape(1, 5)

    print("Desired Pd (base frame) [m]:")
    print(Pd.flatten())
    print("\nSolution q (rad):")
    print(q_sol)
    print("\nSolution q (deg):")
    print(q_sol_deg)
    print("\nq as 1x5 row array (deg):")
    print(q_row)


if __name__ == "__main__":
    main()

