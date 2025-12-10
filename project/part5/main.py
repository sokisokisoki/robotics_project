import cv2
import numpy as np
import time
import general_robotics_toolbox as rox
from Arm_Lib import Arm_Device #import the module associated with the arm
from scipy.spatial.transform import Rotation as R

Arm = Arm_Device() # Get DOFBOT object
time.sleep(.2) #this pauses execution for the given number of seconds



class VideoCamera(object):
    def __init__(self):
        # Using OpenCV to capture from device 0. If you have trouble capturing
        # from a webcam, comment the line below out and use a video file
        # instead.
        self.video = cv2.VideoCapture(0)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 30) 
        self.video.set(cv2.CAP_PROP_CONTRAST, 50) 
        self.video.set(cv2.CAP_PROP_EXPOSURE, 156) 
        self.video.set(3, 640)
        self.video.set(4, 480)
        self.video.set(5, 30)  # set frame
        # If you decide to use video.mp4, you must have this file in the folder
        # as the main.py.
        # self.video = cv2.VideoCapture('video.mp4')
 
    def __del__(self):
        self.video.release()
 
    def get_frame(self):
        success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        if(success == False):
            print ("Read Error!")
            return bytes({1})
        return image
    
    def get_frame2(self):
        success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        if(success == False):
            #print ("Read Error!")
            return bytes({1})
       
        ret, jpeg = cv2.imencode('.jpg', image)
        
        return jpeg.tobytes()
    
def jacobian_inverse(robot,q0,Rd,Pd,Nmax,alpha,tol):
# the inputs are 
	# robot: of the Robot class defined in rox. contains rotation axes, position vectors, joint types
	# q0: the initial guess joint angles as a 5x1 numpy array
	# Rd: the desired 3x3 rotation matrix (R_{0T}) as a numpy array
	# Pd: the desired 3x1 position vector (P_{0T}) as a numpy array
	# Nmax: the maximum number of allowed iterations
	# alpha: the update parameter
	# tol: the tolerances for [roll, pitch, yaw, x, y, z] as a 6x1 numpy array

    # set up storage space
    n = len(q0)
    q = np.zeros((n,Nmax+1))
    q[:,0] = q0
    p0T = np.zeros((3,1))
    RPY0T = np.zeros((3,Nmax+1))
    iternum = 0

    # compute the forward kinematics
    H = rox.fwdkin(robot,q[:,0])
    R = H.R
    P = H.p
    P = np.array([[P[0]], [P[1]], [P[2]]])

    # get the initial errorq[:,0]
    dR = np.matmul(R, np.transpose(Rd))
    r = np.array(rox.R2rpy(dR))[None]
    dX = np.concatenate((np.transpose(r), P-Pd))

# iterate while any error element is greater than its tolerance
    while (np.absolute(dX) > tol).any():
	# stop execution if the maximum number of iterations is exceeded
        if iternum < Nmax:
		# compute the forward kinematics
            H = rox.fwdkin(robot, q[:,iternum])
            R = H.R
            p0T = H.p
            p0T = np.array([[p0T[0]], [p0T[1]], [p0T[2]]])

		# compute the error
            dR = np.matmul(R , np.transpose(Rd))
            r = np.array(rox.R2rpy(dR))[None]
            dX = np.concatenate((np.transpose(r), p0T-Pd))

		# calculate the Jacobian matrix
            Jq = rox.robotjacobian(robot, q[:, iternum])
		# compute the update
            j = np.matmul(np.linalg.pinv(Jq), dX)
		# use the update to generate a new q
            q[:, iternum+1] = q[:, iternum] - np.transpose((alpha * j))
            iternum = iternum + 1
        else:
            break
	# return the final estimate of q
    return q[:, iternum]

def readActualJointAngle(jnum):
    """
    function used to read the position of the specified joint
    readActualJointAngle(jnum) reads the position of joint jnum in degrees
    function returns the joint position in degrees
    """
    # call the function to read the position of joint number jnum
    ang = Arm.Arm_serial_servo_read(jnum)
    return ang

def readAllActualJointAngles():
    q = np.array([Arm.Arm_serial_servo_read(1),Arm.Arm_serial_servo_read(2),Arm.Arm_serial_servo_read(3),Arm.Arm_serial_servo_read(4),Arm.Arm_serial_servo_read(5),Arm.Arm_serial_servo_read(6)])
    return q

def rotx(theta):
    # return the principal axis rotation matrix for a rotation about the Xaxis by theta degrees
    if isinstance(theta,np.ndarray):
        theta = theta[0]
    Rx = R.from_euler('x',theta , degrees = True )
    return Rx

def roty(theta):
    # return the principal axis rotation matrix for a rotation about the Yaxis by theta degrees
    if isinstance(theta,np.ndarray):
        theta = theta[0]
    Ry = R.from_euler('y',theta , degrees = True )
    return Ry

def rotz(theta):
    # return the principal axis rotation matrix for a rotation about the Zaxis by theta degrees
    if isinstance(theta,np.ndarray):
        theta = theta[0]
    Rz = R.from_euler('z',theta , degrees = True )
    return Rz

def get_T_base_ee(R, P, q_current):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = P
    return T

def get_T_ee_cam():
    T = np.eye(4)
    T[:3, 3] = np.array([0.09, 0.04, 0.0])
    return T
    
def intersect_ray_with_table(ray_cam, T_base_cam, Z_table):
    R_base_cam = T_base_cam[:3,:3]
    C = T_base_cam[:3, 3]
    d = R_base_cam @ ray_cam
    
    t = (Z_table - C[2]) / d[2]
    P_base = C + t * d
    return P_base

def build_target_ee_transform(point_base, T_base_cam, approach_offset_cam=np.array([0.0, 0.0, -0.10])):
    """
    Form an EE target transform in the base frame:
      - position = point_base + offset expressed in base frame
      - orientation = simple orientation that points tool Z downwards (world -Z)
    """
    R_base_cam = T_base_cam[:3, :3]
    offset_base = R_base_cam @ approach_offset_cam
    desired_pos = point_base + offset_base

    # Desired orientation: tool z axis pointing down (-Z_base).
    z_axis = np.array([0.0, 0.0, -1.0])
    # Choose x_axis by projecting camera x-axis onto XY plane to keep some facing direction
    cam_x_in_base = R_base_cam[:, 0].copy()
    x_proj = cam_x_in_base.copy()
    x_proj[2] = 0.0
    if np.linalg.norm(x_proj) < 1e-6:
        x_axis = np.array([1.0, 0.0, 0.0])
    else:
        x_axis = x_proj / np.linalg.norm(x_proj)
    y_axis = np.cross(z_axis, x_axis)
    y_norm = np.linalg.norm(y_axis)
    if y_norm < 1e-6:
        # fallback
        y_axis = np.array([0.0, 1.0, 0.0])
    else:
        y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, z_axis)
    R_des = np.column_stack([x_axis, y_axis, z_axis])

    T = np.eye(4)
    T[:3, :3] = R_des
    T[:3, 3] = desired_pos
    return T

if __name__ == "__main__":
    # load calibration constants (calibrated with pose = [0 135 0 0 90 0])
    camera_matrix = np.load('calibration/camera_matrix.npy')
    distortion_coeffs = np.load('calibration/dist_coeffs.npy')

    w, h = 640, 480

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        distortion_coeffs,
        (w, h),
        1,
        (w, h)
    )

    cam = VideoCamera()
    
    inv_newK = np.linalg.inv(new_camera_matrix)
    
    #set up the basis unit vectors
    ex = np.array([1 ,0 ,0])
    ey = np.array([0,1,0])
    ez = np.array([0 ,0 ,1])

    # define the link lengths in meters
    l0 = 0.061 # base to servo 1
    l1 = 0.0435 # servo 1 to servo 2
    l2 = 0.08285 # servo 2 to servo 3
    l3 = 0.08285 # servo 3 to servo 4
    l4 = 0.07385 # servo 4 to servo 5
    l5 = 0.05457 # servo 5 to gripper
    
    P01 = (l0+l1)*ez # translation between base frame and 1 frame in base frame
    P12 = np.zeros(3,) # translation between 1 and 2 frame in 1 frame
    P23 = l2*ex # translation between 2 and 3 frame in 2 frame
    P34 = -l3*ez # translation between 3 and 4 frame in 3 frame
    P45 = np.zeros(3,) # translation between 4 and 5 frame in 4 frame
    P5T = -(l4+l5)*ex # translation between 5 and tool frame in 5 frame
    
    H = np.array([ez, -1*ey, -1*ey, -1*ey, -1*ex]).T
    P = np.array([P01, P12, P23, P34, P45, P5T]).T
    joint_type = [0,0,0,0,0]
    
    robot = rox.Robot(H, P, joint_type)   
    
    tol = np.array(np.transpose([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]])) 
    Nmax = 200
    alpha = 0.1

    while True:
        
        q = readAllActualJointAngles()[:5]
        
        R01 = rotz(q[0]) # rotation between base frame and 1 frame
        R12 = roty(-q[1]) # rotation between 1 and 2 frames
        R23 = roty(-q[2]) # rotation between 2 and 3 frames
        R34 = roty(-q[3]) # rotation between 3 and 4 frames
        R45 = rotx(-q[4]) # rotation between 4 and 5 frames
        R5T = roty(0) #the tool frame is defined to be the same as frame 5
        
        Rot = R01*R12*R23*R34*R45*R5T
        Pot = P01 + R01.apply(P12 + R12.apply(P23+ R23.apply(P34 + R34.apply(P45+ R45.apply(P5T)))))
        
        # construct 4x4 T_base_ee
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = Rot.as_matrix()
        T_base_ee[:3, 3] = Pot
        
         # compute camera transform
        T_ee_cam = get_T_ee_cam()             # EE -> camera (your hand-eye)
        T_base_cam = T_base_ee @ T_ee_cam     # base -> camera
        
        frame = cam.get_frame()

        frame = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([0, 98, 192])
        upper_orange = np.array([179, 255, 255])
        mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
        
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        u = None
        v = None
        MIN_CONTOUR_AREA = 200
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (u, v), 5, (0,255,0), -1)
                    # print("Rotation matrix:\n", Rot.as_matrix())
                    # print("Position vector:\n", Pot)
        
        if u is None or v is None:
            continue
        
        pixel_h = np.array([u, v, 1.0], dtype=np.float64)
        ray_cam = inv_newK @ pixel_h
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        Z_table = 0.0
        
        try:
            P_base = intersect_ray_with_table(ray_cam, T_base_cam, Z_table)  # 3-vector in base frame
        except Exception as e:
            print("Ray intersection failed:", e)
            continue
        
        approach_offset_cam = np.array([0.0, 0.0, -0.10])
        T_target_ee = build_target_ee_transform(P_base, T_base_cam, approach_offset_cam)
        
        print("Detected pixel (u,v):", (u,v))
        print("Object base-frame position:", P_base)
        print("Target EE transform (base frame):\n", T_target_ee)
        
        p = T_target_ee[:3,3].reshape(-1, 1)
        # print(p)
        p[1][0] = 0.17
        p[2][0] = 0.0 
        # print(np.eye(3))
        # print(q)
        
        key = cv2.waitKey(1) & 0xff

        if key == ord('q'):
            new_q = jacobian_inverse(robot,q,np.eye(3),p,Nmax,alpha,tol)
            print(new_q)
            break

        result = cv2.bitwise_and(frame, frame, mask=mask)
        

        cv2.imshow("frame", frame)
        cv2.imshow("result", result)
        
    cv2.destroyAllWindows()