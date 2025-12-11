import cv2
import csv
import numpy as np
import time
from Arm_Lib import Arm_Device #import the module associated with the arm

Arm = Arm_Device() # Get DOFBOT object
time.sleep(.2) #this pauses execution for the given number of seconds


class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture(0)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.video.set(cv2.CAP_PROP_BRIGHTNESS, 30) 
        self.video.set(cv2.CAP_PROP_CONTRAST, 50) 
        self.video.set(cv2.CAP_PROP_EXPOSURE, 156) 
        self.video.set(3, 640)
        self.video.set(4, 480)
        self.video.set(5, 30)
 
    def get_frame(self):
        success, image = self.video.read()
        if(success == False):
            print ("Read Error!")
            return bytes({1})
        return image


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

def moveJoint(jnum,ang,speedtime):
    """
    function used to move the specified joint to the given position
    moveJoint(jnum, ang, speedtime) moves joint jnum to position ang degrees in speedtime milliseconds
    function returns nothing
    """
    # call the function to move joint number jnum to ang degrees in speedtime milliseconds
    Arm.Arm_serial_servo_write(jnum,ang,speedtime)
    return

if __name__ == "__main__":
    # load calibration constants (calibrated with pose = [90 135 0 0 90 0])
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
    
    tol = np.array(np.transpose([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]])) 
    Nmax = 200
    alpha = 0.1

    with open('pixel_u_v.csv','w',newline='') as csvfile:
        writer = csv.writer(csvfile)

        detect = False

        while True:
            
            q = readAllActualJointAngles()[:5]
            
            frame = cam.get_frame()

            frame = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)

            hsv_frame = cv2.cvtColor(frame, 
                                     cv2.COLOR_BGR2HSV)

            lower_orange = np.array([0, 98, 192])
            upper_orange = np.array([179, 255, 255])
            mask = cv2.inRange(hsv_frame, 
                               lower_orange, 
                               upper_orange)
            
            countours, _ = cv2.findContours(mask, 
                                            cv2.RETR_EXTERNAL, 
                                            cv2.CHAIN_APPROX_SIMPLE)
            
            u = None
            v = None
            MIN_CONTOUR_AREA = 200
            if len(countours) > 0:
                c = max(countours, key=cv2.contourArea)
                if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        u = int(M["m10"] / M["m00"])
                        v = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (u, v), 5, (0,255,0), -1)
            
            if u is None or v is None:
                continue
            
            key = cv2.waitKey(1) & 0xff

            if key == ord('d'):
                detect = True
            if key == ord('f'):
                detect = False
            
            if detect:
                print("Detected pixel (u,v):", (u,v))

                writer.writerow([u,v])            

                # bang-bang control for joint 1
                if u < w // 2 - 25:
                    new_joint_angle = min(q[0] + 3, 180)
                    moveJoint(1, new_joint_angle, 200)
                elif u > w // 2 + 25:
                    new_joint_angle = max(q[0] - 3, 0)
                    moveJoint(1, new_joint_angle, 200)

                # bang-bang control for joint 4
                if v < h // 2 - 25:
                    new_joint_angle = min(q[3] + 3, 180)
                    moveJoint(4, new_joint_angle, 200)
                elif v > h // 2 + 25:
                    new_joint_angle = max(q[3] - 3, 0)
                    moveJoint(4, new_joint_angle, 200)

            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            cv2.imshow("frame", frame)
            cv2.imshow("result", result)
            
            if key == ord('q'):
                break
        
    cv2.destroyAllWindows()