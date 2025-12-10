import cv2
import numpy as np

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
    
    while True:

        frame = cam.get_frame()

        frame = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([0, 98, 192])
        upper_orange = np.array([179, 255, 255])
        mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
        
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        MIN_CONTOUR_AREA = 200
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (u, v), 5, (0,255,0), -1)

        result = cv2.bitwise_and(frame, frame, mask=mask)
        action = cv2.waitKey(10) & 0xff

        cv2.imshow("frame", frame)
        cv2.imshow("result", result)

        if action == 27:
            break
        
    cv2.destroyAllWindows()