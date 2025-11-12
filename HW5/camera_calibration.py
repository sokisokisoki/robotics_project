import cv2   # Import all cv2 functions
import numpy as np

# ---- PARAMETERS ----
# Number of inner corners per a chessboard row and column
# Example: (9,6) means 9 inner corners across, 6 down
pattern_size = (8, 5)

# Termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points like (0,0,0), (1,0,0), (2,0,0), ...
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# ---- STEP 1: COLLECT IMAGES ----
cam = cv2.VideoCapture(0)
count = 0
print("Press 's' to save a frame for calibration, 'q' to quit.")

while True:
    result, frame = cam.read()
    if not result:
        print("Camera frame not captured properly.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    retval, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if retval:
        cv2.drawChessboardCorners(frame, pattern_size, corners, retval)
    
    cv2.imshow('Calibration View', frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s') and retval:
        # Improve accuracy of detected corners
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        objpoints.append(objp)
        count += 1
        cv2.imwrite(f"calib_{count}.png", frame)
        print(f"Saved calibration image #{count}")

    elif key == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

if len(objpoints) < 5:
    print("Not enough calibration images captured.")
    exit()

# ---- STEP 2: CALIBRATE CAMERA ----
img_shape = gray.shape[::-1]
retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

print("\nCalibration complete.")
print("Camera matrix:\n", cameraMatrix)
print("Distortion coefficients:\n", distCoeffs)

# ---- STEP 3: CHECK PROJECTION ERROR ----
total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error
print(f"\nMean reprojection error: {total_error / len(objpoints):.6f}")

