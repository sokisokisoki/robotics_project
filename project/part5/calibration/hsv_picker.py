import cv2
import numpy as np

def nothing(x):
    pass

img = np.zeros((300, 512, 3), np.uint8)
cv2.namedWindow("Trackbars")

cv2.createTrackbar("H Min", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("H Max", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

image = cv2.imread("/home/group2/Desktop/Projects/orange_string.png")
if image is None:
    print("not read")
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

while True:
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)

    cv2.imshow("Original Image", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()