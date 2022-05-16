import cv2
import numpy as np

def nothing(i):
    pass

def get_trackbar():
    hue_min = cv2.getTrackbarPos("Hue Min", "TrackedBars")
    hue_max = cv2.getTrackbarPos("Hue Max", "TrackedBars")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackedBars")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackedBars")
    val_min = cv2.getTrackbarPos("Val Min", "TrackedBars")
    val_max = cv2.getTrackbarPos("Val Max", "TrackedBars")

    lower = np.array([hue_min, sat_min, val_min])
    upper = np.array([hue_max, sat_max, val_max])

    return lower, upper

def detectObject(path):
    cv2.namedWindow("TrackedBars")
    cv2.resizeWindow("TrackedBars", 640, 240)

    cv2.createTrackbar("Hue Min", "TrackedBars", 0, 179, nothing)
    cv2.createTrackbar("Hue Max", "TrackedBars", 179, 179, nothing)
    cv2.createTrackbar("Sat Min", "TrackedBars", 0, 255, nothing)
    cv2.createTrackbar("Sat Max", "TrackedBars", 255, 255, nothing)
    cv2.createTrackbar("Val Min", "TrackedBars", 0, 255, nothing)
    cv2.createTrackbar("Val Max", "TrackedBars", 255, 255, nothing)

    while True:
        img = cv2.imread(path)
        lower, upper = get_trackbar()

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(imgHSV, lower, upper)

        res = cv2.bitwise_and(img, img, mask=mask)

        bw = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(bw, 127, 255, 0)
        blur = cv2.GaussianBlur(thresh, (7,7),0)

        contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        #find centre of contours
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(res, (cx, cy), 7, (0, 0, 255), -1)

        cv2.drawContours(res, contours, -1, (0,255,0), 3)
        cv2.imshow("TrackedBars", res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()

    return cx, cy

path = "hand_color_image.jpg"
detectObject(path)