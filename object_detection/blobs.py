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

    cv2.createTrackbar("Hue Min", "TrackedBars", 0, 179, nothing)
    cv2.createTrackbar("Hue Max", "TrackedBars", 179, 179, nothing)
    cv2.createTrackbar("Sat Min", "TrackedBars", 0, 255, nothing)
    cv2.createTrackbar("Sat Max", "TrackedBars", 255, 255, nothing)
    cv2.createTrackbar("Val Min", "TrackedBars", 0, 255, nothing)
    cv2.createTrackbar("Val Max", "TrackedBars", 255, 255, nothing)

    params = cv2.SimpleBlobDetector_Params()

    while True:
        img = cv2.imread(path)
        cv2.resize(res, 640, 240)
        lower, upper = get_trackbar()

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(imgHSV, lower, upper)

        res = cv2.bitwise_and(img, img, mask=mask)

        #bw = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        #blur = cv2.GaussianBlur(bw, (7,7),0)

        #detector = cv2.SimpleBlobDetector_create(params)
        #keypoints = detector.detect(blur)
        #im_with_keypoints = cv2.drawKeypoints(res, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("TrackedBars", res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()

    return cx, cy

path = "blob.jpg"
detectObject(path)