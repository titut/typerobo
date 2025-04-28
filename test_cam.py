import cv2 as cv
import numpy as np

vid_id = 0
cap = cv.VideoCapture(vid_id)

k = np.array([[563.1, 0, 311.5], [0, 561.8, 287.10 ],[0, 0, 1]])

dist = np.array([-0.5539, 0.2666, -.005846, -.004422, .1288])

arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_APRILTAG_36H11)

while True:
    ret, frame = cap.read()
    if ret:

        h, w = frame.shape[:2]
        newcam, roi = cv.getOptimalNewCameraMatrix(k, dist, (w,h), 1, (w,h))

        dst = cv.undistort(frame, k, dist, None, newcam)

        x, y, w, h = roi

        dst = dst[y:y+h, x:x+w]
        image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        boundaries = (([17, 15, 100], [50, 56, 200]),
             ([86, 31, 4], [220, 88, 50]),
             ([25, 146, 190], [62, 174, 250]),
             ([103, 86, 65], [145, 133, 128]))
        light = np.array(boundaries[1][1])
        dark = np.array(boundaries[1][0])
        mask = cv.inRange(dst, dark, light)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        output = cv.drawContours(dst, contours, -1, (0, 255, 0), 3)
        cv.imshow("pls", output)
        
        if cv.waitKey(1) == ord("q"):
            break
        
cap.release()