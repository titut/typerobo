import cv2 as cv
import numpy as np

vid_id = 0
cap = cv.VideoCapture(vid_id)


while True:
    ret, frame = cap.read()
    if ret:
        #image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        boundaries = (([17, 15, 100], [50, 56, 200]),
             ([86, 31, 4], [220, 88, 50]),
             ([25, 146, 190], [62, 174, 250]),
             ([103, 86, 65], [145, 133, 128]))
        light = np.array(boundaries[1][1])
        dark = np.array(boundaries[1][0])
        mask = cv.inRange(frame, dark, light)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        output = cv.drawContours(frame, contours, -1, (0, 255, 0), 3)
        cv.imshow("pls", output)
        
        if cv.waitKey(1) == ord("q"):
            break
        
cap.release()