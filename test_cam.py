import cv2 as cv
import numpy as np
import time

#setup camera
vid_id = 0
cap = cv.VideoCapture(vid_id)

#camera intrinsic parameters
k = np.array([[563.1, 0, 311.5], [0, 561.8, 287.10 ],[0, 0, 1]])

#distortion coefficients
dist = np.array([-0.5539, 0.2666, -.005846, -.004422, .1288])

#aruco marker setup
arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36H11)
arucoParams = cv.aruco.DetectorParameters()
detect = cv.aruco.ArucoDetector(arucoDict, arucoParams)

while True:
    #read frame
    time.sleep(.1)
    ret, frame = cap.read()
    if ret:
        #get shape and new camera matrix
        h, w = frame.shape[:2]
        newcam, roi = cv.getOptimalNewCameraMatrix(k, dist, (w,h), 1, (w,h))

        #undistort frame
        dst = cv.undistort(frame, k, dist, None, newcam)

        #crop frame based on roi
        x, y, w, h = roi

        dst = dst[y:y+h, x:x+w]

        #detect markers
        # corners, ids, rejects = detect.detectMarkers(frame)

        # if len(corners) > 0:
        #     ids = ids.flatten()
            
        #     #draw border around markers
        #     for corner in corners:

        #         corner = corner.reshape((4,2))
        #         topL = (int(corner[0, 0]), int(corner[0, 1]))
        #         topR = (int(corner[1, 0]), int(corner[1, 1]))
        #         botR = (int(corner[2, 0]), int(corner[2, 1]))
        #         botL = (int(corner[3, 0]), int(corner[3, 1]))

        #         cv.line(dst, topL, topR, (0, 0, 255), 2)
        #         cv.line(dst, topR, botR, (0, 0, 255), 2)
        #         cv.line(dst, botR, botL, (0, 0, 255), 2)
        #         cv.line(dst, botL, topL, (0, 0, 255), 2)

        #     #get transformation matrix from marker to camera
        #     marker_size = 1
        #     obj_pts = np.array([[-marker_size / 2, marker_size / 2, 0],[marker_size / 2, marker_size / 2, 0],[marker_size / 2, -marker_size / 2, 0],[-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

        #     corners = np.reshape(corners, (4,2))

        #     ah, rvecs, tvecs = cv.solvePnP(obj_pts, corners, k, dist)

        #     R, _ = cv.Rodrigues(rvecs)

        #     T = np.hstack((R, tvecs.reshape(3, 1)))
        #     T = np.vstack((T, np.array([0, 0, 0, 1])))
        #     print(T, "\n")
        justguy = dst
        dst = cv.cvtColor(dst, cv.COLOR_BGR2HSV)

        #options for what color to detect
        boundaries = (([0, 100, 20], [10, 255, 255]),
                ([90, 100, 20], [110, 255, 255]),
                ([25, 146, 190], [62, 174, 250]),
                ([103, 86, 65], [145, 133, 128]))
        light = np.array(boundaries[1][1])
        dark = np.array(boundaries[1][0])
        mask = cv.inRange(dst, dark, light)

        #find contours around chosen color
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        output = cv.drawContours(dst, contours, -1, (0, 255, 0), 3)
        output = cv.cvtColor(output, cv.COLOR_HSV2RGB)
        # print(contours)
        for contour in contours:
            area=cv.contourArea(contour)
            # print(area)
            if(area>1000):
                x,y,w,h= cv.boundingRect(contour)
                rect = cv.rectangle(output, (x,y), (x+w,y+h), (0,255,0), 5)
                cropped_img=justguy[y:y+h+1, x:x+w+1]
                corners, ids, rejects = detect.detectMarkers(cropped_img)
                if ids != None:
                    cv.imwrite("ah.png", cropped_img)
                    print(ids)
                    break
            else:
                cropped_img=output
                continue
            break
            
        #show frame w/color contours and marker bounding box
        # justguy = cv.drawContours(justguy, contours, -1, (0, 255, 0), 3)
        cv.imshow("a", cropped_img)
        #kill it if q is pressed
        if cv.waitKey(1) == ord("q"):
            break
    
cap.release()