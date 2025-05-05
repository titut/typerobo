import cv2 as cv
import numpy as np
import os


class Camera_dummy:
    def __init__(self):
        # setup camera
        vid_id = 0
        self.cap = cv.VideoCapture(vid_id)

        # camera intrinsic parameters
        self.k = np.array([[563.1, 0, 311.5], [0, 561.8, 287.10], [0, 0, 1]])

        # distortion coefficients
        self.dist = np.array([-0.5539, 0.2666, -0.005846, -0.004422, 0.1288])

        # aruco marker setup
        arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36H11)
        arucoParams = cv.aruco.DetectorParameters()
        self.detect = cv.aruco.ArucoDetector(arucoDict, arucoParams)

        self.in2m = 39.37

    def color_pose(self, color):
        try:
            os.remove("crop.png")
        except:
            print("No File Deleted")

        id_found = False

        while id_found == False:
            ret, frame = self.cap.read()

            h, w = frame.shape[:2]
            newcam, roi = cv.getOptimalNewCameraMatrix(
                self.k, self.dist, (w, h), 1, (w, h)
            )

            # undistort
            dst = cv.undistort(frame, self.k, self.dist, None, newcam)

            # crop frame based on roi
            x, y, w, h = roi

            dst = dst[y : y + h, x : x + w]

            dst = cv.cvtColor(dst, cv.COLOR_BGR2HSV)

            justguy = cv.cvtColor(dst, cv.COLOR_HSV2RGB)

            if color == 1:
                boundaries = ([90, 100, 20], [110, 255, 255])
            else:
                boundaries = ([0, 100, 20], [10, 255, 255])

            light = np.array(boundaries[1])
            dark = np.array(boundaries[0])
            mask = cv.inRange(dst, dark, light)

            contours, _ = cv.findContours(
                mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
            )
            output = cv.drawContours(dst, contours, -1, (0, 255, 0), 3)
            output = cv.cvtColor(output, cv.COLOR_HSV2RGB)
            for contour in contours:
                area = cv.contourArea(contour)
                # print(area)
                if area > 1000:
                    x, y, w, h = cv.boundingRect(contour)
                    rect = cv.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 5)
                    cropped_img = justguy[y : y + h + 1, x : x + w + 1]
                    corners, ids, rejects = self.detect.detectMarkers(cropped_img)
                    if ids != None:
                        cv.imwrite("crop.png", cropped_img)
                        id_found = True
                        break
                else:
                    cropped_img = output
                    continue
                break

            # Sharpen the image
            # kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])

            # sharpened_image = cv.filter2D(cropped_img, -1, kernel)
            # corners, ids, rejects = self.detect.detectMarkers(sharpened_image)
        pls = cv.imread("ah.png")
        corners, ids, rejects = self.detect.detectMarkers(pls)
        # get transformation matrix from marker to camera
        ids = ids.flatten()
        ids = ids.tolist()

        return self.aruco_detection(ids[0])

    # def red_press(self):
    #     transform = self.aruco_detection(1)
    #     return [transform[1,3]/self.in2m, transform[0,3]/self.in2m, transform[2,3]/self.in2m]

    #     #return [0.25,0.22,0.25]

    # def blue_press(self):
    #     transform = self.aruco_detection(2)
    #     return [transform[1,3]/self.in2m, transform[0,3]/self.in2m, transform[2,3]/self.in2m]

    #     #return [0.25, -0.22, 0.25]

    def aruco_detection(self, tag_id):
        print(tag_id)
        id_found = False

        while id_found == False:
            # read frame
            ret, frame = self.cap.read()
            if ret:
                # get shape and new camera matrix
                h, w = frame.shape[:2]
                newcam, roi = cv.getOptimalNewCameraMatrix(
                    self.k, self.dist, (w, h), 1, (w, h)
                )

                # undistort frame
                dst = cv.undistort(frame, self.k, self.dist, None, newcam)

                # crop frame based on roi
                x, y, w, h = roi

                dst = dst[y : y + h, x : x + w]

                # detect markers
                corners, ids, rejects = self.detect.detectMarkers(frame)
                # print(corners)

                if len(corners) > 0:
                    ids = ids.flatten()
                    ids = ids.tolist()
                    print(ids)
                    print(type(ids))
                    tag_found = False
                    idx = 0
                    for id in ids:
                        if id == tag_id:
                            # tag = id
                            print(idx)
                            tag_found = True
                            tag_corners = corners[idx]
                            # print(tag_corners)
                        idx = idx + 1

                    if tag_found == True:
                        # draw border around markers
                        for corner in tag_corners:

                            corner = corner.reshape((4, 2))
                            topL = (int(corner[0, 0]), int(corner[0, 1]))
                            topR = (int(corner[1, 0]), int(corner[1, 1]))
                            botR = (int(corner[2, 0]), int(corner[2, 1]))
                            botL = (int(corner[3, 0]), int(corner[3, 1]))

                            cv.line(dst, topL, topR, (0, 0, 255), 2)
                            cv.line(dst, topR, botR, (0, 0, 255), 2)
                            cv.line(dst, botR, botL, (0, 0, 255), 2)
                            cv.line(dst, botL, topL, (0, 0, 255), 2)

                        # get transformation matrix from marker to camera
                        marker_size = 1
                        obj_pts = np.array(
                            [
                                [-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0],
                            ],
                            dtype=np.float32,
                        )

                        corners = np.reshape(tag_corners, (4, 2))

                        ah, rvecs, tvecs = cv.solvePnP(
                            obj_pts, corners, self.k, self.dist
                        )

                        R, _ = cv.Rodrigues(rvecs)

                        T = np.hstack((R, tvecs.reshape(3, 1)))
                        T = np.vstack((T, np.array([0, 0, 0, 1])))
                        print(T, "\n")
                        print(type(T))
                        id_found = True
                        print(
                            [
                                T[1][3] / self.in2m,
                                T[0][3] / self.in2m,
                                T[2][3] / self.in2m,
                            ]
                        )
                        return [
                            T[1][3] / self.in2m,
                            T[0][3] / self.in2m,
                            T[2][3] / self.in2m,
                        ]
