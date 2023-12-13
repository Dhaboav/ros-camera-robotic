import cv2 as cv
import numpy as np


class Color:
    def __init__(self, camera, width, height):
        self.camera = camera
        self.width = width
        self.height = height

    @staticmethod
    def empty(x):
        pass

    def run(self):
        cv.namedWindow('panel')
        cv.resizeWindow('panel', 320, 240)
        cv.createTrackbar('lower_hue', 'panel', 0, 179, self.empty)
        cv.createTrackbar('upper_hue', 'panel', 255, 255, self.empty)
        cv.createTrackbar('lower_sat', 'panel', 0, 255, self.empty)
        cv.createTrackbar('upper_sat', 'panel', 255, 255, self.empty)
        cv.createTrackbar('lower_val', 'panel', 0, 255, self.empty)
        cv.createTrackbar('upper_val', 'panel', 255, 255, self.empty)

        cap = cv.VideoCapture(self.camera)
        cap.set(3, self.width)
        cap.set(4, self.height)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            lower_hue = cv.getTrackbarPos('lower_hue', 'panel')
            upper_hue = cv.getTrackbarPos('upper_hue', 'panel')
            lower_sat = cv.getTrackbarPos('lower_sat', 'panel')
            upper_sat = cv.getTrackbarPos('upper_sat', 'panel')
            lower_val = cv.getTrackbarPos('lower_val', 'panel')
            upper_val = cv.getTrackbarPos('upper_val', 'panel')
            lower_value = np.array([lower_hue, lower_sat, lower_val])
            upper_value = np.array([upper_hue, upper_sat, upper_val])

            frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            masking = cv.inRange(frame_hsv, lower_value, upper_value)
            output = cv.bitwise_and(frame, frame, mask=masking)

            print(f"""
Lower:({lower_hue}, {lower_sat}, {lower_val})
Upper:({upper_hue}, {upper_sat}, {upper_val})
    """)

            cv.imshow("color", output)
            if cv.waitKey(1) & 0xFF == ord("x"):
                break

        cap.release()
        cv.destroyAllWindows()