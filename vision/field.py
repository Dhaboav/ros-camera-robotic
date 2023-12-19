import cv2 as cv
import numpy as np


def field_color(hsv, upper, lower):
    upper_bound = np.array([upper[0], upper[1], upper[2]])
    lower_bound = np.array([lower[0], lower[1], lower[2]])
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
    color_filter = cv.inRange(src=hsv, lowerb=lower_bound, upperb=upper_bound)
    mask_close = cv.morphologyEx(color_filter, cv.MORPH_CLOSE, kernel, iterations=5)
    mask_open = cv.morphologyEx(mask_close, cv.MORPH_OPEN, None, iterations=1)
    return mask_open


def field_contour(color_input, output):
    contours, hierarchy = cv.findContours(color_input, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    hulls = [cv.convexHull(cnt) for cnt in contours]
    mask = np.zeros_like(output)
    for hull in hulls:
        cv.fillConvexPoly(mask, hull, (255,255,255))
    return mask


def field_display(frame_input, contour_input):
    result = cv.bitwise_and(frame_input, contour_input)
    return result