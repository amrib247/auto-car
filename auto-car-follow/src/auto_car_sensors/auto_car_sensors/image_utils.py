import cv2


def orient_upside_down_camera(frame):
    return cv2.flip(frame, -1)