import math
import cv2
import numpy as np
from properties import GlobalProperties  # Custom configuration parameters

def to_binary_color(frame):
    """
    Convert the image to grayscale and apply binary thresholding.
    :param frame: Original color image
    :return: Binary image (black and white)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    _, binary = cv2.threshold(gray, GlobalProperties.black_threshold, 255, cv2.THRESH_BINARY)
    return binary


def _get_lines(frame, kernel):
    """
    Detect lines in the image using Hough Transform after dilation.
    :param frame: Binary image
    :param kernel: Kernel for morphological dilation
    :return: List of detected line positions
    """
    dltd = cv2.dilate(frame, kernel, iterations=1)
    lines = cv2.HoughLines(255 - dltd, 1, np.pi / 180, GlobalProperties.line_votes)
    return [] if lines is None else np.array([abs(l[0][0]) for l in lines])


def _filter_lines(lines):
    """
    Remove duplicate or closely spaced lines based on configured threshold.
    :param lines: Raw detected line positions
    :return: Filtered list of line positions
    """
    threshold = GlobalProperties.line_space_threshold
    res = []
    for l in lines:
        take = True
        for lr in res:
            if abs(l - lr) < threshold:
                take = False
        if take:
            res.append(int(l))
    return res


def split_to_squares(frame):
    """
    Split the image into square grid cells based on detected lines.
    :param frame: Binary image
    :return: 2D list of image regions representing grid cells, plus line positions
    """
    hkernel = np.ones((1, 100))  # Horizontal line detection kernel
    vkernel = np.ones((100, 1))  # Vertical line detection kernel

    hlines = [0] + _filter_lines(_get_lines(frame, hkernel)) + [frame.shape[0] - 1]
    vlines = [0] + _filter_lines(_get_lines(frame, vkernel)) + [frame.shape[1] - 1]

    hlines.sort()
    vlines.sort()

    res = []
    for i in range(1, len(hlines)):
        res.append([])
        for j in range(1, len(vlines)):
            res[-1].append(frame[hlines[i - 1]:hlines[i], vlines[j - 1]:vlines[j]])
    return res, (hlines, vlines)


def _is_circle(frame):
    """
    Detect if the image contains a circle using Hough Circle Transform.
    :param frame: Binary image region
    :return: True if a circle is detected, False otherwise
    """
    if np.all(frame == 0):
        return False
    circ = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, 1, 20,
                            param1=50, param2=25, minRadius=0, maxRadius=0)
    return circ is not None and len(circ) == 1


def _pad_image(frame, padding=40):
    """
    Crop the image by removing borders to reduce noise.
    :param frame: Image to pad/crop
    :param padding: Number of pixels to crop from each side
    :return: Cropped image
    """
    sh = frame.shape
    return frame[padding:sh[0] - padding, padding:sh[1] - padding]


def _is_empty(frame):
    """
    Determine if the image region is considered empty (almost all white).
    :param frame: Binary image region
    :return: True if empty, False otherwise
    """
    threshold = 50  # Pixel count threshold to consider region empty
    padded = _pad_image(frame)
    return np.count_nonzero(255 - padded) < threshold


def _filter_cross_lines(lines):
    """
    Filter lines based on angle to detect crosses (diagonal lines).
    :param lines: Raw lines from Hough Transform
    :return: List of unique cross line angles
    """
    if lines is None:
        return []

    threshold = 0.35  # Angle similarity threshold (radians)
    min = GlobalProperties.min_cross_angle
    max = GlobalProperties.max_cross_angle
    res = []

    for l in lines:
        ang = l[0][1] if l[0][1] < math.pi/2 else math.pi - l[0][1]
        if ang < min or ang > max:
            continue
        toadd = True
        for r in res:
            if abs(r - l[0][1]) < threshold:
                toadd = False
        if toadd:
            res.append(l[0][1])
    return res


def _is_cross(frame):
    """
    Detect if the image region contains a cross ('X').
    :param frame: Binary image region
    :return: True if a cross is detected, False otherwise
    """
    if np.all(frame == 0):
        return False
    pframe = _pad_image(frame, 20)
    lines = cv2.HoughLines(255 - pframe, 1, np.pi / 180, 120)
    return len(_filter_cross_lines(lines)) == 2


def recognize_shape(frame):
    """
    Recognize the shape inside a grid cell.
    :param frame: Binary image region of the cell
    :return: 'x' for cross, 'o' for circle, 0 for empty
    """
    if _is_cross(frame):
        return 'x'
    if _is_circle(frame):
        return 'o'
    return 0
