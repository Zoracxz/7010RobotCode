from copy import deepcopy
import cv2
import vision.image_processor as ip
import numpy as np

# Global variable to store the video capture object
def init():
    """
    Initialize the camera capture.
    Assumes the camera device is at /dev/video4.
    """
    global cap
    cap = cv2.VideoCapture('/dev/video4')
    print("Video initialized {0}x{1}, {2} fps".format(int(cap.get(3)), int(cap.get(4)), int(cap.get(5))))


def _paste_non_zero(dest, src):
    """
    Paste non-zero pixels from src to dest, keeping dest unchanged elsewhere.
    :param dest: Target image (2D array)
    :param src: Source image (2D array)
    :return: New image with non-zero regions from src copied to dest
    """
    s = deepcopy(dest)
    for i in range(len(s)):
        for j in range(len(s[i])):
            if np.any(src[i][j] != 0):
                s[i][j] = src[i][j]
    return s


def _add_lines(frame, lines):
    """
    Draw detected grid lines on the image.
    :param frame: Image to draw lines on
    :param lines: Tuple of horizontal and vertical line positions
    """
    sh = frame.shape
    for i in lines[0]:  # Horizontal lines
        cv2.line(frame, (0, i), (sh[1], i), (0, 255, 0))
    for i in lines[1]:  # Vertical lines
        cv2.line(frame, (i, 0), (i, sh[0]), (0, 255, 0))


def get_state():
    """
    Capture an image from the camera, detect the grid, and recognize symbols in each cell.
    :return: 2D list representing the board state (e.g., 'x', 'o', or 0 for empty)
    """
    threshold = 5  # Board dimension sanity check threshold
    ret, frame = cap.read()
    
    binary = ip.to_binary_color(frame)  # Convert image to binary for processing
    sq, lines = ip.split_to_squares(binary)  # Split the grid into individual cells
    
    # Sanity check to ensure the grid detection makes sense
    if len(sq) < 2 or len(sq) > threshold or len(sq[0]) > threshold:
        cv2.imshow('frame', binary)
        cv2.waitKey(1) & 0xFF == ord('q')
        return []

    state = []
    # Recognize symbol ('x', 'o', or 0) in each cell
    for row in sq:
        state.append([])
        for column in row:
            state[-1].append(ip.recognize_shape(column))
    
    # Optional: draw lines and display result
    _add_lines(binary, lines)
    cv2.imshow('frame', binary)
    cv2.waitKey(1) & 0xFF == ord('q')  # Required to show image in OpenCV
    return state


def destroy():
    """
    Release the camera and close OpenCV windows.
    """
    cap.release()
    cv2.destroyAllWindows()
