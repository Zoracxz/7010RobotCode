from model import Board
from properties import GlobalProperties
from state.state import State
import vision.state_reader as sr
from utils.tic_tac_toe_utils import diff


def _is_valid(board, board_size):
    """
    Check if the detected board has the correct size.
    :param board: 2D list representing the board state
    :param board_size: Expected board size (e.g., 3 for 3x3)
    :return: True if valid size, False otherwise
    """
    if len(board) != board_size:
        return False
    for row in board:
        if len(row) != board_size:
            return False
    return True


class VisionState(State):
    """
    VisionState is a board state management class for Tic Tac Toe using vision input.
    It continuously captures and verifies the board state using computer vision.
    """

    def __init__(self, board_size, moves):
        """
        Initialize the vision-based board state.
        :param board_size: Size of the board (e.g., 3 for 3x3)
        :param moves: List of allowed move symbols (e.g., ['x', 'o'])
        """
        super().__init__(moves)
        self.board_size = board_size
        sr.init()  # Initialize the camera or vision system

    def board(self):
        """
        Capture and return a stable, valid board state using vision.

        This method:
        - Continuously captures frames using sr.get_state()
        - Compares consecutive frames using diff()
        - Only returns the board if:
          * It has the correct size
          * It remains unchanged for a number of consecutive frames defined by hysteresis

        :return: Board object representing the current stable board state
        """
        threshold = GlobalProperties.histeresis
        board = sr.get_state()
        r = 0  # Stability counter: how many consecutive times the board hasn't changed

        while not _is_valid(board, self.board_size) or r < threshold:
            pboard = board
            board = sr.get_state()

            # If the board hasn't changed, increment stability counter
            if diff(pboard, board)[0] == 0:
                r += 1
            else:
                r = 0  # Reset counter if board changed

        return Board(board)

    def move(self, r, c, move):
        """
        This method is intentionally left empty.
        In vision-based mode, the board is updated by observing physical changes, 
        not by direct programmatic manipulation.
        """
        pass

    def __del__(self):
        """
        Clean up the vision system on object deletion.
        """
        sr.destroy()
