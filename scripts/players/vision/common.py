from errors.move_error import MoveError
from model import Board
from state.state import State


def wait_for_move(prevboard: Board, state: State, expected_move):
    """
    Continuously monitor the board state until exactly one valid move is detected.

    This function is designed for vision-based Tic Tac Toe:
    - It waits for the human player to physically place their symbol on the board
    - It compares consecutive board states to detect changes
    - It validates that exactly one new symbol of the expected type was added

    :param prevboard: The previous stable Board state (before the move)
    :param state: The State object managing current board state
    :param expected_move: The symbol expected to be placed ('x' or 'o')
    :return: (row, column) tuple indicating where the move was made
    :raises MoveError: If an invalid move is detected (e.g., wrong symbol, multiple moves)
    """
    pboard = prevboard  # Previous board snapshot
    board = state.board()  # Current board snapshot

    while True:
        d, diffs = pboard.diff(board)  # Compare boards, get number and positions of differences

        if d == 1:
            # Exactly one change detected
            dr, dc = diffs[0]

            # Check that the changed cell was previously empty
            if pboard(dr, dc) != 0:
                raise MoveError("You can't put a sign on a busy field")

            # Check that the correct symbol was placed
            if board(dr, dc) != expected_move:
                raise MoveError("It's not your sign")

            return dr, dc  # Valid move detected, return position

        elif d > 1:
            # More than one change detected, invalid situation
            raise MoveError("You can only make one move")

        # If no valid move detected, keep monitoring
        pboard = board
        board = state.board()
