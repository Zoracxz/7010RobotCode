# Compare two board states and return the number and positions of differences.
# Return:
#   -1 : Board sizes do not match
#    0 : Boards are identical
#    c : Number of different cells (1 <= c <= 9), along with their positions

def diff(board1, board2):
    """
    Compare two Tic Tac Toe board states and detect the differences.

    :param board1: First board state (2D list)
    :param board2: Second board state (2D list)
    :return: Tuple (number_of_differences, list_of_positions)
             - If board sizes differ: returns (-1, [])
             - If boards are identical: returns (0, [])
             - Else returns (count, list of (row, col) positions that differ)
    """
    if len(board1) != len(board2):
        return -1, []  # Board dimensions mismatch

    diffs = []  # Store positions where cells differ

    for i in range(len(board1)):
        if len(board1[i]) != len(board2[i]):
            return -1, []  # Row dimensions mismatch

        for j in range(len(board1[i])):
            if board1[i][j] != board2[i][j]:
                diffs.append((i, j))  # Record differing cell position

    return len(diffs), diffs
