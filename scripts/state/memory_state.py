from model import Board
from state.state import State


class MemoryState(State):
    """
    MemoryState is a simple board state implementation for Tic Tac Toe
    where the board is stored and updated purely in memory (no vision input).
    Typically used for console-based games or testing.
    """

    def __init__(self, board: Board, moves):
        """
        Initialize the MemoryState with a starting board and allowed moves.

        :param board: Board object representing the initial board state
        :param moves: List of allowed move symbols (e.g., ['x', 'o'])
        """
        super().__init__(moves)
        self._board = board  # Internal board object

    def board(self):
        """
        Return the current board state.

        :return: Board object representing the current game state
        """
        return self._board

    def move(self, r, c, move):
        """
        Apply a move to the board at the specified position.

        :param r: Row index
        :param c: Column index
        :param move: Move symbol to place ('x' or 'o')
        
        The board uses an immutable design, so calling put() returns a new board,
        which replaces the old board.
        """
        self._board = self._board.put(r, c, move)
