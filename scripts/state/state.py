from abc import abstractmethod, ABC
from model import Board

class State(ABC):
    """
    Abstract base class representing the state of a Tic Tac Toe game.
    Concrete subclasses must implement specific methods to manage the board state.
    """

    def __init__(self, moves):
        """
        Initialize the State with allowed moves.
        
        :param moves: List of allowed move symbols (e.g., ['x', 'o'])
        """
        self.moves = moves

    def __str__(self):
        """
        String representation of the board for console display.
        
        :return: Multi-line string showing the board
        """
        result = ""
        board = self.board()  # Get current board object
        for i in range(0, len(board)):
            for j in range(0, len(board)):
                result += str(board(i, j))  # Access board cell at (i, j)
                if j < len(board) - 1:
                    result += " "  # Add space between columns
            result += "\n"  # Newline after each row
        return result

    @abstractmethod
    def board(self) -> Board:
        """
        Abstract method to return the current board object.
        Must be implemented by subclasses.

        :return: Board object representing the current game state
        """
        pass

    def end(self):
        """
        Determine if the game has ended (win or draw).

        :return: True if the game is over, False otherwise
        """
        for m in self.moves:
            if self._winning(m):
                return True  # Someone has won
        return self._is_full()  # Game ends if the board is full (draw)

    def result(self):
        """
        Get the game result.

        :return:
            - Winning symbol ('x' or 'o') if someone won
            - 0 if it's a draw or the game is still ongoing
        """
        for m in self.moves:
            if self._winning(m):
                return m
        return 0  # No winner yet or draw

    def _is_full(self):
        """
        Check if the board is completely filled.
        
        :return: True if full, False otherwise
        """
        return self.board().is_full()

    def _winning(self, move):
        """
        Check if the given move symbol has won the game.
        
        :param move: Symbol to check ('x' or 'o')
        :return: True if that symbol has a winning condition
        """
        return self.board().winning(move)

    @abstractmethod
    def move(self, r, c, move):
        """
        Abstract method to apply a move to the board.
        Must be implemented by subclasses.

        :param r: Row index
        :param c: Column index
        :param move: Move symbol to place ('x' or 'o')
        """
        pass
