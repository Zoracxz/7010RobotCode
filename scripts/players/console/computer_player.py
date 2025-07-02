from players.player import Player
from state.state import State
from players.ai.square_board import optimum_move


class ComputerPlayer(Player):
    """
    ComputerPlayer represents a basic AI player for Tic Tac Toe.
    It uses the optimum_move algorithm (likely Minimax) to determine the best move.
    """

    def __init__(self, symbol, name):
        """
        Initialize the computer player.

        :param symbol: The symbol for this player ('x' or 'o')
        :param name: The name of the player
        """
        super().__init__(symbol, name)

    def move(self, state: State, opponent: Player):
        """
        Make a move using the AI algorithm.
        - Calculates the optimal move for the current board state
        - Applies the move directly to the board

        :param state: The current game state
        :param opponent: The opponent player object (used for symbol information)
        """
        # Use AI algorithm to find the best move (returns Move object with .index)
        movement = optimum_move(state.board(), opponent.symbol, self.symbol, self.symbol).index
        r, c = movement  # Row and column where to place the symbol

        # Apply the move to the board
        state.move(r, c, self.symbol)
