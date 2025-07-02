from players.player import Player
from state.state import State


class HumanPlayer(Player):
    """
    HumanPlayer class represents a human-controlled player for Tic Tac Toe.
    This version uses console-based input for move selection.
    """

    def __init__(self, symbol, name):
        """
        Initialize the human player.

        :param symbol: The symbol for this player ('x' or 'o')
        :param name: The name of the player
        """
        super().__init__(symbol, name)

    def move(self, state: State, opponent: Player):
        """
        Prompt the human player to make a move by entering input via console.
        - Validates the input format
        - Ensures the chosen cell is empty
        - Applies the move if valid

        :param state: Current game state
        :param opponent: The opponent player (unused here, but required by method signature)
        """
        correct_input = False
        board = state.board()  # Get current board state

        while not correct_input:
            movement = input("Enter your move (0-8): ")

            # Validate input length
            if len(movement) > 1:
                raise TypeError("Wrong input! Please enter a single digit between 0 and 8.\n")

            # Validate input character (must be ASCII digits '0' to '8')
            if ord(movement) < 48 or ord(movement) > 56:
                raise TypeError("Wrong input! Please enter a number between 0 and 8.\n")

            movement = int(movement, 10)  # Convert to integer

            row = movement // len(board)
            col = movement % len(board)

            # Check if the chosen cell is already occupied
            if board(row, col) != 0:
                print("This field is already occupied! Please choose another one.\n")
                print(state)  # Print current board for reference
            else:
                correct_input = True  # Valid input

        # Apply the move
        state.move(row, col, self.symbol)
