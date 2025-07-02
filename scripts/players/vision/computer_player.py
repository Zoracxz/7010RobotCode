from errors.move_error import MoveError
from players.ai.square_board import optimum_move
from players.player import Player
from players.vision.common import wait_for_move
from state.state import State
from speech.__init__ import Speech, symbol_name, number_name


class ComputerPlayer(Player):
    """
    ComputerPlayer represents the AI player for Tic Tac Toe.
    It uses the optimum_move algorithm to decide moves and provides voice feedback.
    """

    def __init__(self, symbol, name, voice):
        """
        Initialize the computer player.

        :param symbol: The symbol for this player ('x' or 'o')
        :param name: Name of the player
        :param voice: Speech system object for voice output
        """
        super().__init__(symbol, name)
        self.voice = voice

    def say(self, text):
        """
        Speak a message using the provided speech system.
        :param text: Message to be spoken
        """
        self.voice.say(text)

    def move(self, state: State, opponent):
        """
        Execute the computer's move.
        - Announces it's the computer's turn
        - Calculates the optimum move using minimax strategy
        - Announces the recommended placement
        - Waits for the human player to physically place the symbol
        - Validates that the placement matches the computer's suggestion

        :param state: The current game state
        :param opponent: The opponent player object
        :raises MoveError: If the placement does not match the computer's suggestion
        """
        board = state.board()
        self.say("Now it's my turn!")

        # Determine best move using AI
        movement = optimum_move(board, opponent.symbol, self.symbol, self.symbol).index
        r, c = movement

        # Verbally instruct the human on where to place the symbol
        self.say("Please put {0} in {1} row and {2} column".format(
            symbol_name(self.symbol), number_name(r + 1), number_name(c + 1)))

        try:
            # Wait for human to physically place the symbol on the board
            dr, dc = wait_for_move(board, state, self.symbol)

            # Validate that placement matches computer's suggestion
            if dr != r or dc != c:
                raise MoveError("Sign was putted not in requested place")

        except MoveError as e:
            # If invalid move detected, complain and terminate game
            self.say("You tried to lie to me. I won't play with you, because {0}".format(e))
            raise MoveError("Computer move error")

        self.say("Thank you")
