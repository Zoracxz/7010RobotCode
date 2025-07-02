from players.player import Player

class GameStrategy:
    """
    GameStrategy defines the general structure for a Tic Tac Toe game.
    It acts as a parent class for specific game modes (e.g., vision-based, console-based).
    """

    def __init__(self, players, state):
        """
        Initialize the game with players and board state.

        :param players: List of Player objects (e.g., HumanPlayer, ComputerPlayer)
        :param state: Board state management object (implements board(), move(), etc.)
        """
        self.players = players
        self.state = state

    def introduce(self):
        """
        Return a general introduction message for the game.
        This method can be overridden by subclasses for custom introductions.

        :return: String introduction
        """
        return "Let's play"
