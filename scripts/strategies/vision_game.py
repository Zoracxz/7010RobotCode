#!/usr/bin/env python3

# Import player classes
from players.vision.human_player import HumanPlayer
from players.vision.computer_player import ComputerPlayer

# Import game state management for vision-based board
from state.vision_state import VisionState

# Import the general game strategy base class
from strategies.game_strategy import GameStrategy


class VisionHumanPlayersGame(GameStrategy):
    """
    VisionHumanPlayersGame defines a Tic Tac Toe game where:
    - The board is recognized using computer vision
    - A human plays against the computer
    """

    def __init__(self, voice):
        """
        Initialize the game with players and board state.
        
        :param voice: Speech system instance, used for computer player announcements
        """
        super().__init__(
            players=[
                HumanPlayer(name="Mikolaj", symbol='x'),              # Human player with 'x' symbol
                ComputerPlayer(name="Arek", symbol='o', voice=voice) # Computer player with 'o' symbol and voice output
            ],
            state=VisionState(3, moves=['x', 'o'])  # 3x3 board with allowed moves 'x' and 'o'
        )

    def introduce(self):
        """
        Return the introductory message for the game.
        :return: String introduction
        """
        return "Vision game is a future. Now please show me the board."
