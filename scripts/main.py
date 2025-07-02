#!/usr/bin/env python3

import os
import sys

# Get the absolute path of the current script
script_dir = os.path.dirname(os.path.realpath(__file__))
# Add the script's directory to the Python path for module import
sys.path.append(script_dir)

import rospy

# Import custom error for invalid moves
from errors.move_error import MoveError
# Import speech module for robot voice output
from speech import Speech
# Import game logic for vision-based Tic Tac Toe
from strategies.vision_game import VisionHumanPlayersGame

def result_text(sym, plrs):
    """
    Determine the game result text based on the winning symbol.
    :param sym: The symbol representing the winner ('x', 'o' or 0 for tie)
    :param plrs: List of player objects
    :return: String describing the game result
    """
    if sym == 0:
        return "Tie"
    for p in plrs:
        if p.symbol == sym:
            return "And the winner is... {0}!".format(p.name)
    return "Unknown result"

# Initialize ROS node
rospy.init_node('vision_game_node')
rospy.loginfo('vision_game_node started')

# Initialize the speech system with female voice
narrator = Speech(female=True)

# Create the vision-based Tic Tac Toe game instance
strategy = VisionHumanPlayersGame(voice=narrator)

# Get game state and player information
state = strategy.state
players = strategy.players
turn = 0  # 0: player1's turn, 1: player2's turn

# Speak game introduction
narrator.say(strategy.introduce())

error = False  # Flag to track if an error occurs during the game

# Main game loop
while not state.end():
    print(state)  # Print current board state
    narrator.say("Now it's {0} move".format(players[turn].name))  # Announce whose turn it is
    try:
        players[turn].move(state, players[(turn+1) % 2])  # Execute the current player's move
    except (MoveError, TypeError) as me:
        # Handle invalid move error
        narrator.say("There was an error. You probably did forbidden thing")
        error = True
        break
    turn = (turn + 1) % 2  # Switch turns

# After the game ends
if not error:
    # Announce the winner or tie
    narrator.say(result_text(state.result(), players))
else:
    narrator.say("Game ended without result")
