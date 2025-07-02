#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


def number_name(num):
    """
    Convert a number to its spoken English form.
    :param num: Integer number
    :return: String representation ('first', 'second', etc.)
    """
    if num == 1:
        return 'first'
    elif num == 2:
        return 'second'
    elif num == 3:
        return 'third'
    return num


def symbol_name(symbol):
    """
    Convert a Tic Tac Toe symbol to its spoken name.
    :param symbol: 'x', 'o', or empty
    :return: Spoken string
    """
    if symbol == 'x':
        return 'cross'
    elif symbol == 'o':
        return 'circle'
    return 'empty'


class Speech:
    """
    Speech class for ROS voice output using sound_play.
    This class provides text-to-speech functionality for the robot.
    """

    def __init__(self, female=False, volume=1.0):
        """
        Initialize the speech system.
        
        :param female: True for female voice, False for male voice (Note: voice selection depends on system setup)
        :param volume: Volume level (currently not directly used with sound_play)
        """
        self.engine = SoundClient()
        rospy.sleep(1)  # Wait for sound_play node to be ready
        self.engine.stopAll()  # Stop any previous sounds

        self.female = female
        self.volume = volume  # Reserved for future use

    def say(self, text):
        """
        Speak the provided text through the robot's speaker.
        
        :param text: String message to be spoken
        """
        rospy.loginfo(f"Speaking: {text}")
        voice = "voice_kal_diphone"  # Default TTS voice provided by sound_play
        
        # If system has other voices installed, this can be customized
        self.engine.say(text, voice)

    def stop(self):
        """
        Immediately stop all currently playing sounds or speech.
        """
        self.engine.stopAll()
