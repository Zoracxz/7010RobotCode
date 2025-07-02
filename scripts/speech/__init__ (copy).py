#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

def number_name(num):
    if num == 1:
        return 'first'
    elif num == 2:
        return 'second'
    elif num == 3:
        return 'third'
    return num


def symbol_name(symbol):
    if symbol == 'x':
        return 'cross'
    elif symbol == 'o':
        return 'circle'
    return 'empty'

class Speech:
    def __init__(self, female=False, volume=1.0):
        """
        :param female: True表示女声，False表示男声
        :param volume: 语音音量，0~1
        """
        self.engine = SoundClient()
        rospy.sleep(1)  # 等待 sound_play 节点准备好
        self.engine.stopAll()

        self.female = female
        self.volume = volume

    def say(self, text):
        """
        语音播报指定内容
        """
        rospy.loginfo(f"Speaking: {text}")
        voice = "voice_kal_diphone"

        pub = rospy.Publisher('/voiceout', String, queue_size=10)

        msg = String()
        msg.data = text
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)

        # sound_play 默认只有基本TTS，如果你系统配置了其他voice可以自行扩展
        # 这里为了稳定性用默认

        #self.engine.say(text, voice)
    
    def stop(self):
        """
        停止所有正在播放的语音
        """
        self.engine.stopAll()
