#!/usr/bin/emv python3
from rclpy.node import Node
import rclpy

from voicevox_ros2_interface.msg import Speaker
from std_msgs.msg import String

import threading
import time
import sys
import traceback

import smach

class tts_speaker():
    def __init__(self, node=None, text='テキスト引数が指定されていません。', id=3, timeout=5, wait=True):
        ### node arg check
        if type(node) is not Node:
            rclpy.logging.get_logger('VoiceVox_tty_speaker').error('''
ARGUMENT ERROR !!!
you not define the arg "node" !
please add the Node type instant

This error happend in function : tty_speaker
            ''')
            sys.exit()

        self.wait = wait
        self.node = node

        if self.wait:
            self.status_flag = False
            self.msg_bu = None
            self.status_sub = self.node.create_subscription(String, '/voicevox_ros2/status', self._cb_status, 10)
        
        self.text = text
        self.id = id
        self.init_time = time.time()
        self.timeout = timeout

        self.pub = self.node.create_publisher(Speaker, '/voicevox_ros2/speaker', 10)

        self.thread = threading.Thread(target=self.execute, daemon=True)
        self.thread.start()
        #self.execute()

    def _cb_status(self, msg):
        print(msg)
        if msg.data == 'done':
            print('well')
            self.status_flag = True

    def execute(self):
        while True:
            try:
                topic_info = self.node.get_subscriber_names_and_types_by_node(
                                node_name='voicevox_ros2_core', node_namespace='')

                sp = Speaker()
                sp.text = self.text
                sp.id = self.id

                self.pub.publish(sp)

                break
            except rclpy._rclpy_pybind11.NodeNameNonExistentError:
                if time.time() - self.init_time > self.timeout:
                    rclpy.logging.get_logger('VoiceVox_tty_speaker').error('''
voicevox_ros2_core is not running !
This function need a voicevox_ros2_core . please execute this on other terminal

ros2 run viocevox_ros2 voicevox_ros2_core
                    ''')
                    break
                continue

        if self.wait:
            while not self.status_flag:
                pass

class tts_SpeakerState(smach.State):
    def __init__(self, node, text='テキスト引数が指定されていません。', id=3, timeout=5):
        smach.State.__init__(self,
                        outcomes=['success', 'failure', 'timeout'])
        self.text = text
        self.id = id
        self.node = node
        self.init_time = time.time()
        self.timeout = timeout
        self.pub = self.node.create_publisher(Speaker, '/voicevox_ros2/speaker', 10)

    def execute(self, userdata):
        if type(self.node) is not Node:
            rclpy.logging.get_logger('VoiceVox_tty_speaker_state').error('''
ARGUMENT ERROR !!!
you not define the arg "node" !
please add the Node type instant

This error happend in function : tty_speaker
            ''')
            return 'failure'

        try:
            while True:
                try:
                    topic_info = self.node.get_subscriber_names_and_types_by_node(
                                    node_name='voicevox_ros2_core', node_namespace='')

                    sp = Speaker()
                    sp.text = self.text
                    sp.id = self.id

                    self.pub.publish(sp)
                    return 'success'
                except rclpy._rclpy_pybind11.NodeNameNonExistentError:
                    if time.time() - self.init_time > self.timeout:
                        rclpy.logging.get_logger('VoiceVox_tty_speaker').error('''
    voicevox_ros2_core is not running !
    This function need a voicevox_ros2_core . please execute this on other terminal

    ros2 run viocevox_ros2 voicevox_ros2_core
                        ''')
                        return 'timeout'
                    continue
        except:
            rclpy.logging.get_logger('VoiceVox_tty_speaker_state').error(traceback.format_exc())
            return 'failure'

# debug
if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('vvr2_common_test')
    tts_speaker(node=node, text='こんにちは！これはスピーカーレスポンステストです。生麦生米生卵')
    print('done')
    rclpy.shutdown()
    node.destroy_node()
