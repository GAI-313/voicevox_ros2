#!/usr/bin/emv python3
from rclpy.node import Node
import rclpy

from voicevox_ros2_interface.msg import Speaker

import threading
import time
import sys

class tts_speaker():
    def __init__(self, node=None, text='テキスト引数が指定されていません。', id=3, timeout=5):
        ### node arg check
        if type(node) is not Node:
            rclpy.logging.get_logger('VoiceVox_tty_speaker').error('''
ARGUMENT ERROR !!!
you not define the arg "node" !
please add the Node type instant

This error happend in function : tty_speaker
            ''')
            sys.exit()
        
        self.text = text
        self.id = id
        self.node = node
        self.init_time = time.time()
        self.timeout = timeout

        self.pub = self.node.create_publisher(Speaker, '/voicevox_ros2/speaker', 10)


        #self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        #self.thread.start()
        self.execute()

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

# debug
if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('vvr2_common_test')
    tts_speaker()
    print('done')
    rclpy.shutdown()
    node.destroy_node()
