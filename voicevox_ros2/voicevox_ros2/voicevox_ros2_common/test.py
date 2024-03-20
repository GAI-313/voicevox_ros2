#!/usr/bin/emv python3
from rclpy.node import Node
import rclpy
from voicevox_ros2_tts import tts_speaker


def main():
    rclpy.init()
    node = rclpy.create_node('vvr2_common_test')
    tts_speaker(node, text='こんにちは！これはスピーカーレスポンステストです。生麦生米生卵', id=26)
    print('done')
    rclpy.shutdown()
    node.destroy_node()
    
# debug
if __name__ == '__main__':
    main()
