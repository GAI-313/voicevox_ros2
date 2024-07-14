#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.srv import Speaker

def say(node, text, id=3):
    def send_req(text, id):
        req.text = text
        req.id = id
        future = cli.call_async(req)
        #rclpy.spin_until_future_complete(node, future)
        #return future.result()
    
    cli = node.create_client(Speaker, "voicevox_ros2/speaker_srv")
    while not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn("voicevox_ros2 is not available. wait again")
    req = Speaker.Request()

    res = send_req(text, id)

# debug
if __name__ == '__main__':
    rclpy.init()
    node = Node('voicevox_ros2_sample')
    '''
    say(node, text='こんにちは！ボイスボックスロスへようこそ')
    say(node, text='キャラクターIDを変更すると、しゃべるキャラクターが変わります', id=2)
    say(node, text='新たなキャラクターID、すんだもん、ヘロヘロと、', id=75)
    say(node, text='すんだもん、なみだめ、', id=76)
    say(node, text='中国うさぎ、ノーマル', id=61)
    say(node, text='中国うさぎ、おどろき', id=62)
    say(node, text='中国うさぎ、こわがり', id=63)
    say(node, text='中国うさぎ、へろへろ', id=64)
    '''
    say(node, text='これはテストです', id=74)
