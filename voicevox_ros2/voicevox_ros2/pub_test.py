#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.msg import Speaker

import threading

def main():
    rclpy.init()
    node = Node("voicevox_ros2_pub_test")
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    sp = Speaker()
    sp.text = "こんにちは。ボイスボックスロスツー、パブテストからの通信です。"
    sp.id = 3

    sp_pub = node.create_publisher(Speaker, "/voicevox_ros2/speaker", 10)

    try:
        while rclpy.ok():
            num = sp_pub.get_subscription_count()
            if num > 0:
                sp_pub.publish(sp)
                break
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
