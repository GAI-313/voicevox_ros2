#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.srv import Speaker

import threading

def send_req(text, id):
    req.text = text
    req.id = id
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()
    
def main():
    global req, cli, node

    text = "ボイスボックスロスツー、サービスからの通信です。"
    id = 3

    rclpy.init()
    node = Node("voicevox_ros2_srv")
    node.get_logger().info("start voicevox srv test")
    # soin
    #thread = thread.Threading(target=rclpy.spin, arg=(node,), daemon=True)
    #thread.start()

    rate = node.create_rate(10)

    # create client
    cli = node.create_client(Speaker, "voicevox_ros2/speaker_srv")
    while not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn("voicevox_ros2 is not available. wait again")
    req = Speaker.Request()

    res = send_req(text, id)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
