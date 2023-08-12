#!/usr/bin/env python3
## ROS2
import message_filters
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.msg import Speaker 
from voicevox_ros2_interface import srv 
## VOICEVOX LIBS
import dataclasses
import json
import logging
from argparse import ArgumentParser
from pathlib import Path
from typing import Tuple

import voicevox_core
from voicevox_core import AccelerationMode, AudioQuery, VoicevoxCore

# Other pkgs
import os
import time
#from playsound import playsound
#import sounddevice as sd
#import soundfile as sf
import simpleaudio

class Voicevox_ros2(Node):
    def __init__(self):
        super().__init__("voicevox_ros2_core")
        self.get_logger().info("start voicevox_ros2 ")

        self.text = self.id = None

        self.sub = self.create_subscription(Speaker, "voicevox_ros2/speaker", self.msg_cb, 10)
        self.srv = self.create_service(srv.Speaker, "voicevox_ros2/speaker_srv", self.srv_cb)

    def __del__(self):
        self.get_logger().info("done.")

    def srv_cb(self, req, res):
        self.generate_voice(req.text, req.id)
        res.success = True
        return res

    def msg_cb(self, msg):
        self.generate_voice(msg.text, msg.id)

    def generate_voice(self, text, speaker_id):
        try:
            jtalk_path = os.getenv('JTALK_PATH')
            home_path = os.getenv('HOME')
            generate_path = home_path + "/colcon_ws/src/voicevox_ros2/voicevox_ros2/voicevox_ros2/output.wav"

            out = Path(generate_path)
            acceleration_mode = AccelerationMode.AUTO
            core = VoicevoxCore(
                acceleration_mode=acceleration_mode, open_jtalk_dict_dir=jtalk_path
            )
            core.load_model(speaker_id)
            audio_query = core.audio_query(text, speaker_id)
            wav = core.synthesis(audio_query, speaker_id)
            out.write_bytes(wav)

            self.get_logger().info("GENERATE voice")
            #sig, sr = sf.read(generate_path, always_2d=True)
            #sd.play(sig, sr)
            wav_obj = simpleaudio.WaveObject.from_wave_file(generate_path)
            play_obj = wav_obj.play()
            play_obj.wait_done()
            os.remove(generate_path)
            #self.speaker_id = self.text = None
        except Exception as e:
            print(e)

def main():
    try:
        rclpy.init()
        node = Voicevox_ros2()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()        
    except KeyboardInterrupt:
        pass
"""
def parse_args() -> Tuple[AccelerationMode, Path, str, Path, int]:
    #home_path = os.getenv('HOME')
    jtalk_path = os.getenv('JTALK_PATH')

    if jtalk_path is None:
        print("ERROR! env JTALK_PATH is not exist. please define it")
        os._exit(-1)

    argparser = ArgumentParser()
    argparser.add_argument(
        "--mode",
        default="AUTO",
        type=AccelerationMode,
        help='モード ("AUTO", "CPU", "GPU")',
    )
    argparser.add_argument(
        "--dict-dir",
        default= jtalk_path,
        type=Path,
        help="Open JTalkの辞書ディレクトリ",
    )
    argparser.add_argument(
        "--text",
        default="この音声は、ボイスボックスロスツーを使用して、出力されています。",
        help="読み上げさせたい文章",
    )
    argparser.add_argument(
        "--out",
        default="./output.wav",
        type=Path,
        help="出力wavファイルのパス",
    )
    argparser.add_argument(
        "--speaker-id",
        default=0,
        type=int,
        help="話者IDを指定",
    )
    args = argparser.parse_args()
    return (args.mode)
"""

def display_as_json(audio_query: AudioQuery) -> str:
    return json.dumps(dataclasses.asdict(audio_query), ensure_ascii=False)

if __name__ == "__main__":
    main()
