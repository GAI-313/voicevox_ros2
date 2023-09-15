
#!/usr/bin/env python3
## ROS2
import message_filters
import rclpy
from rclpy.node import Node
from voicevox_ros2_interface.msg import Speaker 
from voicevox_ros2_interface import srv
from std_msgs.msg import String
## VOICEVOX LIBS
import dataclasses
import json
import logging
import re
from argparse import ArgumentParser
from pathlib import Path
from typing import Tuple

import ctypes
#ctypes.cdll.LoadLibrary('/voicevox/onnxruntime-linux-x64-1.13.1/lib/libonnxruntime.so')

import voicevox_core
from voicevox_core import AccelerationMode, AudioQuery, VoicevoxCore

# Other pkgs
import os
import time
#from playsound import playsound
#import sounddevice as sd
#import soundfile as sf
import simpleaudio
import threading

class Voicevox_ros2(Node):
    def __init__(self):
        super().__init__("voicevox_ros2_core")
        self.get_logger().info("start voicevox_ros2 ")

        self.text = self.id = None

        # 英語辞書作成
        self.get_logger().info(os.getenv('KANAENG_PATH'))
        self.dict_path = os.getenv('KANAENG_PATH')
        self.dict = {}
        with open(self.dict_path, mode='r', encoding='utf-8') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if i >= 6:
                    line_list = line.replace('\n', '').split(' ')
                    self.dict[line_list[0]] = line_list[1]
        # 短縮形
        self.reduction=[["It\'s","イッツ"],["I\'m","アイム"],["You\'re","ユーァ"],["He\'s","ヒーィズ"],["She\'s","シーィズ"],["We\'re","ウィーアー"],["They\'re","ゼァー"],["That\'s","ザッツ"],["Who\'s","フーズ"],["Where\'s","フェアーズ"],["I\'d","アイドゥ"],["You\'d","ユードゥ"],["I\'ve","アイブ"],["I\'ll","アイル"],["You\'ll","ユール"],["He\'ll","ヒール"],["She\'ll","シール"],["We\'ll","ウィール"]]

        self.status = 'launch'
        self.string = String()

        self.result_pub = self.create_publisher(String, "voicevox_ros2/status", 10)
        self.sub = self.create_subscription(Speaker, "voicevox_ros2/speaker", self.msg_cb, 10)
        self.srv = self.create_service(srv.Speaker, "voicevox_ros2/speaker_srv", self.srv_cb)

        # self.pub_timer = self.create_timer(0.005, self.state_publish)
        self.pub_thread = threading.Thread(target=self.state_publish) # 別スレッドで state_publish を実行
        self.pub_thread.start()

    def __del__(self):
        self.status = 'finish'
        self.get_logger().info("done.")

    def srv_cb(self, req, res):
        self.status = "getsrv"
        self.generate_voice(req.text, req.id)
        res.success = True
        return res

    def msg_cb(self, msg):
        self.status = "gettopic"
        self.generate_voice(msg.text, msg.id)

    def eng_to_kana(self, text):
        # 読み上げ可能単語を変換
        text = text.replace("+"," プラス ").replace("＋"," プラス ").replace("-"," マイナス ").replace("="," イコール ").replace("＝"," イコール ")
        # No.2、No6みたいに、No.の後に数字が続く場合はノーではなくナンバーと読む
        text = re.sub(r'No\.([0-9])',"ナンバー\\1",text)
        text = re.sub(r'No([0-9])',"ナンバー\\1",text)
        # 短縮形の処理
        for red in self.reduction: text = text.replace(red[0]," "+red[1]+" ")
        # this is a pen.のように、aの後に半角スペース、続いてアルファベットの場合、エーではなくアッと呼ぶ
        text = re.sub(r'a ([a-zA-Z])',"アッ \\1",text)
        # 文を区切る文字は消してはダメなので、前後に半角スペースを挟む
        text = text.replace("."," . ").replace("。"," 。 ").replace("!"," ! ").replace("！"," ！ ")
        # アルファベットとアルファベット以外が近接している時、その間に半角スペースを挟む（この後、英単語を単語ごとに区切るための前準備）
        text_l=list(text)
        for i in range(len(text))[::-1][:-1]:
            if re.compile("[a-zA-Z]").search(text_l[i]) and re.compile("[^a-zA-Z]").search(text_l[i-1]): text_l.insert(i," ")
            elif re.compile("[^a-zA-Z]").search(text_l[i]) and re.compile("[a-zA-Z]").search(text_l[i+-1]): text_l.insert(i," ")
        # 半角スペースや読まなくて良い文字で区切り、各単語の英語をカタカナに変換
        text_split = re.split('[ \,\*\-\_\=\(\)\[\]\'\"\&\$　]',text)
        for i in range(len(text_split)):
            if str.upper(text_split[i]) in self.dict:
                text_split[i] = self.dict[str.upper(text_split[i])]

        return (" ".join(text_split))

    def generate_voice(self, text, speaker_id):
        try:
            self.state = "wait"
            jtalk_path = os.getenv('JTALK_PATH')
            home_path = os.getenv('HOME')
            
            generate_path = __file__.replace("run.py", "output.wav")

            # 英単語からカタカナに変換
            text = self.eng_to_kana(text)
            text = text.replace(" ", "")

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
            self.status = "done"
        except Exception as e:
            self.status = "error"
            print(e)

    def state_publish(self):
        status_b = ''
        while rclpy.ok():
            if self.status != status_b:
                #print(self.status)
                self.string.data = self.status
                self.result_pub.publish(self.string)
                
            status_b = self.status
            if self.status == 'done':
                self.status = 'wait'
            #rclpy.spin_once(self)

def main():
    try:
        rclpy.init()
        node = Voicevox_ros2()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()        
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
