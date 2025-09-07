#!/uysr/bin/env python3
from rclpy.node import Node
import rclpy

from voicevox_ros2_msgs.srv import Speaking
from voicevox_core.blocking import Onnxruntime, OpenJtalk, Synthesizer, VoiceModelFile
from voicevox_core import StyleNotFoundError

import subprocess


class VoiceVoxRos2(Node):
    def __init__(self):
        super().__init__('voicevox_ros2')

        self.declare_parameter('voicevox_onnxruntime_path', '/ws/voicevox_core/onnxruntime/lib/')
        self.declare_parameter('voicevox_model_path', '/ws/voicevox_core/models/vvms/0.vvm')
        self.declare_parameter('open_jtalk_dict_dir', '/ws/voicevox_core/dict/open_jtalk_dic_utf_8-1.11')

        voicevox_onnxruntime_path = self.get_parameter('voicevox_onnxruntime_path').value + Onnxruntime.LIB_VERSIONED_FILENAME
        voicevox_model_path = self.get_parameter('voicevox_model_path').value
        open_jtalk_dict_dir = self.get_parameter('open_jtalk_dict_dir').value

        self.get_logger().info('voicevox_core parameters: \n    %s\n    %s\n    %s'%(voicevox_onnxruntime_path, voicevox_model_path, open_jtalk_dict_dir))

        self.synthesizer = Synthesizer(Onnxruntime.load_once(filename=voicevox_onnxruntime_path), OpenJtalk(open_jtalk_dict_dir))
        with VoiceModelFile.open(voicevox_model_path) as model:
            self.get_logger().info('load model: %s'%voicevox_model_path)
            self.synthesizer.load_voice_model(model)
        
        self.speaking_service = self.create_service(
            Speaking,
            'speak',
            self.speaking_cb
        )

        self.get_logger().info('VoiceVox ROS2 Start!')
    

    def speaking_cb(self, req:Speaking.Request, res:Speaking.Response):
        speaker_id = req.speaker_id
        text = req.text
        pitch_scale = req.pitch_scale
        intonation_scale = req.intonation_scale
        speed_scale = req.speed_scale
        volume_scale = req.volume_scale
        enable_interrogative_upspeak = req.enable_interrogative_upspeak
        res.success = False
        wav = None

        self.get_logger().info('Request:\n  text: %s\n  speaker_id: %d  pitch_scale: %d  intonation_scale: %d  speed_scale: %d  volume_scale: %d'%(text, speaker_id, pitch_scale, intonation_scale, speed_scale, volume_scale))
        
        try:
            audio_query = self.synthesizer.create_audio_query(text, speaker_id)
            audio_query.pitch_scale += pitch_scale
            audio_query.intonation_scale += intonation_scale
            audio_query.speed_scale += speed_scale
            audio_query.volume_scale += volume_scale
            wav = self.synthesizer.synthesis(audio_query, speaker_id, enable_interrogative_upspeak=enable_interrogative_upspeak)
            
            if wav:
                self.get_logger().info('Successfully generate speech !')
                self.get_logger().info('Speaking ...')

                command = [
                    'aplay',
                    '-f', 'S16_LE',
                ]
                proc = subprocess.Popen(command, stdin=subprocess.PIPE)
                proc.stdin.write(wav)
                proc.stdin.close()
                proc.wait()
                self.get_logger().info('Speaking Done !')
                res.success = True
        
        except StyleNotFoundError as e:
            self.get_logger().error(str(e))
        
        return res

def main():
    rclpy.init()
    node = VoiceVoxRos2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
