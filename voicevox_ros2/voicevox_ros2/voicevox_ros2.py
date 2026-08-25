#!/usr/bin/env python3
# Copyright (c) 2026 Nakatogawa Laboratory. All rights reserved.

from array import array
import subprocess

from rclpy.node import Node
import rclpy
from voicevox_core import StyleNotFoundError
from voicevox_core.blocking import Onnxruntime, OpenJtalk, Synthesizer, VoiceModelFile
from voicevox_ros2_msgs.srv import Speaking


class VoiceVoxRos2(Node):
    def __init__(self):
        super().__init__('voicevox_ros2')

        self.declare_parameter('voicevox_onnxruntime_path', '/ws/voicevox_core/onnxruntime/lib/')
        self.declare_parameter('voicevox_model_path', '/ws/voicevox_core/models/vvms/0.vvm')
        self.declare_parameter(
            'open_jtalk_dict_dir',
            '/ws/voicevox_core/dict/open_jtalk_dic_utf_8-1.11'
        )
        self.declare_parameter('use_aplay', False)

        voicevox_onnxruntime_path = (
            self.get_parameter('voicevox_onnxruntime_path').value
            + Onnxruntime.LIB_RECOMMENDED_VERSIONED_FILENAME
        )
        voicevox_model_path = self.get_parameter('voicevox_model_path').value
        open_jtalk_dict_dir = self.get_parameter('open_jtalk_dict_dir').value

        self.get_logger().info(
            'voicevox_core parameters:\n  %s\n  %s\n  %s' % (
                voicevox_onnxruntime_path, voicevox_model_path, open_jtalk_dict_dir
            )
        )

        self.synthesizer = Synthesizer(
            Onnxruntime.load_once(filename=voicevox_onnxruntime_path),
            OpenJtalk(open_jtalk_dict_dir)
        )
        with VoiceModelFile.open(voicevox_model_path) as model:
            self.get_logger().info('load model: %s' % voicevox_model_path)
            self.synthesizer.load_voice_model(model)

        self.speaking_service = self.create_service(
            Speaking,
            'speak',
            self.speaking_cb
        )

        self.get_logger().info('VoiceVox ROS2 Start!')

    def speaking_cb(self, req: Speaking.Request, res: Speaking.Response):
        speaker_id = req.speaker_id
        text = req.text
        pitch_scale = req.pitch_scale
        intonation_scale = req.intonation_scale
        speed_scale = req.speed_scale
        volume_scale = req.volume_scale
        enable_interrogative_upspeak = req.enable_interrogative_upspeak
        res.success = False
        res.wav_data = array('B')
        wav = None
        use_aplay = self.get_parameter('use_aplay').value

        self.get_logger().info(
            'Request:\n  text: %s\n  speaker_id: %d'
            '  pitch_scale: %.2f  intonation_scale: %.2f'
            '  speed_scale: %.2f  volume_scale: %.2f' % (
                text, speaker_id, pitch_scale, intonation_scale, speed_scale, volume_scale
            )
        )

        try:
            if not text:
                self.get_logger().error('Empty text received for synthesis')
                res.success = False
                return res

            audio_query = self.synthesizer.create_audio_query(text, speaker_id)
            # Apply offset scales from request
            audio_query.pitch_scale += pitch_scale
            audio_query.intonation_scale += intonation_scale
            audio_query.speed_scale += speed_scale
            audio_query.volume_scale += volume_scale

            wav = self.synthesizer.synthesis(
                audio_query, speaker_id, enable_interrogative_upspeak=enable_interrogative_upspeak
            )
            if wav and len(wav) > 0:
                res.wav_data = array('B', wav)
                res.success = True
            else:
                self.get_logger().error('VoiceVox synthesis produced empty audio data')
                res.success = False
                return res

            if use_aplay:
                self.get_logger().info('Successfully generated speech! Playing with aplay...')
                command = ['aplay']
                proc = subprocess.Popen(command, stdin=subprocess.PIPE)
                proc.stdin.write(bytes(wav))
                proc.stdin.close()
                proc.wait()
                self.get_logger().info('Speaking Done!')

        except StyleNotFoundError as e:
            self.get_logger().error('Style not found: %s' % str(e))
            res.success = False
        except Exception as e:
            self.get_logger().error('VoiceVox synthesis exception: %s' % str(e))
            res.success = False

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


if __name__ == '__main__':
    main()
