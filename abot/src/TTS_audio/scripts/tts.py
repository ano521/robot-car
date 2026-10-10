#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import torchaudio
import torch
import os
from ChatTTS import ChatTTS

# 全局变量用于存储ChatTTS实例和参数
chat = None
params_infer_code = None

def text_callback(msg):
    global chat, params_infer_code
    texts = [msg.data]
    wavs = chat.infer(texts, params_infer_code=params_infer_code)
    torchaudio.save("/media/abot/999/abot/src/TTS_audio/output/output_d1.wav", torch.from_numpy(wavs[0]), 24000)
    os.system('mplayer %s' % "/media/abot/999/abot/src/TTS_audio/output/output_d1.wav")

def main():
    global chat, params_infer_code
    
    # 初始化ROS节点
    rospy.init_node('chat_tts_node', anonymous=True)
    
    # 初始化ChatTTS
    chat = ChatTTS.Chat()
    chat.load_models(compile=False)  # 设置为False获得更快速度，设置为True获得更佳效果
    speaker = torch.load('/media/abot/999/abot/src/TTS_audio/speaker/speaker_5_girl.pth', map_location=torch.device('cpu'))

    params_infer_code = {
        'spk_emb': speaker,  # 讲话人
        'prompt': '[speed_10]',
        'temperature': 0.3,
        'top_P': 0.7,
        'top_K': 20,
    }
    
    # 创建订阅者
    rospy.Subscriber('text_to_speech', String, text_callback)
    rospy.loginfo("TTS节点启动成功")
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
