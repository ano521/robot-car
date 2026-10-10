#!/usr/bin/env python3

#coding: utf-8
import rospy
import pyaudio
import wave
import os
from faster_whisper import WhisperModel
from std_msgs.msg import String
music_path="/media/abot/999/abot/src/whisper/mp3/start_record.mp3"
music1_path="/media/abot/999/abot/src/whisper/mp3/end_record.mp3"
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

def audio_callback(msg):
    rospy.loginfo("Received audio message, starting recording and recognition")
    start_audio()
    rospy.loginfo("Recording and recognition completed")

def start_audio(time=5, save_file="/media/abot/999/abot/src/whisper/mp3/test.wav"):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 16000
    RECORD_SECONDS = time  # 需要录制的时间
    WAVE_OUTPUT_FILENAME = save_file  # 保存的文件名

    p = pyaudio.PyAudio()  # 初始化
    rospy.loginfo("ON")
    os.system('mplayer %s' % music_path)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)  # 创建录音文件
    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)  # 开始录音

    rospy.loginfo("OFF")
    os.system('mplayer %s' % music1_path)

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')  # 保存
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    rospy.loginfo("Starting recognition")
    model_size = "/media/abot/999/abot/src/whisper/whisper-tiny-zh-ct2-int8"
    model = WhisperModel(model_size, device="cpu", compute_type="int8")
    segments, info = model.transcribe("/media/abot/999/abot/src/whisper/mp3/test.wav", beam_size=5, language='zh')

    rospy.loginfo("Detected language '%s' with probability %f", info.language, info.language_probability)

    for segment in segments:
        rospy.loginfo("[%.2fs -> %.2fs] %s", segment.start, segment.end, segment.text)

def audio_subscriber():
    rospy.init_node('audio_subscriber', anonymous=True)
    rospy.Subscriber("audio_topic", String, audio_callback)
    rospy.loginfo("Audio subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    audio_subscriber()
