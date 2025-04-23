#!/home/sungjae/.conda/envs/dia/bin/python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import soundfile as sf
from dia.model import Dia
import threading
from collections import deque

class DiaNode:
    def __init__(self):
        rospy.init_node('dia_node')
        self.subscriber = rospy.Subscriber('input_text', String, self.text_callback)
        self.soundhandle = SoundClient()
        self.text_queue = deque()
        self.lock = threading.Lock()
        self.is_processing = False  # 현재 재생 중인지 상태 체크

        rospy.loginfo("Loading Dia model...")
        self.model = Dia.from_pretrained("nari-labs/Dia-1.6B")
        rospy.loginfo("Dia model loaded.")

        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_text)

    def text_callback(self, msg):
        with self.lock:
            self.text_queue.append(msg.data)
            rospy.loginfo(f"Text enqueued. Queue size: {len(self.text_queue)}")

    def process_text(self, event):
        with self.lock:
            if self.is_processing or not self.text_queue:
                return
            text = self.text_queue.popleft()
            self.is_processing = True

        rospy.loginfo("Generating audio...")
        try:
            output = self.model.generate(text, max_tokens=3072, cfg_scale=4.5)
            sf.write("/tmp/dia_output.wav", output, 44100)
            rospy.sleep(0.5)
            self.soundhandle.playWave("/tmp/dia_output.wav")
            rospy.loginfo("Audio sent to sound_play.")
        except Exception as e:
            rospy.logerr(f"Generation/playback error: {e}")
        finally:
            # Wait until soundplay finishes — simple blocking
            rospy.sleep(5.0)  # 기본 음성 길이에 따라 조절 필요
            with self.lock:
                self.is_processing = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DiaNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
