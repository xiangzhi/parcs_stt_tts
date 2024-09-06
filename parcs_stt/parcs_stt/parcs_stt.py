import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import sounddevice as sd
from pydub import AudioSegment
import numpy as np 
import tempfile
import openai
import os 

class ParcsSTT(Node):

    def __init__(self):
        super().__init__('parcs_stt')
        self._publisher = self.create_publisher(String, "/parcs_stt/chatbot", 10)
        self._subscription = self.create_subscription(String, "/parcs_tts/chatbot_ack", self.ackCallback, 10)
        self._subscription

        self.background_noise_dbfs = self.measure_background_noise()
        self.silence_threshold = self.background_noise_dbfs - 2 # param???????
        print(f"Silence threshold: {self.silence_threshold}")

    def ackCallback(self, msg):
        self.main()

    def measure_background_noise(self, duration=3, samplerate=44100):
        print("Calibrating for background noise...")
        audio_chunk = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
        sd.wait()

        audio_segment = AudioSegment(
            audio_chunk.tobytes(),
            frame_rate=samplerate,
            sample_width=audio_chunk.dtype.itemsize,
            channels=1
        )

        dBFS_value = audio_segment.dBFS
        print(f"Background noise dBFS value: {dBFS_value}")
        return dBFS_value
    
    def detect_audio_silence(self, audio_segment : AudioSegment, pause_duration, threshold):
        min_silence_len = 1000*pause_duration
    
        samples = np.array(audio_segment.get_array_of_samples())
        rms_value = np.sqrt(np.mean(np.square(samples)))

        # convertnig rms to dbfs
        rms_dBFS = 20 * np.log10(rms_value / 32768.0)
        print("RMS Value: ", rms_value)
        print("RMS to dBFS Value: ", rms_dBFS)
        return rms_dBFS < threshold

    def record_audio(self, threshold=-40.0, chunk_duration=2, pause_duration=2, samplerate=44100): # chunk duration was 1
        print('Recording...')
        audio_chunks = []
        pause_counter = 0

        while True:
            # record a chunk of audio
            audio_chunk = sd.rec(int(chunk_duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
            # wait until audio is done recording
            sd.wait() 

            # pause detection for audio to end
            audio_segment = AudioSegment(
                audio_chunk.tobytes(),
                frame_rate=samplerate,
                sample_width=audio_chunk.dtype.itemsize,
                channels=1
            )

            audio_chunks.append(audio_segment)
            print("No pause detected. Adding audio segment...")

            # detects any silence in the audio segment
            silence_detected = self.detect_audio_silence(audio_segment, pause_duration, threshold)
            if silence_detected:
                pause_counter += 1
            else:
                pause_counter = 0
            
            # stops recording if pause is detected
            if pause_counter > 0:
                print("Pause detected. Stopping detection...")
                break

        # combines the audio chunks until the silence threshold was met
        combined_audio = sum(audio_chunks)

        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_audio_file:
            combined_audio.export(temp_audio_file.name, format="wav")
            print("Recording complete.")
            return temp_audio_file.name
        
    def speech_to_text(self, audio_file):
        with open(audio_file, "rb") as file:
            response = openai.audio.transcriptions.create(
                model='whisper-1',
                file=file
            )

        response_text = response.text
        print("Query: ", response_text)
        return response_text
    
    def main(self):
        audio_file = self.record_audio(threshold=self.silence_threshold)

        text = String()
        text.data = self.speech_to_text(audio_file)

        self._publisher.publish(text)

        os.remove(audio_file)

def main(args=None):
    rclpy.init(args=args)

    node = ParcsSTT()
    node.main()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()