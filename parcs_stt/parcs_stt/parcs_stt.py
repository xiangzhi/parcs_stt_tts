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

        # parameters
        threshold_param = -2
        # mic_param to be made later
        stt_interpreter_param = 'openai'

        self.declare_parameter("threshold", threshold_param)
        self.declare_parameter("interpreter", stt_interpreter_param)

        self.threshold_param = self.get_parameter("threshold").get_parameter_value().integer_value
        
        self.stt_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value

        self.get_logger().info(f"Threshold: {self.threshold_param}")
        self.get_logger().info(f"Interpreter: {self.stt_interpreter_param}")

        if self.stt_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable

        self.background_noise_dbfs = self.measure_background_noise()
        self.silence_threshold = self.background_noise_dbfs + self.threshold_param 
        self.get_logger().info(f"Silence threshold: {self.silence_threshold}")

    def ackCallback(self, msg):
        self.main()

    def measure_background_noise(self, duration=3, samplerate=44100):
        self.get_logger().info("Calibrating for background noise...")
        audio_chunk = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
        sd.wait()

        audio_segment = AudioSegment(
            audio_chunk.tobytes(),
            frame_rate=samplerate,
            sample_width=audio_chunk.dtype.itemsize,
            channels=1
        )

        dBFS_value = audio_segment.dBFS
        self.get_logger().info(f"Background noise dBFS value: {dBFS_value}")
        return dBFS_value
    
    def detect_audio_silence(self, audio_segment : AudioSegment, pause_duration, threshold):
        min_silence_len = 1000*pause_duration
    
        samples = np.array(audio_segment.get_array_of_samples())
        rms_value = np.sqrt(np.mean(np.square(samples)))

        # converting rms to dbfs
        rms_dBFS = 20 * np.log10(rms_value / 32768.0)
        self.get_logger().info(f"RMS Value: {rms_value}")
        self.get_logger().info(f"RMS to dBFS Value: {rms_dBFS}")
        return rms_dBFS < threshold

    def record_audio(self, threshold=-40.0, chunk_duration=2, pause_duration=2, samplerate=44100): # chunk duration was 1
        self.get_logger().info('Recording...')
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
            self.get_logger().info("No pause detected. Adding audio segment...")

            # detects any silence in the audio segment
            silence_detected = self.detect_audio_silence(audio_segment, pause_duration, threshold)
            if silence_detected:
                pause_counter += 1
            else:
                pause_counter = 0
            
            # stops recording if pause is detected
            if pause_counter > 0:
                self.get_logger().info("Pause detected. Stopping detection...")
                break

        # combines the audio chunks until the silence threshold was met
        combined_audio = sum(audio_chunks)

        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_audio_file:
            combined_audio.export(temp_audio_file.name, format="wav")
            self.get_logger().info("Recording complete.")
            return temp_audio_file.name
        
    def speech_to_text(self, audio_file):
        if (self.stt_interpreter_param == 'openai'):
            with open(audio_file, "rb") as file:
                response = openai.audio.transcriptions.create(
                    model='whisper-1',
                    file=file
                )
        else:
            self.get_logger().error("Not a valid STT interpreter (i.e. openai)")

        response_text = response.text
        self.get_logger().info(f"Query: {response_text}")
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