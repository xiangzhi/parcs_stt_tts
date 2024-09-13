import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import sounddevice as sd
from pydub import AudioSegment
import numpy as np 
import tempfile
import openai
import os 
from rclpy.action import ActionServer
from parcs_stt_tts_msgs.action import Listen
from parcs_stt_tts_msgs.action import Recalibrate

class ParcsSTT(Node):

    def __init__(self):
        super().__init__('parcs_stt')

        self._publisher = self.create_publisher(String, "/speech_to_text", 10)

        self._stt_action_server = ActionServer(
            self,
            Listen,
            'listen',
            self.listen_callback
        )

        self._recalibrate_action_server = ActionServer(
            self,
            Recalibrate,
            'recalibrate',
            self.recalibrate_callback
        )

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

        self.set_background_noise()
    
    '''listen action server callback'''
    def listen_callback(self, goal_handle):
        self.get_logger().info("Executing listening goal...")
        
        self._goal_handle = goal_handle

        result = Listen.Result()
        result.transcript = self.record_and_produce_text()
        goal_handle.succeed()
        return result
    
    '''recalibration action server callback'''
    def recalibrate_callback(self, goal_handle):
        self.get_logger().info("Executing recalibrate goal...")

        self._goal_handle = goal_handle

        result = Recalibrate.Result()
        self.set_background_noise()
        result.dbfs = self.background_noise_dbfs
        result.threshold = self.silence_threshold
        goal_handle.succeed()
        return result

    '''
    measures and sets the background noise to adjust the silence threshold
    measurement is in dBFS (decibels relative to full scale) which describes
    the amplitude of a signal relative to the maximum possible level in digital
    audio systems
    '''
    def set_background_noise(self):
        self.background_noise_dbfs = self.measure_background_noise()
        self.silence_threshold = self.background_noise_dbfs + self.threshold_param 
        self.get_logger().info(f"Silence threshold (dBFS): {self.silence_threshold}")

    '''measure the background noise to adjust the silence threshold'''
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
    
    '''detects if there is a pause in audio'''
    def detect_audio_silence(self, audio_segment : AudioSegment, pause_duration, threshold):
        min_silence_len = 1000*pause_duration # pause_duration in seconds
    
        samples = np.array(audio_segment.get_array_of_samples())
        rms_value = np.sqrt(np.mean(np.square(samples)))

        # converting rms to dbfs
        rms_dBFS = 20 * np.log10(rms_value / 32768.0)
        # uncomment to test for RMS and dBFS threshold values
        # self.get_logger().info(f"RMS Value: {rms_value}")
        # self.get_logger().info(f"RMS to dBFS Value: {rms_dBFS}")
        return rms_dBFS < threshold
    
    '''records the audio until silence is detected and returns the audio file'''
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
    
    '''interprets the given audio file into text'''
    def interpret_audio(self, audio_file):
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
    
    '''handles recording, producing text as a string type, and publishing a String to the topic'''
    def record_and_produce_text(self):
        audio_file = self.record_audio(threshold=self.silence_threshold)

        text = String()
        text.data = self.interpret_audio(audio_file)

        self._publisher.publish(text)

        # os.remove(audio_file)

        return text.data

def main(args=None):
    rclpy.init(args=args)

    node = ParcsSTT()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()