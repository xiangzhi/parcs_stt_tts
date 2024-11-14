import os 
import tempfile

import openai
import numpy as np 
import sounddevice as sd
from pydub import AudioSegment
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String 

from parcs_stt_tts_msgs.action import Listen
from parcs_stt_tts_msgs.action import Recalibrate

class ParcsSTT(Node):

    def __init__(self):
        '''
        publisher 
        ---------
        '/speech_to_text' | produced STT result is published to this topic
        
        actions
        -------
        'listen' | listens once and returns speech as text after a pause period 
        'recalibrate' | recalibrates the node for background noise

        parameters
        ----------
        'relative_threshold' | a value that is added to the silence threshold value after calibration | default: 0.5
        'set_threshold' | the silence threshold as a specified value; 0.0 means auto-calibration is used | default: 0.0
        'interpreter' | the recognizer for speech | default: 'openai'
        'pause_duration' | the amount of time of a pause in seconds to begin processing audio; the audio chunk length | default: 2.0
        'microphone' | the microphone used to calibrate and record audio, default if it cannot be found | default: 'default'

        note for thresholds: louder sounds yield a higher dBFS value; lower thresholds are less sensitive to noise
        '''
        super().__init__('parcs_stt')

        # handle publisher
        self._publisher = self.create_publisher(String, "/speech_to_text", 10)

        # create action servers
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

        # set parameters
        rel_threshold_param = 0.5 
        set_threshold_param = 0.0
        stt_interpreter_param = 'openai'
        pause_duration_param = 2.0
        mic_param = 'default'

        self.declare_parameter("relative_threshold", rel_threshold_param)
        self.declare_parameter("set_threshold", set_threshold_param)
        self.declare_parameter("interpreter", stt_interpreter_param)
        self.declare_parameter("pause_duration", pause_duration_param)
        self.declare_parameter("microphone", mic_param)

        self.set_threshold_param = self.get_parameter("set_threshold").get_parameter_value().double_value
        self.rel_threshold_param = self.get_parameter("relative_threshold").get_parameter_value().double_value
        self.stt_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value
        self.pause_duration_param = self.get_parameter("pause_duration").get_parameter_value().double_value
        self.mic_param = self.get_parameter("microphone").get_parameter_value().string_value

        # adjust microphone settings
        if self.mic_param == 'default':
            self.get_logger().info(f"Using default microphone.")
            self.mic_index = None
            mic_device = sd.query_devices(sd.default.device[0], 'input')  
        else:
            self.mic_index = self.__get_mic_index(self.mic_param)
            mic_device = sd.query_devices(self.mic_index, 'input') 

        self.sample_rate = int(mic_device['default_samplerate'])
        self.get_logger().info(f"Microphone sample rate: {self.sample_rate}")

        # adjust threshold
        if self.set_threshold_param != 0.0:
            self.get_logger().info(f"Threshold set to {self.set_threshold_param}. Not calibrating.")
        else:
            self.get_logger().info(f"Change in threshold after calibration: {self.rel_threshold_param}")

        self.set_background_noise()

        self.get_logger().info(f"Pause/chunk duration: {self.pause_duration_param}")

        # handle STT interpreter
        self.get_logger().info(f"STT Interpreter: {self.stt_interpreter_param}")

        if self.stt_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable

        # other variables
        self.pauses_detected = 0

        self.get_logger().info(f"STT node ready.\n--------------------------------------------")
    
    '''gets the microphone index; None if default'''
    def __get_mic_index(self, mic_name):
        mics = sd.query_devices() # print this to see device names
        for i, mic in enumerate(mics):
            if mic['max_input_channels'] > 0 and mic_name in mic['name']:
                self.get_logger().info(f"Using {mic_name} at index {i}.")
                return i
        self.get_logger().info(f"Couldn't find microphone {mic_name}, using default.")
        return None  # return none if no microphone found
    
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
        if self.set_threshold_param == 0.0:
            self.set_background_noise()
        else:
            self.get_logger().info("Cannot recalibrate when given a set threshold.")
        
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
        if self.set_threshold_param != 0.0:
            self.background_noise_dbfs = 0.0
            self.silence_threshold = self.set_threshold_param
        else:
            self.background_noise_dbfs = self.measure_background_noise()
            self.silence_threshold = self.background_noise_dbfs + self.rel_threshold_param 
            self.get_logger().info(f"Base silence threshold (dBFS): {self.background_noise_dbfs}")
            self.get_logger().info(f"Silence threshold with added parameter (dBFS): {self.silence_threshold}")

    '''measure the background noise to adjust the silence threshold'''
    def measure_background_noise(self, duration=3, samplerate=44100):
        self.get_logger().info("Calibrating for background noise...")
        audio_chunk = sd.rec(
            int(duration * self.sample_rate), 
            samplerate=self.sample_rate, 
            channels=1, 
            dtype='int16',
            device=self.mic_index
        )
        sd.wait()

        audio_segment = AudioSegment(
            audio_chunk.tobytes(),
            frame_rate=self.sample_rate,
            sample_width=audio_chunk.dtype.itemsize,
            channels=1
        )

        dBFS_value = audio_segment.dBFS
        self.get_logger().info(f"Background noise dBFS value: {dBFS_value}")
        return dBFS_value
    

    def detect_audio_activity(self, cur_dbfs) -> bool:
        self.get_logger().info(f"current dbfs:{cur_dbfs}: {cur_dbfs > self.silence_threshold}")

        return cur_dbfs > self.silence_threshold



    '''detects if there is a pause in audio'''
    def detect_audio_silence(self, cur_dbfs, prev_dbfs, started_speaking):
        self.get_logger().info(f"current dbfs:{cur_dbfs}")
        # good for starting speaking, not good for running because it won't have significant changes
        if not started_speaking:
            if abs(cur_dbfs - prev_dbfs) >= 5.0:
                started_speaking = True
                self.get_logger().info("Speech detected.")
                return False, started_speaking # no silence
            else:
                return True, started_speaking  # silence
        else:
            if abs(cur_dbfs - self.silence_threshold) >= 2.0: # tentative value for the dbfs to be between
                return False, started_speaking # no silence
            else: 
                return True, started_speaking # silence

    '''records the audio until silence is detected and returns the audio file'''
    def record_audio(self, threshold, samplerate=44100, chunk_duration=0.5): # chunk duration was 1
        self.get_logger().info('Recording...')
        audio_chunks = []
        started_speaking = False
        prev_dbfs = threshold

        while True:

            diff_chunk_duration = chunk_duration * 2.5 if not started_speaking else chunk_duration

            # record a chunk of audio
            audio_chunk = sd.rec(
                int(diff_chunk_duration * self.sample_rate), 
                samplerate=self.sample_rate, 
                channels=1, 
                dtype='int16',
                device=self.mic_index
            )
            # wait until audio is done recording
            sd.wait() 

            # pause detection for audio to end
            audio_segment = AudioSegment(
                audio_chunk.tobytes(),
                frame_rate=self.sample_rate,
                sample_width=audio_chunk.dtype.itemsize,
                channels=1
            )

            current_dbfs = audio_segment.dBFS
            
            # uncomment to check dbfs values for debugging
            # self.get_logger().info(f"Measured segment's dBFS: {current_dbfs}")

            # detects any silence in the audio segment
            # silence_detected, started_speaking = self.detect_audio_silence(current_dbfs, prev_dbfs, started_speaking)

            # if started_speaking:
            #     if silence_detected:
            #         self.pauses_detected += 1
            #         self.get_logger().info(f"Pause detected. Current pause counter: {self.pauses_detected}")
            #     else:
            #         self.get_logger().info("No pause detected. Adding audio segment and restart counter")
            #         self.pauses_detected = 0
            # else:
            #     self.get_logger().info("No speech detected. Adding audio segment...")
            #     prev_dbfs = current_dbfs

            if started_speaking:
                audio_chunks.append(audio_segment) 
                # detect audio level
                if self.detect_audio_activity(current_dbfs):
                    self.get_logger().info("No pause detected.")
                    # restart counter
                    self.pauses_detected = 0
                else:
                    self.get_logger().info("pause detected.")
                    if self.pauses_detected >= self.pause_duration_param:
                        self.get_logger().info("Reached pause duration. Stopping detection...")
                        self.pauses_detected = 0
                        break
                    self.pauses_detected += 1
            else:
                if self.detect_audio_activity(current_dbfs):
                    self.get_logger().info("speech detected. Adding audio segment...")
                    started_speaking = True
                    self.pauses_detected = 0
                    audio_chunks.append(audio_segment) 

                
            # OLD LOGIC
            # if silence_detected and started_speaking:
            #     self.pauses_detected += 1
            #     self.get_logger().info(f"Pause detected. Current pause counter: {self.pauses_detected}")
            # else:
            #     if not started_speaking:
            #         self.get_logger().info("No speech detected. Adding audio segment...")
            #     else:
            #         self.get_logger().info("No pause detected. Adding audio segment...")
            #     prev_dbfs = current_dbfs
            #     self.pauses_detected = 0
            
            # audio_chunks.append(audio_segment) 

            # # stops recording if pause is detected 
            # if self.pauses_detected >= self.pause_duration_param:
            #     self.get_logger().info("Reached pause duration. Stopping detection...")
            #     self.pauses_detected = 0
            #     break

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
                    file=file,
                    language='en'
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