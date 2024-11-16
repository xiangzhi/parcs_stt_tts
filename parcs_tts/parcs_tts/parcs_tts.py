import os 
import time 
import signal
import subprocess
import threading

import openai
import pyaudio
import sounddevice as sd
import numpy as np
from scipy.signal import resample

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String 
from action_msgs.msg import GoalStatus

from parcs_stt_tts_msgs.action import TTS
from parcs_stt_tts_msgs.srv import Stop

class ParcsTTS(Node):

    def __init__(self):
        '''
        actions
        -------
        'tts' | initiates the text to speech with the goal message

        services
        --------
        'stop' | stops the text to speech immediately when called

        parameters
        ----------
        'personality' | the personality of the AI if a response is being automatically generated | default: 'you like humans most of the time and are a helpful robot'
        'interpreter' | what is used to make the text into speech | default: 'festival'
            Options (case sensitive): 'openai', 'festival'
        'gen_response' | whether an AI response will be generated with the given goal message as a query | default: 'false'
            Options (case sensitive): 'true','false'
        'speaker' | the speaker that tts is played from, default if it cannot be found | default: 'default'

        '''
        super().__init__('parcs_tts')

        # action and service server initializing
        self.tts_callback_group = MutuallyExclusiveCallbackGroup() # handles one callback at a time
        self.stop_callback_group = MutuallyExclusiveCallbackGroup()

        self._tts_action_server = ActionServer(
            self,
            TTS,
            'tts',
            self.tts_callback,
            callback_group=self.tts_callback_group,
            cancel_callback=self.abort_goal_req
        )
        
        # parameters
        tts_interpreter_param = 'festival' # 'festival' or 'openai', case sensitive
        speaker_param = 'default'
        gen_response_param = 'false' # must be 'true' or 'false', case sensitive
        personality_param = 'you like humans most of the time and are a helpful robot'

        self.declare_parameter("interpreter", tts_interpreter_param)
        self.declare_parameter("speaker", speaker_param)
        self.declare_parameter("gen_response", gen_response_param)
        self.declare_parameter("personality", personality_param)

        self.tts_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value
        self.speaker_param = self.get_parameter("speaker").get_parameter_value().string_value
        self.gen_response_param = self.get_parameter("gen_response").get_parameter_value().string_value
        self.personality_param = self.get_parameter("personality").get_parameter_value().string_value

        # handle TTS interpreter
        self.get_logger().info(f"TTS interpreter: {self.tts_interpreter_param}")
        self.get_logger().info(f"Generating response: {self.gen_response_param}")

        if self.tts_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable
            self.speaker = "alloy"

        # adjust speaker settings
        p = pyaudio.PyAudio()
        if self.speaker_param == 'default':
            self.get_logger().info(f"Using default speaker.")
            self.speaker_index = p.get_default_output_device_info()['index']
            speaker_device = p.get_default_output_device_info()
        else:
            self.speaker_index = self.__get_speaker_index(self.speaker_param)
            
            # handle the default case
            if self.speaker_index is None:
                self.speaker_index = p.get_default_output_device_info()['index']
                speaker_device = p.get_default_output_device_info()
            else:
                speaker_device = p.get_device_info_by_index(self.speaker_index) 

        self.sample_rate = int(speaker_device['defaultSampleRate'])
        self.get_logger().info(f"Speaker sample rate: {self.sample_rate}")
    
        # variables
        self.stop_flag = False
        self.tts_process = None # for festival or other interpreters that would use subprocess.Popen
        self.tts_goal = None
        self.prod_speech_result = None
        self._goal_handle = None

        self.get_logger().info(f"TTS node ready.\n--------------------------------------------")
    
    '''gets the speaker index; None if not found'''
    def __get_speaker_index(self, speaker_name):
        p = pyaudio.PyAudio()
        speakers = p.get_device_count() # print this to see device names
        for i in range(speakers):
            speaker = p.get_device_info_by_index(i)
            if speaker['maxOutputChannels'] > 0 and speaker_name in speaker['name']:
                self.get_logger().info(f"Using {speaker_name} at index {i}.")
                return i
        self.get_logger().info(f"Couldn't find speaker {speaker_name}, using default.")
        return None  # return none if no speaker found

    '''tts action server callback'''
    def tts_callback(self, goal_handle):
        self.get_logger().info("Executing TTS goal...")

        # variable reset before each round of TTS
        self.prod_speech_result = None
        self.stop_flag = False

        # goal handle processing
        self._goal_handle = goal_handle
        self.tts_goal = goal_handle 

        goal = self._goal_handle.request
        tts_string = goal.tts 

        self.get_logger().info(f"Received TTS goal: {tts_string}")

        # generates responses if desired
        if self.gen_response_param == 'true':
            response = self.generate_response(goal.tts)
            tts_string = response

        # escapes apostrophes and quotations for processing
        tts_string.replace("'", "\\'")
        tts_string.replace('"', '\\"')

        # establishes the result
        result = TTS.Result()
        result.msg = '' # empty as placeholder value
        result.stopped = False # false until manually stopped

        try:
            # thread that handles all text to speech 
            tts_thread = threading.Thread(target=self.produce_speech, args=(tts_string,))
            # thread that handles waiting for manual stop signal or automatic stopping
            wait_for_stop_thread = threading.Thread(target=self.wait_for_stop_signal)
            tts_thread.start()
            wait_for_stop_thread.start()

            # waiting for a manual stop or the TTS to finish
            wait_for_stop_thread.join()

            goal_status = goal_handle.status

            if (goal_status in [GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED]):
                # finished TTS
                tts_thread.join()
                result.msg = self.prod_speech_result
                self.get_logger().info('Finished TTS.') 
                goal_handle.succeed()
            elif (goal_status == GoalStatus.STATUS_ABORTED):
                # manually stopped TTS
                stop_event = threading.Event()
                stop_event.set()
                tts_thread.join()
                result.stopped = True # manually stopped
                self.get_logger().info("TTS aborted.")
            else:
                pass 
        except Exception as e:
            self.stop_flag = True
            self.get_logger().error(f'Error occurred: {e}')
            if goal_status not in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                goal_handle.abort()
        return result
    
    '''waits until the stop flag is set to true'''
    def wait_for_stop_signal(self):
        while not self.stop_flag:
                time.sleep(1)

    '''handles the service call'''
    def abort_goal_req(self, request):

        self.get_logger().info("Aborting TTS goal...")

        # handles killing/aborting asynchronously 
        stop_thread = threading.Thread(target=self.handle_abort)
        stop_thread.start()
        stop_thread.join()

        return rclpy.actiopn.server.CancelResponse.ACCEPT
    
    '''aborts/kills all text to speech processses'''
    def handle_abort(self):
        if self.stop_flag: 
            self.get_logger().info("Already attempted to stop. Ignoring request.")
            return
        
        if not self.tts_goal: 
            self.get_logger().info("No TTS to stop.")
            return
        
        if self.tts_goal.status == GoalStatus.STATUS_EXECUTING:
            # kills tts if handled in a subprocess
            if self.tts_process is not None: 
                os.killpg(os.getpgid(self.tts_process.pid), signal.SIGTERM)
                self.get_logger().info("Killed process.")
                self.tts_process = None

            self.stop_flag = True

            self.get_logger().info("TTS stopped by service.")
            if self.tts_goal.status not in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                self.tts_goal.abort()
        else:
            self.get_logger().info("TTS not currently executing.")

    '''produces speech given the message and interpreter, plays it from speakers, and publishes the produced message as a String to the topic'''
    def produce_speech(self, msg):
        if self.tts_interpreter_param == 'openai':
            self.text_to_speech_openai(msg)
        elif self.tts_interpreter_param == 'festival':
            self.text_to_speech_festival(msg)
        else:
            self.get_logger().error("Invalid TTS interpreter. Choose a valid one (i.e. openai, festival)")

        done_msg = String()
        done_msg.data = msg

        self.prod_speech_result = msg

        # finished speaking; stop set to true
        self.stop_flag = True

        return msg
    
    '''generates a response via AI if parameters allow for it'''
    def generate_response(self, query):
        
        if self.tts_interpreter_param == 'openai':
            response = openai.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.personality_param},
                    {"role": "user", "content": query},
                ],
            )
            response_data = response.choices[0].message.content
            print("AI Response: ", response_data)
        else:
            self.get_logger().info("Could not generate a response; must specify an interpreter with that capability.")
            return

        return response_data
    
    '''uses openai to process the text and say it'''
    def text_to_speech_openai(self, text):
        p = pyaudio.PyAudio()

        player_stream = p.open(
            format=pyaudio.paInt16, 
            channels=1, 
            rate=self.sample_rate, 
            output=True,
            output_device_index=self.speaker_index
        )  #rate=24000

        start_time = time.time() 

        with openai.audio.speech.with_streaming_response.create( 
            model="tts-1", 
            voice=self.speaker, 
            response_format="pcm",  # similar to WAV, but without a header chunk at the start. 
            input=text, 
        ) as response: 
            print(f"Time to first byte: {int((time.time() - start_time) * 1000)}ms") 
            for chunk in response.iter_bytes(chunk_size=1024): 
                if self.stop_flag: # handles stopping
                    break
                if self.sample_rate == 24000:
                    player_stream.write(chunk)
                else:
                    # resampling if necessary
                    resample_factor = self.sample_rate / 24000

                    # converting bytes into numpy array
                    audio_data_np = np.frombuffer(chunk, dtype=np.int16)

                    # resampling the data
                    num_samples = int(len(audio_data_np) * resample_factor)
                    resampled_data = resample(audio_data_np, num_samples)

                    # converting the numpy array back to bytes
                    output_data = resampled_data.astype(np.int16).tobytes()

                    # writing resampled data to audio player stream
                    player_stream.write(output_data)

        print(f"Done in {int((time.time() - start_time) * 1000)}ms.")

        # stop stream after finishing
        player_stream.stop_stream()
        player_stream.close()
        p.terminate()

    '''uses festival to process the text and say it'''
    def text_to_speech_festival(self, text):

        # subprocess to begin tts
        self.tts_process = subprocess.Popen(f"echo {text} | festival --tts", shell=True, preexec_fn=os.setpgrp) 

        # Wait for the process to complete without blocking the event loop
        monitor_thread = threading.Thread(target=self.monitor_process)
        monitor_thread.start()
        monitor_thread.join()
    
    '''monitors the process, sleeping while TTS is not stopped'''
    def monitor_process(self):
        try:
            while not self.stop_flag:
                if self.tts_process and self.tts_process.poll() is None:
                    time.sleep(0.1)
                else:
                    break
        except Exception as e:
            self.get_logger().info(f"Exception while monitoring process: {e}")
    
def main(args=None):
    try: 
        rclpy.init(args=args)

        tts_node = ParcsTTS()

        # multithreaded because want to process to service callbacks simultaneously with action callbacks
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(tts_node)

        executor.spin()

        rclpy.shutdown()
    except (KeyboardInterrupt, Exception):
        pass

if __name__ == '__main__':
    main()
