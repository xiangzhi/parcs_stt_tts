import asyncio
import signal
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import sounddevice as sd
from pydub import AudioSegment
import numpy as np 
import tempfile
import openai
import os 
import pyaudio
import time 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from parcs_stt_tts_msgs.action import TTS
from parcs_stt_tts_msgs.srv import Stop
from concurrent.futures import ThreadPoolExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

class ParcsTTS(Node):

    def __init__(self):
        super().__init__('parcs_tts')

        # parameters
        # speaker parameters to be made later
        personality_param = 'you like humans most of the time and are a helpful robot'
        tts_interpreter_param = 'festival' # 'festival' or 'openai', case sensitive
        gen_response_param = 'false' # must be 'true' or 'false', case sensitive

        self.declare_parameter("personality", personality_param)
        self.declare_parameter("interpreter", tts_interpreter_param)
        self.declare_parameter("gen_response", gen_response_param)

        self.personality_param = self.get_parameter("personality").get_parameter_value().string_value
        self.tts_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value
        self.gen_response_param = self.get_parameter("gen_response").get_parameter_value().string_value

        if self.tts_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable
            self.speaker = "alloy"
        
        # action and service server initializing
        self.tts_callback_group = MutuallyExclusiveCallbackGroup() # handles one callback at a time
        self.stop_callback_group = MutuallyExclusiveCallbackGroup()

        self._tts_action_server = ActionServer(
            self,
            TTS,
            'tts',
            self.tts_callback,
            callback_group=self.tts_callback_group
        )

        self._stop_serv = self.create_service(Stop, 
                                              'stop', 
                                              self.abort_goal_req,
                                              callback_group=self.stop_callback_group)
        
        # variables
        self.stop_flag = False
        self.tts_process = None # for festival or other interpreters that would use subprocess.Popen
        self.tts_goal = None
        self.prod_speech_result = None

        self.get_logger().info(f"TTS interpreter: {self.tts_interpreter_param}")
        self.get_logger().info(f"TTS node ready.\n--------------------------------------------")

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
    def abort_goal_req(self, request, response):
        self.get_logger().info("Aborting TTS goal...")
        self.abort_resp = Stop.Response()

        # handles killing/aborting asynchronously 
        stop_thread = threading.Thread(target=self.handle_abort)
        stop_thread.start()
        stop_thread.join()

        return self.abort_resp 
    
    '''aborts/kills all text to speech processses'''
    def handle_abort(self):
        if not self.stop_flag: 
            if self.tts_goal:
                if (self.tts_goal.status == GoalStatus.STATUS_EXECUTING):
                    # kills tts if handled in a subprocess
                    if self.tts_process is not None: 
                        os.killpg(os.getpgid(self.tts_process.pid), signal.SIGTERM)
                        self.get_logger().info("Killed process.")
                        self.tts_process = None

                    self.abort_resp.success = True 
                    self.stop_flag = True

                    self.get_logger().info("TTS stopped by service.")
                    if self.tts_goal.status not in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]:
                        self.tts_goal.abort()
                else:
                    self.abort_resp.success = False
                    self.get_logger().info("TTS not executing.")
            else:
                self.abort_resp.success = False
                self.get_logger().info("No TTS to stop.")
        else:
            self.abort_resp.success = False
            self.get_logger().info("Already attempted to stop. Ignoring request.")
    

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
    
    '''generates a response'''
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

        player_stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=24000, output=True) 

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
                player_stream.write(chunk) 

        print(f"Done in {int((time.time() - start_time) * 1000)}ms.")

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
            if self.tts_process:
                while not self.stop_flag and self.tts_process.poll() is None:
                    time.sleep(0.1)
            else:
                while not self.stop_flag:
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().info(f"Exception while waiting for process: {e}")
    
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
