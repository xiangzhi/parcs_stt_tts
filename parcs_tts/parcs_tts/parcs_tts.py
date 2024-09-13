import asyncio
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
from parcs_stt_tts_msgs.action import StopTTS

class ParcsTTS(Node):

    def __init__(self):
        super().__init__('parcs_tts')
        self._publisher = self.create_publisher(String, "/text_to_speech", 10)
        # self._subscription = self.create_subscription(String, "/parcs_stt/chatbot", self.produce_speech, 10)
        # self._subscription

        # parameters
        # speaker to be made later
        personality_param = 'you like humans most of the time and are a helpful robot'
        tts_interpreter_param = 'festival' # 'festival' 'openai'
        gen_response_param = 'false' # must be 'true' or 'false

        self.declare_parameter("personality", personality_param)
        self.declare_parameter("interpreter", tts_interpreter_param)
        self.declare_parameter("gen_response", gen_response_param)

        self.personality_param = self.get_parameter("personality").get_parameter_value().string_value
        
        self.tts_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value

        self.gen_response_param = self.get_parameter("gen_response").get_parameter_value().string_value

        if self.tts_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable
            self.speaker = "alloy"
        
        self._tts_action_server = ActionServer(
            self,
            TTS,
            'tts',
            self.tts_callback
        )

        self._stop_tts_action_server = ActionServer(
            self,
            StopTTS,
            'stop_tts',
            self.stop_tts_callback,
            goal_callback=self.stop_goal_callback
        )

        self.stop_flag = False
        self.tts_process = None # for festival or other interpreters that would use subprocess.Popen
        self.stop_thread = None
        self.done_with_speech = False

        self.get_logger().info(f"TTS interpreter: {self.tts_interpreter_param}")

    '''tts action server callback'''
    async def tts_callback(self, goal_handle):
        self.get_logger().info("Executing TTS goal...")
        
        self._goal_handle = goal_handle
        goal = self._goal_handle.request
        tts_string = goal.tts

        self.get_logger().info(f"Received TTS goal: {tts_string}")

        if self.gen_response_param == 'true':
            response = await asyncio.to_thread(self.generate_response, goal.tts) # self.generate_response(goal.tts)
            #to be fixed 
            tts_string = response

        tts_string.replace("'", "\\'")
        tts_string.replace('"', '\\"')

        result = TTS.Result()
        # result.msg = self.produce_speech(tts_string)

        # Create or get an event loop and run the asynchronous code
        self.loop = asyncio.get_event_loop()

        if self.loop.is_running():
            # If the event loop is already running, use asyncio.to_thread
            result.msg = await asyncio.to_thread(self.produce_speech, tts_string)
        else:
            # If there's no running event loop, create one and run produce_speech
            # asyncio.new_event_loop()
            # result.msg = await asyncio.to_thread(self.produce_speech, tts_string)
            result.msg = await asyncio.run(self.produce_speech(tts_string))
        
        self.get_logger().info('Finished TTS.')
        goal_handle.succeed()
        return result
    
    '''stop tts action server callback'''
    def stop_tts_callback(self, goal_handle):
        self.get_logger().info("Executing stop TTS goal...")
        
        self._goal_handle = goal_handle

        if self.stop_thread is None:
            self.get_logger().info("Starting stop thread...")
            self.stop_thread = threading.Thread(target=self.stop_playback)
            self.stop_thread.start()
            # self.stop_playback()
            goal_handle.succeed()
        else:
            goal_handle.canceled() 

        result = StopTTS.Result()
        result.stopped = self.stop_flag
        return result
    
    '''ensures that only one stop goal is sent at once'''
    def stop_goal_callback(self, goal_request):
        if self.stop_flag: 
            self.get_logger().info('Stopping already in progress.')
            return GoalResponse.REJECT
        else: 
            return GoalResponse.ACCEPT
    
    '''produces speech given the message and interpreter, plays it from speakers, and publishes the produced message as a String to the topic'''
    async def produce_speech(self, msg):
        if self.tts_interpreter_param == 'openai':
            await self.text_to_speech_openai(msg)
        elif self.tts_interpreter_param == 'festival':
            await self.text_to_speech_festival(msg)
            # while self.tts_process.poll() is None:
            #     time.sleep(1)
            #     # asyncio.sleep(1)
        else:
            self.get_logger().error("Invalid TTS interpreter. Choose a valid one >:( (i.e. openai, festival)")

        done_msg = String()
        done_msg.data = msg

        self._publisher.publish(done_msg)

        # reset the stop flag
        self.stop_flag = False

        return msg
    
    '''generates a response'''
    async def generate_response(self, query):
        
        if self.tts_interpreter_param == 'openai':
            response = await openai.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.personality},
                    {"role": "user", "content": query},
                ],
            )
            response_data = response.choices[0].message.content
            print("AI Response: ", response_data)
        else:
            self.logger("Could not generate a response; must specify an interpreter with that capability.")
            return

        return response_data
    
    '''uses openai to process the text and say it'''
    async def text_to_speech_openai(self, text):

        player_stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=24000, output=True) 

        start_time = time.time() 

        with await openai.audio.speech.with_streaming_response.create( 
            model="tts-1", 
            voice=self.speaker, 
            response_format="pcm",  # similar to WAV, but without a header chunk at the start. 
            input=text, 
        ) as response: 
            print(f"Time to first byte: {int((time.time() - start_time) * 1000)}ms") 
            for chunk in response.iter_bytes(chunk_size=1024): 
                if self.stop_flag: # handles stopping
                    self.stop_flag = False
                    self.done_with_speech = True
                    break
                player_stream.write(chunk) 

        print(f"Done in {int((time.time() - start_time) * 1000)}ms.")
        self.done_with_speech = True

    '''uses festival to process the text and say it'''
    async def text_to_speech_festival(self, text):
        # os.system('echo %s | festival --tts' % text)

        # self.tts_process = subprocess.Popen(f"echo {text} | festival --tts", shell=True)
        self.tts_process = await asyncio.create_subprocess_shell(f"echo {text} | festival --tts")

        # Wait for the process to complete without blocking the event loop
        await self.tts_process.communicate()
    
    '''monitors the tts process and kills it if manually stopped'''
    def monitor_tts(self):
        while self.tts_process.poll() is None:
            # if self.stop_flag: # handles stopping if necessary
            self.get_logger().info('Playback manually stopped. Stopping...')
            self.tts_process.kill() # or could use .terminate() and .wait()
            self.tts_process.wait()
            self.get_logger().info('Killed TTS process.')
            self.tts_process = None
            self.stop_flag = False
            return
            # time.sleep(0.1)

    '''changes the flag to stop the playback'''
    def stop_playback(self):
        if not self.stop_flag:
            self.get_logger().info('Changing stopped flag to true...')
            self.stop_flag = True
        
        while self.stop_flag:
            if self.tts_process is not None:
                self.monitor_tts()
        
        if not self.stop_flag and self.stop_thread is not None:
            self.stop_thread.join()
            self.stop_thread = None
            self.get_logger().info("Ended stop thread.")

def main(args=None):
    rclpy.init(args=args)

    node = ParcsTTS()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
