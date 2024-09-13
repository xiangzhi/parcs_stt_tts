import asyncio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from parcs_stt_tts_msgs.action import TTS, StopTTS
import openai
import os 
import threading

class ParcsTTSTest(Node):

    def __init__(self):
        super().__init__('parcs_tts_test')
        self._publisher = self.create_publisher(String, "/text_to_speech", 10)
        # parameters
        personality_param = 'you like humans most of the time and are a helpful robot'
        tts_interpreter_param = 'festival'  # 'openai'
        gen_response_param = 'false'  # must be 'true' or 'false

        self.declare_parameter("personality", personality_param)
        self.declare_parameter("interpreter", tts_interpreter_param)
        self.declare_parameter("gen_response", gen_response_param)

        self.personality_param = self.get_parameter("personality").get_parameter_value().string_value
        self.tts_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value
        self.gen_response_param = self.get_parameter("gen_response").get_parameter_value().string_value

        if self.tts_interpreter_param == 'openai':
            openai.api_key = os.getenv("OPENAI_API_KEY")  # get api key as environmental variable
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
        self.tts_process = None

        self.get_logger().info(f"TTS interpreter: {self.tts_interpreter_param}")

    async def tts_callback(self, goal_handle):
        self.get_logger().info("Executing TTS goal...")
        
        self._goal_handle = goal_handle
        goal = self._goal_handle.request
        tts_string = goal.tts

        self.get_logger().info(f"Received TTS goal: {tts_string}")

        if self.gen_response_param == 'true':
            response = await self.generate_response(goal.tts)
            tts_string = response

        tts_string = tts_string.replace("'", "\\'").replace('"', '\\"')

        result = TTS.Result()
        result.msg = await self.produce_speech(tts_string)
        goal_handle.succeed()
        return result
    
    def stop_tts_callback(self, goal_handle):
        self.get_logger().info("Executing stop TTS goal...")
        
        self._goal_handle = goal_handle

        if self.stop_flag:
            goal_handle.canceled() 
            result = StopTTS.Result()
            result.stopped = self.stop_flag
            return result
        
        self.stop_flag = True
        goal_handle.succeed()
        
        result = StopTTS.Result()
        result.stopped = self.stop_flag
        return result
    
    async def stop_goal_callback(self, goal_request):
        if self.stop_flag: 
            self.get_logger().info('Stopping already in progress.')
            return GoalResponse.REJECT
        else: 
            return GoalResponse.ACCEPT
    
    async def produce_speech(self, msg):
        if self.tts_interpreter_param == 'openai':
            await self.text_to_speech_openai(msg)
        elif self.tts_interpreter_param == 'festival':
            await self.text_to_speech_festival(msg)
        else:
            self.get_logger().error("Invalid TTS interpreter. Choose a valid one >:( (i.e. openai, festival)")

        done_msg = String()
        done_msg.data = msg
        self._publisher.publish(done_msg)

        self.stop_flag = False

        self.get_logger().info('Finished TTS.')
        return msg
    
    async def generate_response(self, query):
        if self.tts_interpreter_param == 'openai':
            response = await asyncio.to_thread(
                openai.chat.completions.create,
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.personality},
                    {"role": "user", "content": query},
                ]
            )
            response_data = response.choices[0].message.content
            self.get_logger().info("AI Response: ", response_data)
            return response_data
        else:
            self.get_logger().error("Could not generate a response; must specify an interpreter with that capability.")
            return None
    
    async def text_to_speech_openai(self, text):
        import pyaudio

        player_stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=24000, output=True) 
        start_time = time.time() 

        async with openai.audio.speech.with_streaming_response.create( 
            model="tts-1", 
            voice=self.speaker, 
            response_format="pcm",  # similar to WAV, but without a header chunk at the start. 
            input=text, 
        ) as response: 
            self.get_logger().info(f"Time to first byte: {int((time.time() - start_time) * 1000)}ms") 
            async for chunk in response.iter_bytes(chunk_size=1024): 
                if self.stop_flag: 
                    self.stop_flag = False
                    break
                player_stream.write(chunk) 

        self.get_logger().info(f"Done in {int((time.time() - start_time) * 1000)}ms.")

    async def text_to_speech_festival(self, text):
        # Create an event loop if it doesn't exist
        try:
            loop = asyncio.get_running_loop()  # Get the current running loop
        except RuntimeError:
            loop = asyncio.new_event_loop()  # Create a new loop if none is running
            asyncio.set_event_loop(loop)

        self.tts_process = await asyncio.create_subprocess_shell(
            f"echo {text} | festival --tts",
            shell=True
        )

        while self.tts_process.poll() is None:
            await asyncio.sleep(1)  # Non-blocking wait until process is done

    def stop_playback(self):
        if self.tts_process:
            self.get_logger().info('Changing stopped flag to true...')
            self.stop_flag = True
            self.tts_process.terminate()
            self.tts_process.wait()
            self.get_logger().info('Stopped TTS process.')
        self.stop_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = ParcsTTSTest()

    # Start asyncio event loop in a separate thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def run_event_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()

    event_loop_thread = threading.Thread(target=run_event_loop, args=(loop,), daemon=True)
    event_loop_thread.start()

    # Spin ROS 2 node in the main thread
    rclpy.spin(node)

    # Shutdown everything after ROS 2 node stops
    loop.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
