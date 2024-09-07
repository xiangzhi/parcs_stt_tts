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

class ParcsTTS(Node):

    def __init__(self):
        super().__init__('parcs_tts')
        self._publisher = self.create_publisher(String, "/parcs_tts/chatbot_ack", 10)
        self._subscription = self.create_subscription(String, "/parcs_stt/chatbot", self.textCallback, 10)
        self._subscription

        # parameters
        # speaker to be made later
        personality_param = 'you like humans most of the time and are a helpful robot'
        tts_interpreter_param = 'festival' # 'openai'

        self.declare_parameter("personality", personality_param)
        self.declare_parameter("interpreter", tts_interpreter_param)

        self.personality_param = self.get_parameter("personality").get_parameter_value().string_value
        
        self.tts_interpreter_param = self.get_parameter("interpreter").get_parameter_value().string_value
        

        if self.tts_interpreter_param == 'openai':
            openai.api_key= os.getenv("OPENAI_API_KEY") #get api key as environmental variable
            self.speaker = "alloy"

    def textCallback(self, msg):
        # response = self.generate_response(msg.data)

        if self.tts_interpreter_param == 'openai':
            self.text_to_speech_openai(msg.data)
        elif self.tts_interpreter_param == 'festival':
            self.text_to_speech_festival(msg.data)
        else:
            self.get_logger().error("Invalid TTS interpreter. Choose a valid one >:( (i.e. openai, festival)")

        done_msg = String()
        done_msg.data = 'Finished TTS.'

        self._publisher.publish(done_msg)
    
    '''uncomment if you want to generate responses with openai'''
    # def generate_response(self, query):
        
    #     response = openai.chat.completions.create(
    #         model="gpt-3.5-turbo",
    #         messages=[
    #             {"role": "system", "content": self.personality},
    #             {"role": "user", "content": query},
    #         ],
    #     )
    #     response_data = response.choices[0].message.content
    #     print("AI Response: ", response_data)

    #     return response_data.replace("'", "")

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
                player_stream.write(chunk) 

        print(f"Done in {int((time.time() - start_time) * 1000)}ms.")

    def text_to_speech_festival(self, text):

       os.system('echo %s | festival --tts' % text)

    

def main(args=None):
    rclpy.init(args=args)

    node = ParcsTTS()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
