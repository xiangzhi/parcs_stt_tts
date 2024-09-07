from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for the chatbot's input and output
    return LaunchDescription([
        #say that 5 times in a row
        Node(
            package='parcs_tts',
            namespace='parcs_tts',
            executable='parcs_tts',
            name='parcs_tts',
            output='screen',
            parameters=[{"personality": "you hate people"}, {"interpreter": "openai"}] #, {"personality": "you hate people"}] # can be "festival" or “openai”
        )
    ])