from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for the chatbot's input and output
    return LaunchDescription([
        #say that 5 times in a row
        Node(
            package='parcs_stt',
            namespace='parcs_stt',
            executable='parcs_stt',
            name='parcs_stt',
            output='screen',
            parameters=[{"threshold": -2}, {"interpreter": "openai"}] # can be “anthropic” or “openai”
        )
    ])