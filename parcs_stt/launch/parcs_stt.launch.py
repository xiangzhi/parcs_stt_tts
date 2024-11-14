from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for the chatbot's input and output
    return LaunchDescription([
        Node(
            package='parcs_stt',
            executable='parcs_stt',
            name='parcs_stt',
            output='screen',
            parameters=[{"threshold": 5.0}, {"interpreter": "openai"}] # can be “anthropic” or “openai”
        )
    ])