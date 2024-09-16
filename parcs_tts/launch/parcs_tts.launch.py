from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for text to speech and its tester
    '''Note: Must launch the TTS node alongside another node that uses it for it to function.'''

    return LaunchDescription([
        Node(
            package='parcs_tts',
            namespace='parcs_tts',
            executable='parcs_tts',
            name='parcs_tts',
            output='screen',
            parameters=[{"personality": "you are a helpful robot"}, # the personality if generating responses
                        {"interpreter": "openai"}, # 'festival' or 'openai', case sensitive 
                        {"gen_response": "false"} # whether you want to generate responses or not: 'true' or 'false', case sensitive
                        ] 
        ),
        Node(
            package='parcs_tts',
            namespace='parcs_tts',
            executable='tts_tester',
            name='tts_tester',
            output='screen',
        )
    ])