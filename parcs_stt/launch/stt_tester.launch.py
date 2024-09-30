from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for the chatbot's input and output
    return LaunchDescription([
        Node(
            package='parcs_stt',
            namespace='stt',
            executable='parcs_stt',
            name='parcs_stt',
            output='screen',
            parameters=[{"relative_threshold": 1.0}, # relative threshold depending on calibration value, 0.5 works well for quiet areas
                        {"set_threshold": 0.0}, # set the threshold directly without calibration; 0.0 means it will use calibration
                        {"interpreter": 'openai'}, # can be “anthropic” or “openai”
                        {"pause_duration": 2.0}, # the amount of time of a pause in seconds to begin processing audio; the audio chunk length
                        {"microphone": 'default'} # the name of the microphone that you want to be used for detection
                        ] 
        ),
        Node(
            package='parcs_stt',
            namespace='stt',
            executable='stt_tester',
            name='stt_tester',
            output='screen',
        ),
    ])