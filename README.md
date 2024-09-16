# PARCS STT and TTS Nodes
- Created by Percy Masetti Zannini and Emily Taylor

These ROS2 packages handle speech-to-text (STT) and text-to-speech (TTS) in ROS2 nodes. 

The STT node adjusts for background noise and can publish text to the `/speech_to_text` ROS2 topic. It has the actions Listen and Recalibrate. Listen will take in one round of speech, stopping when it has detected silence. Recalibrate will readjust for background noise when requested. 

The TTS node has the action TTS and the service Stop. When requested, the TTS action will take in the message sent with the goal and say it through your default speakers. At any point while TTS is playing, the Stop service can be requested to immediately terminate processes and stop speech mid-sentence. If desired, it can handle response generation with OpenAI from the text it was given; this is a parameter. 

---

As of the latest update, the following interpreters are supported:

parcs_stt
- OpenAI

parcs_tts 
- OpenAI 
- Festival

## Installation
```shell
# clone
$ cd ~/ros2_ws/src
$ git clone https://github.com/emilytaylor6/parcs_stt_tts.git

# dependencies
$ cd parcs_stt_tts
$ pip3 install -r requirements.txt

# building
$ cd ~/ros2_ws
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build
```

## Running 
To use customized parameters, such as another interpreter instead of OpenAI, use a launch file.

Speech to text demo launch:
```shell
$ ros2 launch parcs_stt stt_tester.launch.py
```

Text to speech demo launch:
```shell
$ ros2 launch parcs_tts parcs_tts.launch.py
```

Text parameters are lower case unless expressed otherwise. 

---

To run the nodes regularly, use the following:

Speech to text:
```shell
$ ros2 run parcs_stt parcs_stt 
```

Text to speech:
```shell
$ ros2 run parcs_tts parcs_tts
```

## Setting an API Key
Default parameters include using OpenAI, and therefore, require and environmental variable. To set one, use the following in the terminal in which you are running the code:
```shell
$ export OPENAI_API_KEY='api-key-here'
```
To avoid needing to do this every time, you can set a permanent environmental variable by using a text editor to access `~/.bashrc` and adding the line above to the end of the file. 