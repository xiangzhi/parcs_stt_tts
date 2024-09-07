# PARCS STT and TTS Nodes
- Created by Percy Masetti Zannini and Emily Taylor

These ROS2 packages handle speech-to-text (STT) and text-to-speech (TTS) in ROS2 nodes. 

The STT node adjusts for background noise and publishes the text to the `/parcs_stt/chatbot` ROS2 topic. It is subscribed to the `/parcs_tts/chatbot_ack` ROS2 topic for acknowledgement, which runs the node again, taking in another round of input to be produced. 

The TTS node subcribes to the `/parcs_stt/chatbot` ROS2 topic and produces audio to repeat the text it was given. When it is finished repeating the text, it publishes to the `/parcs_tts/chatbot_ack` ROS2 topic to acknowledge that it has finished playing the audio. 

---

As of the latest update, the following interpreters are supported.

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
To use customized parameters, such as another interpreter instead of OpenAI, use a launch file:
```shell
$ ros2 launch parcs_tts parcs_tts.launch.py
$ ros2 launch parcs_stt parcs_stt.launch.py
```
Text parameters are lower case unless expressed otherwise. 
---

To run the nodes regularly, use the following:
```shell
$ ros2 run parcs_tts parcs_tts
$ ros2 run parcs_stt parcs_stt 
```

## Setting an API Key
Default parameters include using OpenAI, and therefore, require and environmental variable. To set one, use the following in the terminal in which you are running the code:
```shell
$ export OPENAI_API_KEY='api-key-here'
```
To avoid needing to do this every time, you can set a permanent environmental variable by using a text editor to access `~/.bashrc` and adding the line above to the end of the file. 