import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from parcs_stt_tts_msgs.action import Listen
from parcs_stt_tts_msgs.action import Recalibrate
from pynput import keyboard

# uncomment if you'd like to use the keyboard module instead of pynput
# import keyboard
# import threading
# import time

class STTTester(Node):

    def __init__(self):
        super().__init__('stt_tester')
        self.get_logger().info('Started testing node. Please type "l" to start recording or "r" to recalibrate for background noise.')

        self._listen_action_client = ActionClient(self, Listen, 'listen')
        self._recalibrate_action_client = ActionClient(self, Recalibrate, 'recalibrate')
        self._goal_in_progress = False

        self._keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self._keyboard_listener.start()
    
    '''LISTEN'''
    def send_listen_goal(self):
        goal_msg = Listen.Goal()

        self._listen_action_client.wait_for_server()

        self._goal_in_progress = True
        self._send_goal_future = self._listen_action_client.send_goal_async(goal_msg, feedback_callback=self.listen_feedback_callback)
        self._send_goal_future.add_done_callback(self.listen_goal_response_callback)

    def listen_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Listen goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Listen goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.listen_result_callback)

    def listen_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Listen result received. Transcript: {result.transcript}')
        self._goal_in_progress = False
    
    def listen_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Listen feedback received: {feedback_msg}')

    '''RECALIBRATION'''
    def send_recalibrate_goal(self):
        goal_msg = Recalibrate.Goal()

        self._recalibrate_action_client.wait_for_server()

        self._goal_in_progress = True
        self._send_goal_future = self._recalibrate_action_client.send_goal_async(goal_msg, feedback_callback=self.recalibrate_feedback_callback)
        self._send_goal_future.add_done_callback(self.recalibrate_goal_response_callback)

    def recalibrate_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Recalibration goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Recalibration goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.recalibrate_result_callback)

    def recalibrate_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Recalibration result received.\ndBFS: {result.dbfs}\nsilence_threshold: {result.threshold}')
        self._goal_in_progress = False
    
    def recalibrate_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Recalibration feedback received: {feedback_msg}')
    
    '''Handles key presses to send goals'''
    def on_key_press(self, key):
        try:
            if key.char == 'l' and not self._goal_in_progress:
                self.send_listen_goal()
            elif key.char == 'r' and not self._goal_in_progress:
                self.send_recalibrate_goal()
        except AttributeError:
            pass

# '''Thread for continuous keyboard input'''
# def keyboard_listener(node):
#     while rclpy.ok():
#         if keyboard.is_pressed('l') and not node._goal_in_progress:
#             node.send_listen_goal()
        
#         elif keyboard.is_pressed('r') and not node._goal_in_progress:
#             node.send_recalibrate_goal()
        
#         time.sleep(0.1)  # Polling delay to avoid high CPU usage

def main(args=None):
    rclpy.init(args=args)

    node = STTTester()
    # keyboard_thread = threading.Thread(target=keyboard_listener, args=(node,))
    # keyboard_thread.start()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down.")
    
    finally:
        # keyboard_thread.join() # stops keyboard thread
        node._keyboard_listener.stop() # stops keyboard listener
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()