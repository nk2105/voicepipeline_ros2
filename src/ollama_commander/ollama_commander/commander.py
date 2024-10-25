#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import ollama
import json
import time

model = 'twist'
role = 'user'

class Commander(Node):
    def __init__(self):
        super().__init__('whisper_commander')

        # Create subscribers and publishers
        self.command_subscriber = self.create_subscription(String, 'whisper_transcript', self.transcript_callback, 10)
        self.command_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Command state variable
        self.twist_command = Twist()
        self.last_command = None

        # Flag to track if we should keep publishing
        self.is_moving = False

        # Publishing rate for continuous command (10 Hz)
        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.publish_last_command)

        # Timeout settings
        self.timeout = 5.0  # 5 seconds
        self.timeout_timer = self.create_timer(self.timeout, self.stop_on_timeout)

        print("Commander node is running.")

    def publish_last_command(self):
        """Continuously publish the last command at a fixed rate (simulates holding down a key)."""
        if self.is_moving and self.last_command != 'stop':
            self.command_publisher.publish(self.twist_command)

    def transcript_callback(self, msg):
        transcription = msg.data.lower().strip()
        if not transcription:
            print("Ignoring empty transcription")
            return 

        # Split and clean up the command phrases
        command_phrases = [phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?') for phrase in transcription.split(',')]
        if not command_phrases:
            print("No valid commands found")
            return
        
        # Send the first command to Ollama
        response = ollama.chat(model=model, messages=[{'role': role, 'content': command_phrases[0]}])
        response_content = response.get('message', {}).get('content', '')

        if not response_content:
            print("No valid response from Ollama.")
            return

        # Extract linear and angular velocities from the response
        if self.update_twist_command(response_content):
            # Publish the updated command if successful
            self.command_publisher.publish(self.twist_command)
            self.is_moving = True
            self.last_command = command_phrases[0]
            self.reset_timeout()  # Reset timeout whenever a new command is received
            print(response_content)

    def stop_movement(self):
        """Stop all robot movement."""
        self.twist_command = Twist()
        self.last_command = 'stop'
        self.is_moving = False
        self.command_publisher.publish(self.twist_command)

    def stop_on_timeout(self):
        """Stops the robot if no new command is received within the timeout duration."""
        if self.is_moving:
            print("Timeout reached, stopping the robot.")
            self.stop_movement()

    def update_twist_command(self, response):
        """Extract linear and angular velocities using JSON parsing if possible."""
        try:
            # Assume the response is a JSON string
            parsed_response = json.loads(response)

            # Safely extract linear and angular values using `.get()`
            self.twist_command.linear.x = float(parsed_response.get('message', {}).get('linear', {}).get('x', 0.0))
            self.twist_command.linear.y = float(parsed_response.get('message', {}).get('linear', {}).get('y', 0.0))
            self.twist_command.angular.z = float(parsed_response.get('message', {}).get('angular', {}).get('z', 0.0))

            # Log the extracted values for debugging
            print(f"Extracted velocities: linear.x={self.twist_command.linear.x}, "
                  f"linear.y={self.twist_command.linear.y}, angular.z={self.twist_command.angular.z}")
            return True
        except json.JSONDecodeError as e:
            print(f"JSON parsing error: {str(e)}")
            self.stop_movement()
            return False

    def reset_timeout(self):
        """Resets the timeout timer whenever a new command is received."""
        self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(self.timeout, self.stop_on_timeout)

    def destroy_node(self):
        # Log message before shutting down
        print("Shutting down commander node.")
        time.sleep(0.1)  # Add a small delay to ensure the log message is published
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        commander.destroy_node()  # Call destroy_node first

        if rclpy.ok():
            rclpy.shutdown()
            print("\n\033[93mQuitting..\033[0m")

if __name__ == '__main__':
    main()
