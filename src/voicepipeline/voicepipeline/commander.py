import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Commander(Node):
    def __init__(self):
        super().__init__('whisper_commander')

        self.command_subscriber = self.create_subscription(
            String,
            'whisper_transcript',
            self.transcript_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Command state variables
        self.twist_command = Twist()
        self.last_command = None
        self.speed = 0.5  # Initial speed
        self.turn = 1.0   # Initial turning speed
        self.max_speed = 2.0
        self.min_speed = 0.1

        # Publishing rate for continuous command (10 Hz)
        self.timer_period = 0.1  # 0.1 seconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_last_command)

        # Timeout for stopping the robot (10 seconds)
        self.timeout_duration = 10.0  # Timeout after 10 seconds
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

        # Flag to track if we should keep publishing
        self.is_moving = False

        # Predetermined dictionary for transcription commands
        self.straight = ['go straight', 'move forward', 'advance', 'proceed', 'forward', 'straight']
        self.backward = ['go back', 'move back', 'reverse', 'retreat', 'backward']
        self.left = ['go left', 'move left', 'turn left', 'left']
        self.right = ['go right', 'move right', 'turn right', 'right']
        self.stop = ['stop', 'halt', 'cease', 'pause', 'standby']
        self.speed_up = ['speed up', 'faster', 'accelerate', 'increase speed']
        self.slow_down = ['slow down', 'slower', 'decelerate', 'reduce speed']

    def move_forward(self):
        self.twist_command.linear.x = self.speed
        self.twist_command.angular.z = 0.0
        self.last_command = 'forward'
        self.get_logger().info("Moving forward")
        self.is_moving = True  # Set moving flag

    def move_backward(self):
        self.twist_command.linear.x = -self.speed
        self.twist_command.angular.z = 0.0
        self.last_command = 'backward'
        self.get_logger().info("Moving backward")
        self.is_moving = True  # Set moving flag

    def rotate_left(self):
        self.twist_command.angular.z = self.turn
        self.twist_command.linear.x = 0.0
        self.last_command = 'left'
        self.get_logger().info("Rotating left")
        self.is_moving = True  # Set moving flag

    def rotate_right(self):
        self.twist_command.angular.z = -self.turn
        self.twist_command.linear.x = 0.0
        self.last_command = 'right'
        self.get_logger().info("Rotating right")
        self.is_moving = True  # Set moving flag

    def stop_movement(self):
        self.twist_command = Twist()  # Reset the Twist command (stops the robot)
        self.last_command = 'stop'
        self.get_logger().info("Stopping")
        self.is_moving = False  # Stop moving flag

    def adjust_speed(self, delta):
        new_speed = self.speed + delta
        if self.min_speed <= new_speed <= self.max_speed:
            self.speed = new_speed
            self.get_logger().info(f"Speed adjusted to: {self.speed}")
        else:
            self.get_logger().info(f"Speed adjustment out of bounds: {new_speed}")

    def publish_last_command(self):
        """Continuously publish the last command at a fixed rate (simulates holding down a key)."""
        if self.is_moving and self.last_command != 'stop':
            self.command_publisher.publish(self.twist_command)

    def stop_on_timeout(self):
        """Stops the robot if no new command is received within the timeout duration."""
        if self.is_moving:
            self.get_logger().info("Timeout reached, stopping the robot.")
            self.stop_movement()
            self.publish_last_command()  # Ensure a stop command is published

    def transcript_callback(self, msg):
        transcription = msg.data.lower()

        if not transcription.strip():  # Check if transcription is empty or only contains whitespace
            self.get_logger().info("Ignoring empty transcription")
            return  # Return immediately if transcription is empty
        
        # Remove punctuation and split transcription into individual commands or phrases
        command_phrases = [
            phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?') for phrase in transcription.split(',')
        ]
        
        for phrase in command_phrases:
            if phrase in self.straight:
                self.move_forward()
                self.reset_timeout_timer()
                break
            elif phrase in self.backward:
                self.move_backward()
                self.reset_timeout_timer()
                break
            elif phrase in self.left:
                self.rotate_left()
                self.reset_timeout_timer()
                break
            elif phrase in self.right:
                self.rotate_right()
                self.reset_timeout_timer()
                break
            elif phrase in self.stop:
                self.stop_movement()
                self.reset_timeout_timer()
                break
            elif phrase in self.speed_up:
                self.adjust_speed(0.1)
                break
            elif phrase in self.slow_down:
                self.adjust_speed(-0.1)
                break
            else:
                self.get_logger().info(f"Unknown command: '{phrase}'")
                self.stop_movement()

    def reset_timeout_timer(self):
        """Resets the timeout timer when a new command is received."""
        self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
