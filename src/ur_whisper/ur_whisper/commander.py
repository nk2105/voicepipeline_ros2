import rclpy
from rclpy.node import Node
from ur_interfaces.msg import URWhisperCommands, URWhisperCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import math

class Commander(Node):
    def __init__(self):
        super().__init__('whisper_commander')
        self.command_subscriber = self.create_subscription(
            String,
            'whisper_transcript',
            self.transcript_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/Spot/odometry',
            self.odom_callback,
            10
        )
        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.previous_position = None
        self.total_distance = 0.0
        self.max_distance = 5.0
        self.twist_command = Twist()

        # Predetermined dictionary for transcription commands
        self.straight = ['go straight', 'move forward', 'advance', 'proceed', 'forward', 'straight']
        self.backward = ['go back', 'move back', 'reverse', 'retreat', 'backward']
        self.left = ['go left', 'move left', 'turn left', 'left']
        self.right = ['go right', 'move right', 'turn right', 'right']
        self.pan_to_left = ['pan left', 'leftward', 'leftwards']
        self.pan_to_right = ['pan right', 'rightward', 'rightwards']
        
        self.stop = ['stop', 'halt', 'cease', 'pause', 'standby']
        self.position = ['position', 'pose', 'location', 'place']

        self.grip = ['grip', 'grab', 'hold', 'clutch', 'grasp']
        self.release = ['release', 'let go', 'drop', 'unclutch']

        self.tcp_rotation_left = ['tcp rotate left', 'tcp turn left', 'tcp spin left', 'tcp twist left']
        self.tcp_rotation_right = ['tcp rotate right', 'tcp turn right', 'tcp spin right', 'tcp twist right']

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        if self.previous_position is not None:
            self.distance = self.calculate_distance(self.previous_position, current_position)
            self.total_distance += self.distance
            if self.total_distance >= self.max_distance:
                self.stop_movement()
                self.total_distance = 0.0
                self.get_logger().info(f"Maximum distance reached: {self.max_distance}")
    
            if self.distance > 0.01:
                self.get_logger().info(f"Total distance traveled: {self.total_distance}")
        
        self.previous_position = current_position

    def calculate_distance(self, pos1: Point, pos2: Point) -> float:
        return math.sqrt(
            (pos2.x - pos1.x) ** 2 + 
            (pos2.y - pos1.y) ** 2 + 
            (pos2.z - pos1.z) ** 2
        )
    def move_forward(self):
        self.twist_command.linear.x = 0.4
        self.get_logger().info("Moving forward")

    def move_backward(self):
        self.twist_command.linear.x = -0.4
        self.get_logger().info("Moving backward")

    def rotate_left(self):
        self.twist_command.angular.z = 0.1
        self.twist_command.linear.x = 0.0
        self.get_logger().info("Rotating left")

    def rotate_right(self):
        self.twist_command.angular.z = -0.1
        self.twist_command.linear.x = 0.0
        self.get_logger().info("Rotating right")

    def pan_left(self):
        self.twist_command.linear.y = 0.1
        self.get_logger().info("Panning left")

    def pan_right(self):
        self.twist_command.linear.y = -0.1
        self.get_logger().info("Panning right")

    def stop_movement(self):
        self.twist_command.linear.x = 0.0
        self.twist_command.angular.z = 0.0
        self.twist_command.linear.y = 0.0
        self.distance = 0.0
        self.get_logger().info("Stopping")
        self.command_publisher.publish(self.twist_command)


    def transcript_callback(self, msg):
        transcription = msg.data.lower()  # Convert to lowercase for consistency
        if not transcription.strip():  # Check if transcription is empty or only contains whitespace
            self.get_logger().info("Ignoring empty transcription")
            return  # Return immediately if transcription is empty
        
        try:
            # Remove punctuation and split transcription into individual commands or phrases
            command_phrases = [
                phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?') for phrase in transcription.split(',')
            ]

            # Flag to check if any valid command is detected
            valid_command_detected = False

        
            for command_phrase in command_phrases:
                # Set linear and angular velocities based on recognized commands
                if command_phrase in self.straight:
                    self.move_forward()
                    valid_command_detected = True
                elif command_phrase in self.backward:
                    self.move_backward()
                    valid_command_detected = True
                elif command_phrase in self.left:
                    self.rotate_left()
                    valid_command_detected = True
                elif command_phrase in self.right:
                    self.rotate_right()
                    valid_command_detected = True               
                elif command_phrase in self.stop:
                    self.stop_movement()
                    valid_command_detected = True
                elif command_phrase in self.pan_to_left:
                    self.pan_left()
                    valid_command_detected = True
                elif command_phrase in self.pan_to_right:
                    self.pan_right()
                    valid_command_detected = True

            if valid_command_detected:
                # Publish the command only if a valid command is detected
                self.command_publisher.publish(self.twist_command)
                self.get_logger().info("Published Twist command successfully")

            else:
                # Log that no valid command was recognized
                self.get_logger().info(f"No valid command recognized from transcription: '{transcription}'")
                self.stop_movement()

        except Exception as e:
            self.get_logger().error(f"Error processing transcription: {e}")



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
