import rclpy
from rclpy.node import Node
from ur_interfaces.msg import GaitInput
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import math

class Commander(Node):
    def __init__(self,
            settings=None, 
            x = 0.0,
            y = 0.0,
            z = 0.1,
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
            StepLength = 0.024,
            LateralFraction = 0.0,
            YawRate = 0.0,
            StepVelocity = 0.01,
            ClearanceHeight = 0.024,
            PenetrationDepth = 0.003,
            SwingPeriod = 0.2,
            YawControl = 0.0,
            YawControlOn = 0.0,
            status = 0,):
        
        super().__init__('whisper_commander')
        spot_name = 'Spot'
        self.command_subscriber = self.create_subscription(
            String,
            'whisper_transcript',
            self.transcript_callback,
            10
        )

        #self.odom_subscriber = self.create_subscription(
        #    Odometry,
        #    '/Spot/odometry',
        #    self.odom_callback,
        #    10
        #)
        self.command_publisher = self.create_publisher(
            GaitInput,
            '/' + spot_name + '/inverse_gait_input',
            10
        )
        
        self.gait_msg = GaitInput()
        self.gait_msg.x = x
        self.gait_msg.y = y
        self.gait_msg.z = z
        self.gait_msg.roll = roll
        self.gait_msg.pitch = pitch
        self.gait_msg.yaw = yaw
        self.gait_msg.step_length = StepLength
        self.gait_msg.lateral_fraction = LateralFraction
        self.gait_msg.yaw_rate = YawRate
        self.gait_msg.step_velocity = StepVelocity
        self.gait_msg.clearance_height = ClearanceHeight
        self.gait_msg.penetration_depth = PenetrationDepth
        self.gait_msg.swing_period = SwingPeriod
        self.gait_msg.yaw_control = YawControl
        self.gait_msg.yaw_control_on = YawControlOn

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

    def move_forward(self):
        self.gait_msg.step_length = 0.05
        self.gait_msg.step_velocity = 0.05
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Moving forward")

    def move_backward(self):
        self.gait_msg.step_length = -0.05
        self.gait_msg.step_velocity = 0.05
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Moving backward")

    def move_left(self):
        self.gait_msg.lateral_fraction = -0.5
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Moving left")

    def move_right(self):
        self.gait_msg.lateral_fraction = 0.5
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Moving right")

    def rotate_left(self):
        self.gait_msg.yaw_rate = 0.1
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Rotating left")

    def rotate_right(self):
        self.gait_msg.yaw_rate = -0.1
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Rotating right")

    def stop_movement(self):
        self.gait_msg.step_length = 0.0
        self.gait_msg.lateral_fraction = 0.0
        self.gait_msg.yaw_rate = 0.0
        self.gait_msg.step_velocity = 0.0
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Stopping movement")

    def speed_up(self):
        self.gait_msg.step_velocity += 0.01
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Speeding up")

    def slow_down(self):
        self.gait_msg.step_velocity -= 0.01
        self.command_publisher.publish(self.gait_msg)
        self.get_logger().info("Slowing down")


    def transcript_callback(self, msg):
        transcription = msg.data.lower()

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
                    self.move_left()
                    valid_command_detected = True
                elif command_phrase in self.pan_to_right:
                    self.move_right()
                    valid_command_detected = True

            if not valid_command_detected:
                self.get_logger().info("No valid command detected")
                self.stop_movement()

            else:
                self.get_logger().info("Published command successfully")

        except ValueError as e:
            self.get_logger().error(f"Error processing transcription: {e}")
        except TypeError as e:
            self.get_logger().error(f"Error processing transcription: {e}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")




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
