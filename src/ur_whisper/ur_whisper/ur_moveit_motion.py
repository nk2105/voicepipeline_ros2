#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from geometry_msgs.msg import Pose, PoseStamped, Twist
import tf2_ros
import transforms3d
from tf2_geometry_msgs import do_transform_pose
from moveit.core.robot_state import RobotState
from std_msgs.msg import String

import math

class URMoveItMotion(Node):
    def __init__(self):
        super().__init__('ur_moveit_motion')
        
        self.moveit = MoveItPy("moveit_py")
        self.arm = self.moveit.get_planning_component('ur_manipulator')

        self.command_subscriber = self.create_subscription(
            String,
            '/whisper_transcript',
            self.transcript_callback,
            10
        )
        self.set_last_pose = None
        self.is_running = False

        self.straight = ['go straight', 'move forward', 'advance', 'proceed', 'forward', 'straight']
        self.backward = ['go back', 'move back', 'reverse', 'retreat', 'backward']
        self.left = ['go left', 'move left', 'turn left', 'left']
        self.right = ['go right', 'move right', 'turn right', 'right']
        self.stop = ['stop', 'halt', 'cease', 'pause', 'standby']
        self.position = ['position', 'pose', 'location', 'place']
        self.grip = ['grip', 'grab', 'hold', 'clutch', 'grasp']
        self.release = ['release', 'let go', 'drop', 'unclutch']
        self.tcp_rotation_left = ['tcp rotate left', 'tcp turn left', 'tcp spin left', 'tcp twist left']
        self.tcp_rotation_right = ['tcp rotate right', 'tcp turn right', 'tcp spin right', 'tcp twist right']

    def execute_cartesian_path(self, x, y, z, roll, pitch, yaw):
        current_pose = self.moveit.get_current_pose()
        target_pose = PoseStamped()
        if current_pose is None:
            current_pose = self.moveit.get_current_pose()

    
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = current_pose.position.x + x
        target_pose.pose.position.y = current_pose.position.y + y
        target_pose.pose.position.z = current_pose.position.z + z


        quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]


        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose)

        waypoints = [current_pose, target_pose]
        (plan, fraction) = self.moveit.compute_cartesian_path(
            waypoints,
            0.01, 
            0.0
        )

        if fraction > 0.9:
            planning_result = self.arm.plan(plan, wait=True)
            trajectory = planning_result.trajectory
            self.moveit.execute(trajectory)
            self.get_logger().info("Cartesian path executed successfully")
        else:
            self.get_logger().warn("Cartesian path planning failed")


    def go_straight(self):
        self.execute_cartesian_path(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)

    def go_back(self):
        self.execute_cartesian_path(-0.1, 0.0, 0.0, 0.0, 0.0, 0.0)

    def go_left(self): 
        self.execute_cartesian_path(0.0, 0.1, 0.0, 0.0, 0.0, 0.0)

    def go_right(self):
        self.execute_cartesian_path(0.0, -0.1, 0.0, 0.0, 0.0, 0.0)

    def transcript_callback(self, msg):

        transcription = msg.data.lower()
        if not transcription.strip():
            self.get_logger().info("Ignoring empty transcription")
            return

        try:
            command_phrases = [
                phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?') for phrase in transcription.split(',')
            ]
            valid_command_detected = False
            for phrase in command_phrases:

                if phrase in self.straight:
                    self.go_straight()
                    valid_command_detected = True
                elif phrase in self.backward:
                    self.go_back()
                    valid_command_detected = True
                elif phrase in self.left:
                    self.go_left()
                    valid_command_detected = True
                elif phrase in self.right:
                    self.go_right()
                    valid_command_detected = True   
                elif phrase in self.stop:
                    self.arm.stop()

            if not valid_command_detected:
                self.arm.stop()
                self.get_logger().warn("Invalid command: {}. Stopping".format(transcription))
            else:
                self.get_logger().info("Executing command: {}".format(transcription)) 

        except Exception as e:
            self.get_logger().error('An error occurred: {}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    node = URMoveItMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt and Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()