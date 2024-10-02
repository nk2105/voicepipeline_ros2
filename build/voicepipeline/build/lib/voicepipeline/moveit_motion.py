#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from geometry_msgs.msg import Pose, PoseStamped, Twist
import tf2_ros
import transforms3d
from moveit.core.robot_state import RobotState
from std_msgs.msg import String

import math

class URMoveItMotion(Node):
    def __init__(self):
        super().__init__('moveit_motion')
        self.logger = rclpy.logging.get_logger("moveit_py.pose_goal")

        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component('manipulator')
        self.gripper = self.moveit.get_planning_component('gripper')
        self.logger.info("MoveItPy initialized")

        self.command_subscriber = self.create_subscription(
            String,
            '/whisper_transcript',
            self.transcript_callback,
            10
        )
        

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
    
    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        ):
        """A helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
        else:
                plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                robot.execute(robot_trajectory, controllers=[])
        else:
                logger.error("Planning failed")
    def execute_cartesian_path(self, x, y, z, rx, ry, rz):
        current_pose = self.arm.get_current_pose()
        self.arm.set_start_state_to_current_state()

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = current_pose.pose.position.x + x   
        target_pose.pose.position.y = current_pose.pose.position.y + y   
        target_pose.pose.position.z = current_pose.pose.position.z + z

        # You need to convert rx, ry, rz into a quaternion if you want to use them for orientation
        # Here, just assuming they are for orientation change
        quaternion = transforms3d.euler.euler2quat(rx, ry, rz)  # Assuming rx, ry, rz are Euler angles
        target_pose.pose.orientation.x = quaternion[0]
        target_pose.pose.orientation.y = quaternion[1]
        target_pose.pose.orientation.z = quaternion[2]
        target_pose.pose.orientation.w = quaternion[3]

        self.arm.set_goal_state(pose_stamped_msg=target_pose, pose_link='end_effector_link')

        self.plan_and_execute(self.moveit, self.arm, self.logger)


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
                    self.gripper.stop()
                    valid_command_detected = True

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
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = URMoveItMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()