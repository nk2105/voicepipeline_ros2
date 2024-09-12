import rclpy, os, json
from rclpy.node import Node
from ur_interfaces.msg import URWhisperCommands, URWhisperCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import math
from openai import OpenAI

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

        self.openai_client = OpenAI(
            api_key = os.getenv('OPENAI_API_KEY')
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

    def transcript_callback(self, msg):
        transcription = msg.data  # Convert to lowercase for consistency
        if not transcription.strip():  # Check if transcription is empty or only contains whitespace
            self.get_logger().info("Ignoring empty transcription")
            return  # Return immediately if transcription is empty
        chatgpt_message = """
You are to convert human directions as instructioons for movement for a mobile robot. The robot can move in x and y directions. The robot can also rotate along z axis. 
Movement and rotation are specified with a decimal value. They will br published as geometry_msgs.msg/Twist messages.
There will be two distinct commands: "Move" and "Rotate". Do not confuse them.

If a request to "move" an axis is made, but a value is not given, use the default value of 0.08.
If a request to "rotate" an axis is made, but a value is not given, use the default value of 0.2.
For any axis that is not mentioned use the value 0.0.
Multiple movements are directed using the word "and then".

For movement and rotation:
    left is negative in angular y axis
    right is positive in angular y axis
    forwards is positive in linear x axis
    backwards is negative in linear x axis
    pan left is negative in linear y axis
    pan right is positive in linear y axis


If you are asked to stop, set all values to 0.0.
If the transcription is not understood, stop the robot.

Do not include any explanations, only provide RFC8259 compliant JSON response following this format without deviation:
[
    {
        "linear": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
    }
]

If the directions have multiple steps, you may return those steps as multiple objects inside the top level JSON array.
If the directions are not valid, return an empty array.

---


""" + transcription
    
        try:
            completion = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": chatgpt_message}
                ]
            )

            if len(completion.choices) >= 1:
                outcome = json.loads(completion.choices[0].message.content)
                self.get_logger().info(outcome)
                commands = []
                for data in outcome:
                    command = Twist()
                    command.linear.x = data['linear']['x']
                    command.linear.y = data['linear']['y']
                    command.linear.z = data['linear']['z']
                    command.angular.x = data['angular']['x']
                    command.angular.y = data['angular']['y']
                    command.angular.z = data['angular']['z']
                    commands.append(command)

                if len(commands) > 0:
                    for command in commands:
                        self.command_publisher.publish(command)
                        self.get_logger().info("Published Twist command successfully")

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")


    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        if self.previous_position is not None:
            distance = math.sqrt(
                (current_position.x - self.previous_position.x) ** 2 +
                (current_position.y - self.previous_position.y) ** 2
            )
            self.total_distance += distance
            self.get_logger().info(f"Total distance: {self.total_distance}")
            if self.total_distance >= self.max_distance:
                self.get_logger().info("Stopping")
                self.twist_command.linear.x = 0.0
                self.twist_command.angular.z = 0.0
                self.command_publisher.publish(self.twist_command)
        self.previous_position = current_position

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            
