# voicepipeline_ros2

This project uses ROS 2 and various Python packages for real-time communication and data processing. Below are the instructions for setting up the environment and installing dependencies.

## Installation

### 1. ROS 2 Installation

Ensure that you have ROS 2 installed on your system. You can follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for your operating system.

### 2. Python Dependencies

Install the following libraries:

```plaintext
whisper
sounddevice
```

### 3. Clone the Repository

```plaintext
cd your_ros2_ws/src
git clone https://github.com/nk2105/voicepipeline_ros2.git
```

### 4. Build and source 

```plaintext
cd your_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Usage

Launch the pipeline

```plaintext
ros2 launch voicepipeline voicepipeline.launch.py
```


