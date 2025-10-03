# dynamic_transform_publisher

A ROS 2 tool similar to `static_transform_publisher`, but for **dynamic transforms**.  
It subscribes to messages like Odometry or Pose and republishes them as TF frames.  
Useful for debugging, visualization in RViz, and test environments.

## Features
- Supports multiple message types:
  - `nav_msgs/msg/Odometry`
  - `geometry_msgs/msg/PoseStamped`
  - `geometry_msgs/msg/PoseWithCovarianceStamped`
  - `geometry_msgs/msg/TransformStamped`
  - `geometry_msgs/msg/Pose`
  - `geometry_msgs/msg/PoseWithCovariance`
- Simple command line interface
- Auto-detects parent/child frame ids (or manual override)

## Installation

# Clone into your ROS 2 workspace
cd ~/ros2_ws/src

git clone https://github.com/omer/dynamic_transform_publisher.git

cd ~/ros2_ws

colcon build --packages-select dynamic_transform_publisher

source install/setup.bash

# Usage

ros2 run dynamic_transform_publisher dynamic_transform_publisher_node \
<topic_name> <msg_type> <parent_frame|auto> <child_frame|auto>
  
# Examples

# Convert Odometry to TF
ros2 run dynamic_transform_publisher dynamic_transform_publisher_node /odom nav_msgs/msg/Odometry odom base_link


# Convert PoseStamped to TF
ros2 run dynamic_transform_publisher dynamic_transform_publisher_node /pose geometry_msgs/msg/PoseStamped map base_link


# Pass through TransformStamped
ros2 run dynamic_transform_publisher dynamic_transform_publisher_node /tf_in geometry_msgs/msg/TransformStamped auto auto

