#!/bin/bash

# Debug function
debug() {
    echo "[Ros2 Docker Info]: $@" >&2
}

debug "Starting script with parameters: $@"

# Note: After updating this script, either rebuild the Docker image or run "update_workplace_entrypoint" in the container.

# Build ROS dependency
debug "Setting up ROS dependencies for ${ROS_DISTRO}"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source install/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# sudo apt-get update
# rosdep update

# Restart udev daemon
debug "Restarting udev daemon"
sudo service udev restart

# Set up common aliases
debug "Setting up common aliases"
echo "alias hdr_start='cd /workspaces/isaac_ros-dev; source /opt/ros/humble/setup.bash; cd /workspaces/isaac_ros-dev; source install/setup.bash; ros2 launch grand_tour_box hdr_cameras.launch.py'" >> ~/.bashrc
echo "alias hdr_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select v4l2_camera; source install/setup.bash;'" >> ~/.bashrc
echo "alias gt_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select grand_tour_box; source install/setup.bash;'" >> ~/.bashrc
echo "alias ros1_bridge_start='source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics'" >> ~/.bashrc
echo "alias hdr_encoder='ros2 launch isaac_ros_h264_encoder isaac_ros_h264_encoder.launch.py'" >> ~/.bashrc

# Ensures an interactive shell started with docker run has the aliases set.
echo "source ~/.bashrc" >> ~/.bash_profile
source ~/.bashrc

# Source the command handling script and pass all arguments
source /usr/local/bin/scripts/entrypoint-commands.sh "$@"

#  
# cd /workspaces/ros2_ws; colcon build --symlink-install --packages-select rosbag2_composable_recorder; source install/setup.bash;