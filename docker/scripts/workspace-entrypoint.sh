#!/bin/bash

# Debug function
debug() {
    echo "[Ros2 Docker Info]: $@" >&2
}
debug "Starting script with parameters: $@"

# Set up common aliases
# echo "alias hdr_start='cd /workspaces/isaac_ros-dev; source /opt/ros/humble/setup.bash; cd /workspaces/isaac_ros-dev; source install/setup.bash; ros2 launch rosbag2_composable_recorder recorder.launch.py'" >> ~/.bashrc
# echo "alias hdr_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select v4l2_camera; source install/setup.bash;'" >> ~/.bashrc
# echo "alias recorder_build='cd /workspaces/isaac_ros-dev; colcon build --symlink-install --packages-select rosbag2_composable_recorder; source install/setup.bash;'" >> ~/.bashrc

# echo "alias ros1_bridge_start='source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics'" >> ~/.bashrc
# echo "alias hdr_encoder='ros2 launch isaac_ros_h264_encoder isaac_ros_h264_encoder.launch.py'" >> ~/.bashrc

# Ensures an interactive shell started with docker run has the aliases set.
# echo "source ~/.bashrc" >> ~/.bash_profile
# echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# echo "source install/setup.bash" >> ~/.bashrc


# Restart udev daemon
sudo service udev restart
source ~/.bashrc

echo TEST TEST TEST
# Main function to handle commands
handle_command() {
    if [ $# -eq 0 ]; then
        debug "No command provided, opening login shell"
        /bin/bash -l  # Open a login shell
    else
        debug "Executing Python script with arguments: $@"

        source /workspaces/ros2_ws/install/setup.bash
        python3 /usr/local/bin/scripts/entrypoint.py "$@"
    fi
}
# Execute handle_command with all passed arguments
handle_command $@