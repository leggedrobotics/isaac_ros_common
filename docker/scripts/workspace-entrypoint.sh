#!/bin/bash

# Debug function
debug() {
    echo "[Ros2 Docker Info]: $@" >&2
}
debug "Starting script with parameters: $@"

# Set up common aliases
# echo "alias recorder_build='cd /workspaces/ros2_ws; colcon build --symlink-install --packages-select rosbag2_composable_recorder; source install/setup.bash;'" >> ~/.bashrc
# echo "alias ros1_bridge_start='source /opt/ros/noetic/setup.bash; source /opt/ros/humble/setup.bash; cd /workspaces/bridge_ws; source install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics'" >> ~/.bashrc

# Restart udev daemon
sudo service udev restart
source ~/.bashrc

# Main function to handle commands
handle_command() {
    if [ $# -eq 0 ]; then
        debug "No command provided, opening login shell"
        /bin/bash -l  # Open a login shell
    else
        debug "Executing Python script with arguments: $@"

        source /opt/ros/humble/setup.bash; source /workspaces/ros2_ws/install/setup.bash; python3 /usr/local/bin/scripts/entrypoint.py "$@"
    fi
}
# Execute handle_command with all passed arguments
handle_command $@