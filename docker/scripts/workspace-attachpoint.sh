#!/bin/bash

# Debug function
debug() {
    echo "[Ros2 Docker Info]: $@" >&2
}

debug "Starting script with parameters: $@"

# Source the ROS setup based on the ROS distribution set in your environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source user-specific bash configurations if any
source ~/.bashrc

# Source the standalone command handling script and pass all arguments
source /usr/local/bin/scripts/workspace-entrypoint.sh "$@"
