#!/bin/bash

# Commands appended to the Jetson Docker Launch script, in order to start
# various tasks in the foreground.

# Function to log debug messages
debug() {
    echo "[Ros2 Docker Info]: $*"
}

# Main function to handle commands
handle_command() {
    case "$1" in
        start_recording)
            shift  # Remove the first argument ('start_recording')
            debug "Command: start_recording, remaining params: $@"
            if [ -z "$1" ]; then
                echo "Error: No run_id provided."
                exit 1
            fi
            run_id="$1"
            debug "Run ID: $run_id"



            shift  # Remove the run_id from the arguments list

            # Build topic string dynamically based on additional arguments
            topics=""
            while [ -n "$1" ]; do
                topics+="$1 "
                shift  # Remove the topic from the arguments list
            done

            debug "Starting ROS2 bag recording with topics: $topics"
            source /opt/ros/humble/setup.bash
            source /workspaces/isaac_ros-dev/install/setup.bash
            
            service_name="/gt_box/recorder_hdr_left/start_recording"
            service_type="rosbag2_composable_recorder/srv/StartRecording"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{timestamp: '$run_id'}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi


            service_name="/gt_box/recorder_hdr_right/start_recording"
            service_type="rosbag2_composable_recorder/srv/StartRecording"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{timestamp: '$run_id'}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi

            service_name="/gt_box/recorder_hdr_front/start_recording"
            service_type="rosbag2_composable_recorder/srv/StartRecording"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{timestamp: '$run_id'}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi


            # ros2 bag record -s mcap $topics -o "/data/${run_id}/hdr" --max-bag-duration 300
            ;;
        stop_recording)
            debug "Starting ROS2 bag recording with topics: $topics"
            source /opt/ros/humble/setup.bash
            source /workspaces/isaac_ros-dev/install/setup.bash

            service_name="/gt_box/recorder_hdr_left/stop_recording"
            service_type="std_srvs/srv/Trigger"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi


            service_name="/gt_box/recorder_hdr_right/stop_recording"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi

            service_name="/gt_box/recorder_hdr_front/stop_recording"
            
            if ros2 service list | grep -q "$service_name"; then
                echo "Service $service_name is available. Proceeding with the call."
                ros2 service call "$service_name" "$service_type" "{}"
            else
                echo "Service $service_name is not available. Skipping the call."
            fi
            ;;
        hdr_start)
            debug "Executing hdr_start alias"
            source /opt/ros/humble/setup.bash
            cd /workspaces/isaac_ros-dev
            source install/setup.bash
            ros2 launch rosbag2_composable_recorder  recorder.launch.py
            ;;
        ros1_bridge_start)
            debug "Executing ros1_bridge_start alias"
            source /opt/ros/noetic/setup.bash
            source /opt/ros/humble/setup.bash
            source install/setup.bash
            ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
            ;;
        *)
            # If no known command is provided, open an interactive shell
            if [ -z "$1" ]; then
                debug "No command provided, opening login shell"
                /bin/bash -l  # Open a login shell
            else
                # Attempt to execute the command provided
                debug "No existing command found. Executing provided command: $@"
                $@
            fi
            ;;
    esac
}

# Execute handle_command with all passed arguments
handle_command "$@"
