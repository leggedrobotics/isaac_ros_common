#!/usr/bin/env python3

import sys
import subprocess
import logging
import time
import yaml
from rclpy.node import Node
from rclpy import init, shutdown
from rclpy.executors import SingleThreadedExecutor
from rosbag2_composable_recorder.srv import StartRecording
from std_srvs.srv import Trigger


logging.basicConfig(level=logging.INFO, format='[Ros2 Docker Info]: %(message)s')

class Ros2RecorderClient(Node):
    def __init__(self):
        super().__init__('ros2_recorder_client')

    def call_service(self, service_name, srv_type, request, timeout_sec=3.0):
        try:
            client = self.create_client(srv_type, service_name)
            if not client.wait_for_service(timeout_sec=timeout_sec):
                logging.error(f"Service {service_name} is not available.")
                return False
            future = client.call_async(request)
            self.executor.spin_until_future_complete(future)
            if future.result() is not None:
                logging.info(f"Successfully called {service_name}.")
                return True
            else:
                logging.error(f"Failed to call service {service_name}.")
                return False
        except Exception as e:
            logging.error(f"Service call failed: {e}")
            return False
        finally:
            self.destroy_client(client)

    def start_recording_service(self, service_name, run_id, topics):
        request = StartRecording.Request()
        request.timestamp = run_id
        request.topics = topics
        return self.call_service(service_name, StartRecording, request)

    def stop_recording_service(self, service_name):
        request = Trigger.Request()
        return self.call_service(service_name, Trigger, request)

def run_command(command):
    try:
        subprocess.run(command, check=True, shell=True)
    except subprocess.CalledProcessError as e:
        logging.error(f"Command failed: {e}")
        sys.exit(1)

def start_recording(args):
    if len(args) < 2:
        logging.error("Error: No run_id or topics provided.")
        sys.exit(1)
    
    run_id = args[0]
    topics = [t for t in args[1:]]
    
    if len(topics) == 0:
        logging.error("Error: No topics provided.")
        sys.exit(1)

    logging.info(f"Run ID: {run_id}")
    logging.info(f"Topics: {topics}")
                 
    init()
    
    executor = SingleThreadedExecutor()
    
    for key in ["left", "right", "front"]:
        camera_topics = [t for t in topics if t.find(key) != -1]
        if len(camera_topics) != 0:
            service = f'/gt_box/recorder_hdr_{key}/start_recording'
            try:
                recorder_client = Ros2RecorderClient()
                executor.add_node(recorder_client)
                recorder_client.start_recording_service(service, run_id, camera_topics)
            finally:
                recorder_client.destroy_node()
        time.sleep(1)
        
    shutdown()

def stop_recording():
    logging.info("Stopping ROS2 bag recording")
    services = [
        "/gt_box/recorder_hdr_left/stop_recording",
        "/gt_box/recorder_hdr_right/stop_recording",
        "/gt_box/recorder_hdr_front/stop_recording"
    ]
    init()
    recorder_client = Ros2RecorderClient()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder_client)
        
    for service in services:
        recorder_client.stop_recording_service(service)
    shutdown()
    
def hdr_start():
    logging.info("Executing hdr_start alias")
    run_command("ros2 launch rosbag2_composable_recorder recorder.launch.py")

def ros1_bridge_start():
    ValueError("Not implemented")
    logging.info("Executing ros1_bridge_start alias")
    cmd = ""
    cmd += "source /opt/ros/noetic/setup.bash; "
    cmd += "source /opt/ros/humble/setup.bash; "
    cmd += "source install/setup.bash; "
    cmd += "ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics"
    run_command(cmd)

def main():
    if len(sys.argv) < 1:
        logging.error("No command provided")
        sys.exit(1)
    

    print("ALL", sys.argv)
    
    command = sys.argv[1]
    args = sys.argv[2:] if len(sys.argv) > 1 else None 
    print("CMD", command, " ARGS ", args)
    
    
    
    
    if command == "start_recording":
        start_recording(args)
    elif command == "stop_recording":
        stop_recording()
    elif command == "hdr_start":
        hdr_start()
    elif command == "ros1_bridge_start":
        ros1_bridge_start()
    else:
        logging.error(f"Unknown command: {command}")
        sys.exit(1)

if __name__ == "__main__":
    main()