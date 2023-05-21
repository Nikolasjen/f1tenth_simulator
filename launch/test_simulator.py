#!/usr/bin/env python3
import subprocess
import random
import time
import os
import rospy
from std_msgs.msg import String
import yaml
import rospkg
import rosgraph

F1TENTH_PATH = rospkg.RosPack().get_path('f1tenth_simulator')
MAP_PATH = os.path.join(F1TENTH_PATH, "maps/porto.yaml")
PARAMS_PATH = os.path.join(F1TENTH_PATH, "params.yaml")
params = None

def result_callback(msg, namespace):
    print("Received results from {} finished with result: {}".format(namespace, msg.data))

def run_simulations(simulation_duration_seconds, numberOfSimulations):
    if (params == None):
        print("ERROR: Simulation parameters were not loaded before simulation start attempt!")
        return -1

    # Define the namespaces and master URIs for each simulation
    simulations = []

    for i in range(1, numberOfSimulations+1):
        print("simulation {} has started".format(i))
        simulations.append({"namespace": "sim{}".format(i), 
                            "master_uri": "http://localhost:{}".format(11311 + i)})

    

    
    # Run each simulation in a separate subprocess
    processes = []
    roscore_processes = []
    for sim in simulations:
        # Set the environment variables for the simulation
        env = os.environ.copy()
        env["ROS_NAMESPACE"] = sim["namespace"]
        env["ROS_MASTER_URI"] = sim["master_uri"]

        # Start a separate roscore for each simulation
        roscore_process = start_roscore(sim["master_uri"])
        roscore_processes.append(roscore_process)

        # Set parameters on the Parameter Server
        for param_name, param_value in params.items():
            rospy.set_param(sim["namespace"] + "/" + param_name, param_value)

        # Start individual nodes in subprocesses
        command_map_server = "rosrun map_server map_server _map:={}".format(os.path.join(sim["namespace"],MAP_PATH))
        print(command_map_server)
        print(command_map_server)
        print(command_map_server)
        print(command_map_server)
        print(command_map_server)
        print(command_map_server)
        print(command_map_server)
        command_simulator = "rosrun f1tenth_simulator simulator _rosparam_load:={}".format(PARAMS_PATH)
        command_mux_controller = "rosrun f1tenth_simulator mux _rosparam_load:={}".format(PARAMS_PATH)
        command_behavior_controller = "rosrun f1tenth_simulator behavior_controller _rosparam_load:={}".format(PARAMS_PATH)
        command_keyboard = "rosrun f1tenth_simulator keyboard _rosparam_load:={}".format(PARAMS_PATH)
        command_mydrive_walker = "rosrun f1tenth_simulator mydrive_walk _rosparam_load:={}".format(PARAMS_PATH)
        # More nodes...

        print("Starting subprocess {}".format(env["ROS_MASTER_URI"]))
        process_map_server = subprocess.Popen(command_map_server, shell=True, env=env)
        process_simulator = subprocess.Popen(command_simulator, shell=True, env=env)
        process_mux_controller = subprocess.Popen(command_mux_controller, shell=True, env=env)
        process_behavior_controller = subprocess.Popen(command_behavior_controller, shell=True, env=env)
        process_keyboard = subprocess.Popen(command_keyboard, shell=True, env=env)
        process_mydrive_walker = subprocess.Popen(command_mydrive_walker, shell=True, env=env)
        # More processes...

        processes.append(process_map_server)
        processes.append(process_simulator)
        processes.append(process_mux_controller)
        processes.append(process_behavior_controller)
        processes.append(process_keyboard)
        processes.append(process_mydrive_walker)
        # More processes...        

        # Wait a moment before starting the next simulation
        time.sleep(2)
    
    # List to store subscribers
    subscribers = []

    # For each simulation, create a separate subscriber
    for sim in simulations:
        namespace = sim["namespace"]
        topic = "{}/simulation_results".format(namespace)
        subscriber = rospy.Subscriber(topic, String, result_callback, callback_args=namespace)
        subscribers.append(subscriber)
        print("Subscribing to {}".format(sim["namespace"]))
    
    # Wait for all simulations to finish
    for process in processes:
        process.wait()

    # Terminate all roscore processes
    for roscore_process in roscore_processes:
        if roscore_process is not None:
            roscore_process.terminate()
            roscore_process.wait()

    # Spin to keep the script alive
    rospy.spin()


def start_roscore(master_uri):
    # Start ROSCORE
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = master_uri

    roscore_process = subprocess.Popen("roscore -p {}".format(master_uri.split(":")[-1]), shell=True, env=env)
    time.sleep(2)  # Give it some time to start before checking again
    if not rosgraph.is_master_online():
        print("ERROR: Unable to start ROS master at {}!".format(master_uri))
        return None
    else:
        print("ROS master at {} started successfully.".format(master_uri))
        return roscore_process


if __name__ == '__main__':
    try:
        # Start the initial ROS master
        initial_roscore_process = start_roscore("http://localhost:11311")

        # Initialize the listener node
        rospy.init_node('main_script')

        # Load parameters from params.yaml file
        with open(PARAMS_PATH, 'r') as f:
            params = yaml.load(f, Loader=yaml.FullLoader)
 
        run_simulations(simulation_duration_seconds=10, numberOfSimulations=1)

        if initial_roscore_process is not None:
            initial_roscore_process.terminate()
            initial_roscore_process.wait()

    except:
        print("Process terminated... error in test_simulator code!")
        pass
