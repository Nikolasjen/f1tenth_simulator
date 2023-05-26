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

LIST_OF_NODE_NAMES = ["racecar_simulator", "mux_controller", "behavior_controller", "keyboard", "mydrive_walker"]

def result_callback(msg, namespace):
    print("Received results from {} finished with result: {}".format(namespace, msg.data))

def run_simulations(simulation_duration_seconds, numberOfSimulations):
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

        # Load parameters on the Parameter Server

        ## List of node names (e.g. the name for the simulator node is racecar_simulator 
        ## - see in the ROS initialisation, in the main method at the bottom of each nodes code)
        for n_name in LIST_OF_NODE_NAMES:
            param_command = "rosparam load {} /{}/{}".format(PARAMS_PATH, sim["namespace"], n_name)
            param_process = subprocess.Popen(param_command, shell=True, env=env)
            param_process.wait()  # Wait for the process to complete
        

        # Start individual nodes in subprocesses
        localMapPath = os.path.join(sim["namespace"],MAP_PATH)
        command_map_server = "rosrun map_server map_server {}".format(localMapPath)
        
        command_simulator = "rosrun f1tenth_simulator simulator"
        command_racecar_model = 'roslaunch f1tenth_simulator racecar_model.launch'
        command_mux_controller = "rosrun f1tenth_simulator mux"
        command_behavior_controller = "rosrun f1tenth_simulator behavior_controller"
        command_keyboard = "rosrun f1tenth_simulator keyboard"
        command_mydrive_walker = "rosrun f1tenth_simulator mydrive_walk"
        # More nodes...
        
        print("Starting subprocess {}".format(env["ROS_MASTER_URI"]))
        process_map_server = subprocess.Popen(command_map_server, shell=True, env=env)
        time.sleep(3)
        print("running - map server")
        process_racecar_model = subprocess.Popen(command_racecar_model, shell=True, env=env)
        time.sleep(3)
        print("running - racecar_model")
        process_simulator = subprocess.Popen(command_simulator, shell=True, env=env)
        time.sleep(2)
        print("running - simulator")
        process_mux_controller = subprocess.Popen(command_mux_controller, shell=True, env=env)
        #time.sleep(2)
        print("running - mux")
        process_behavior_controller = subprocess.Popen(command_behavior_controller, shell=True, env=env)
        #time.sleep(2)
        print("running - behavior controller")
        process_keyboard = subprocess.Popen(command_keyboard, shell=True, env=env)
        #time.sleep(2)
        print("running - keyboard")
        process_mydrive_walker = subprocess.Popen(command_mydrive_walker, shell=True, env=env)
        print("running - mydrive")
        # More processes...

        #parem_tester = subprocess.Popen("rosparam list", shell=True, env=env)
        #print("---")
        #node_tester = subprocess.Popen("rosnode list", shell=True, env=env)
        #processes.append(parem_tester)
        #processes.append(node_tester)

        processes.append(process_map_server)
        processes.append(process_simulator)
        processes.append(process_racecar_model)
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
        print("Waiting...")
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

        # Open map
        #command_map_server = "rosrun map_server map_server {}".format(MAP_PATH)
        #process_map_server = subprocess.Popen(command_map_server, shell=True, env=os.environ.copy())
        #time.sleep(3)
        #print("running - map")
 
        run_simulations(simulation_duration_seconds=10, numberOfSimulations=1)

        if initial_roscore_process is not None:
            #process_map_server.terminate()
            #process_map_server.wait()
            initial_roscore_process.terminate()
            initial_roscore_process.wait()

    except:
        print("Process terminated... error in test_simulator code!")
        pass
