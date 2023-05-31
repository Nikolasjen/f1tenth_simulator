#!/usr/bin/env python3
import subprocess
import random
import time
import os
import rospy
from std_msgs.msg import String
import rospkg
import rosgraph



F1TENTH_PATH = rospkg.RosPack().get_path('f1tenth_simulator')
MAP_PATH = os.path.join(F1TENTH_PATH, "maps/porto.yaml")
PARAMS_PATH = os.path.join(F1TENTH_PATH, "params.yaml")
TEMP_FILES_PATH = os.path.join(F1TENTH_PATH,"launch", "tmp")
PATH_TO_RVIZ_CONFIG_FILE = os.path.join(F1TENTH_PATH, "launch/simulator.rviz")
PATH_TO_TEMPLATE_RVIZ_CONFIG_FILE = os.path.join(F1TENTH_PATH, "launch/template_simulator.rviz")

LIST_OF_NODE_NAMES = ["racecar_simulator", "mux_controller", "behavior_controller", "keyboard", "mydrive_walker"]
POPULATION_SIZE = 4

results = []

class ResultsListener:
    def __init__(self, namespace):
        self.namespace = namespace
        self.resultTopic = "/{}/results".format(self.namespace)
        print("Python topic: ", self.resultTopic)

        #rospy.Publisher("/") Start a publisher that publishes the max simulation duration
        rospy.Subscriber(self.resultTopic, String, self.results_callback)
        print("Subscribing to {}".format(self.namespace))

    def results_callback(self, msg):
        global results
        print("Received results from {} finished with result: {}".format(self.namespace, msg.data))
        results.append((self.namespace, msg.data))
        self.terminate_ros_nodes()

    def terminate_ros_nodes(self):
        try:
            print("Attempting to terminate all nodes under namespace {}".format(self.namespace))
            nodes_list_command = "rosnode list"
            nodes_list_process = subprocess.Popen(nodes_list_command.split(), stdout=subprocess.PIPE)
            output, error = nodes_list_process.communicate()
            if error:
                print("Error while fetching node list: {}".format(error))
                return
            for node in output.decode().split('\n'):
                if node.startswith('/' + self.namespace):
                    kill_node_command = "rosnode kill {}".format(node)
                    subprocess.Popen(kill_node_command.split())
                    print("Terminated node: {}".format(node))
        except Exception as e:
            print("Error while terminating nodes: {}".format(e))




def start_subprocess(name, command, environment, shell=True):
    try:
        this_subprocess = subprocess.Popen(command, shell=shell, env=environment)
        print("running - {}".format(name))
        return this_subprocess
    except:
        print("FATAL ERROR WHILE STARTING {}".format(name))

def run_simulations(simulation_duration_seconds, numberOfSimulations):
    # Define the namespaces for each simulation
    simulations = []

    for i in range(1, numberOfSimulations+1):
        print("simulation {} has started".format(i))
        simulations.append({"namespace": "sim{}".format(i)})

    # Run each simulation in a separate subprocess
    processes = []
    print("Num of subprocesses: 0 = {}".format(len(processes)))

    for sim in simulations:
        # Set the environment variables for the simulation
        env = os.environ.copy()
        env["ROS_NAMESPACE"] = sim["namespace"]
        
        # Load parameters on the Parameter Server
        ## List of node names
        for n_name in LIST_OF_NODE_NAMES:
            param_command = "rosparam load {} /{}/{}".format(PARAMS_PATH, sim["namespace"], n_name)
            param_process = subprocess.Popen(param_command, shell=True, env=env)
            param_process.wait()  # Wait for the process to complete

        # Start individual nodes in subprocesses
        print("Starting subprocess {}".format(env["ROS_MASTER_URI"]))

        """
        processes.append(start_subprocess(name="map server",
                                          command="rosrun map_server map_server {}".format(MAP_PATH),
                                          environment=env))
        """
        processes.append(load_racecar_model(sim,env))
        print("running - racecar_model")

        processes.append(start_subprocess(name="simulator",
                                          command="rosrun f1tenth_simulator simulator",
                                          environment=env))
        processes.append(start_subprocess(name="mux",
                                          command="rosrun f1tenth_simulator mux",
                                          environment=env))
        processes.append(start_subprocess(name="behavior controller",
                                          command="rosrun f1tenth_simulator behavior_controller",
                                          environment=env))
        #processes.append(start_subprocess(name="keyboard",
        #                                  command="rosrun f1tenth_simulator keyboard",
        #                                  environment=env))
        processes.append(start_subprocess(name="mydrive",
                                          command="rosrun f1tenth_simulator mydrive_walk",
                                          environment=env))
        # More processes...

        # Start RViz for this simulation
        processes.append(run_RViz(sim, env))
        print("running - RViz")
        
        # Wait a moment before starting the next simulation
        #time.sleep(2)

    """ 
    param_tester = subprocess.Popen("rosnode list", shell=True, env=env)
    processes.append(param_tester)
    """

    time.sleep(2)
    # Wait for all simulations to finish
    for i, process in enumerate(processes):
        print("waiting... {}".format(i))
        process.wait()


def create_racecar_model_urdf(env):
    # Loading racecar_model parameters
    RACECAR_MODEL_PARAMS_PATH = os.path.join(F1TENTH_PATH, "racecar.xacro")

    # Converting xacro file to urdf and loading as parameter
    xacro_command = "xacro --inorder {} > {}/{}.urdf".format(RACECAR_MODEL_PARAMS_PATH, TEMP_FILES_PATH, "racecar") # sim["namespace"]
    xacro_process = subprocess.Popen(xacro_command, shell=True, env=env)
    xacro_process.wait()

def load_racecar_model(sim, env):
    # Read the URDF from the temporary file and set it as a parameter
    with open('{}/{}.urdf'.format(TEMP_FILES_PATH, "racecar"), 'r') as urdf_file: # sim["namespace"]
        urdf_content = urdf_file.read()
    param_command = "rosparam set /{}/racecar/robot_description '{}'".format(sim["namespace"], urdf_content)
    param_process = subprocess.Popen(param_command, shell=True, env=env)
    param_process.wait()

    # After setting the robot_description parameter:
    command_racecar_model = 'rosrun robot_state_publisher robot_state_publisher'
    # Update the environment variable for the namespace
    env['ROS_NAMESPACE'] = '{}/racecar'.format(sim["namespace"])
    process_racecar_model = subprocess.Popen(command_racecar_model, shell=True, env=env)
    # Reset the namespace after launching the robot_state_publisher node
    env['ROS_NAMESPACE'] = sim["namespace"]

    return process_racecar_model


def run_RViz(sim, env):
    # Create a new RViz config file by replacing the placeholder with the namespace
    rViz_template_file_path = os.path.join(sim['namespace'], PATH_TO_TEMPLATE_RVIZ_CONFIG_FILE)
    with open(rViz_template_file_path, 'r') as template_file:
        template_content = template_file.read()
    rviz_config_content = template_content.replace('NAMESPACE_PLACEHOLDER', sim["namespace"])
    rviz_config_path = '{}/rviz_config_{}.rviz'.format(TEMP_FILES_PATH, sim["namespace"])
    with open(rviz_config_path, 'w') as rviz_config_file:
        rviz_config_file.write(rviz_config_content)

    
    # Start RViz for this simulation (sim) -- headded
    rviz_command = 'rosrun rviz rviz -d ' + rviz_config_path
    process_rviz = subprocess.Popen(rviz_command, shell=True, env=env)
    return process_rviz
    """
    # Start RViz for this simulation (sim) using xvfb-run -- headless
    rviz_command = 'xvfb-run -a rosrun rviz rviz -d ' + rviz_config_path
    process_rviz = subprocess.Popen(rviz_command, shell=True, env=env)
    return process_rviz
    """
    




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
        # TODO: clean up launch/tmp folder.
        # Create racecar.urdf
        environment = os.environ.copy()
        create_racecar_model_urdf(environment)


        # Start the initial ROS master
        initial_roscore_process = start_roscore("http://localhost:11311")

        # Initialize the listener node
        rospy.init_node('main_script')

        # Open map
        process_map_server = start_subprocess(name="map server",
                                          command="rosrun map_server map_server {}".format(MAP_PATH),
                                          environment=environment)
        
        # Initialize ResultListeners for each simulation
        # For each simulation, create a separate subscriber
        for i in range(1, POPULATION_SIZE+1):
            ResultsListener("sim{}".format(i))
 
        run_simulations(simulation_duration_seconds=10, numberOfSimulations=POPULATION_SIZE)

        # Spin to keep the script alive
        rospy.spin()

        for res in results:
            print(res)

        if initial_roscore_process is not None:
            process_map_server.terminate()
            process_map_server.wait()
            #process.wait()
            initial_roscore_process.terminate()
            initial_roscore_process.wait()


    except:
        print("Process terminated... error in test_simulator code!")
        # Spin to keep the script alive
        rospy.spin()