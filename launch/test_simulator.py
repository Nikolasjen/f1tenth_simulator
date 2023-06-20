#!/usr/bin/env python3
import subprocess
import random
import time
import os
import rospy
from std_msgs.msg import String
import rospkg
import rosgraph
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray, Pose, Point


F1TENTH_PATH = rospkg.RosPack().get_path('f1tenth_simulator')

# --------------- YOU CAN CHOOSE FREELY BETWEEN THESE MAPS --------------- #
#MAP_PATH = os.path.join(F1TENTH_PATH, "maps/porto.yaml")
#MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/porto_centerline.csv")

MAP_PATH = os.path.join(F1TENTH_PATH, "maps/Austin_map.yaml")
MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/Austin_centerline.csv")
#MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/Austin_new_line.csv")

#MAP_PATH = os.path.join(F1TENTH_PATH, "maps/levine_blocked.yaml")
##MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/levine_centerline.csv")
#MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/levine_new_line.csv")

#MAP_PATH = os.path.join(F1TENTH_PATH, "maps/IMS_map.yaml")
#MAP_CENTERLINE_PATH = os.path.join(F1TENTH_PATH, "maps/IMS_centerline.csv")

# --------------- PARAMETER DEFINITION --------------- #
PARAMS_PATH = os.path.join(F1TENTH_PATH, "params.yaml")
TEMP_FILES_PATH = os.path.join(F1TENTH_PATH,"launch", "tmp")
PATH_TO_RVIZ_CONFIG_FILE = os.path.join(F1TENTH_PATH, "launch/simulator.rviz")
PATH_TO_TEMPLATE_RVIZ_CONFIG_FILE = os.path.join(F1TENTH_PATH, "launch/template_simulator.rviz")

LIST_OF_NODE_NAMES = ["racecar_simulator", "mux_controller", "behavior_controller", "keyboard", "mydrive_walker"]
POPULATION_SIZE = 1 # Number of simulations that will run
RUN_WITH_RVIZ = True # Set to False, if you don't want to have a visual.
MAX_RUNNING = 10

results = []
current_num_of_running_sims = 0
solutions = [] # One set of path points = one possible solution


class SimulationHandler:
    def __init__(self, namespace, solutions_idx):
        self.namespace = namespace
        self.resultTopic = "/{}/results".format(self.namespace)
        self.mapTopic = "/{}/map_request".format(self.namespace)
        self.solutions_idx = solutions_idx

        # Start publishers that publishes
        rospy.Subscriber(self.resultTopic, String, self.results_callback)   # Subscribe to the simulation result topic
        rospy.Subscriber(self.mapTopic, String, self.map_request_callback)  # Subscribe to the map request topic
        print("Subscribing to {}".format(self.namespace))

    def results_callback(self, msg):
        global results
        global current_num_of_running_sims
        print("Received results from {} finished with result: {}".format(self.namespace, msg.data))
        self.terminate_ros_nodes()
        results.append((self.namespace, msg.data))
        current_num_of_running_sims -= 1

    def map_request_callback(self, msg):
        print("received map request from {}".format(self.namespace))
        pose_array_msg = PoseArray()
        global solutions
        for point in solutions[self.solutions_idx]:
            pose_msg = Pose()
            pose_msg.position.x = point[0]  # x-coordinate
            pose_msg.position.y = point[1]  # y-coordinate
            # We ignore the z-coordinate and the orientation, so they will default to 0.0

            pose_array_msg.poses.append(pose_msg)

        # Publish the map data
        self.map_publisher = rospy.Publisher("/{}/map_data".format(self.namespace), PoseArray, queue_size=10)
        rospy.sleep(1)  # Wait for network connections to be established
        self.map_publisher.publish(pose_array_msg)


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
                if node.startswith('/' + self.namespace + '/'):
                    kill_node_command = "rosnode kill {}".format(node)
                    subprocess.Popen(kill_node_command.split())
                    print("Terminated node: {}".format(node))
        except Exception as e:
            print("Error while terminating nodes: {}".format(e))


def read_csv_points():
    points = []
    
    # Open file in read mode
    with open(MAP_CENTERLINE_PATH, 'r') as file_handler:
        my_lines = file_handler.readlines()
        
        # Ignore first line
        my_lines = my_lines[1:] # TODO: MOVE TO MUTATE FUNCTION
        
        # Read each line
        for my_line in my_lines:
            # Split line by comma
            my_line = my_line.split(',')
            
            # Get x and y values
            x_str = my_line[0].strip()
            y_str = my_line[1].strip()
            
            # Cast x and y to double
            x = float(x_str)
            y = float(y_str)
            
            # Append point to points list
            points.append([x, y])

        file_handler.close()
    return points


def start_subprocess(name, command, environment, shell=True):
    try:
        this_subprocess = subprocess.Popen(command, shell=shell, env=environment)
        #print("running - {}".format(name))
        return this_subprocess
    except:
        print("FATAL ERROR WHILE STARTING {}".format(name))

def run_simulations(max_running, numberOfSimulations):
    global current_num_of_running_sims
    # Define the namespaces for each simulation
    simulations = []

    for i in range(1, numberOfSimulations+1):
        simulations.append({"namespace": "sim{}".format(i)})

    # Run each simulation in a separate subprocess
    processes = []
    

    for sim in simulations:
        # Ensure only max_running simulations at once
        while current_num_of_running_sims >= max_running:
            time.sleep(1) # wait a second
        current_num_of_running_sims += 1

        print("{} has started".format(sim['namespace']))
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
        processes.append(load_racecar_model(sim,env))
        processes.append(start_subprocess(name="simulator",
                                          command="rosrun f1tenth_simulator simulator",
                                          environment=env))
        processes.append(start_subprocess(name="mux",
                                          command="rosrun f1tenth_simulator mux",
                                          environment=env))
        processes.append(start_subprocess(name="behavior controller",
                                          command="rosrun f1tenth_simulator behavior_controller",
                                          environment=env))
        processes.append(start_subprocess(name="mydrive",
                                          command="rosrun f1tenth_simulator mydrive_walk",
                                          environment=env))
        if RUN_WITH_RVIZ:
            # Start RViz for this simulation
            processes.append(run_RViz(sim, env))

    time.sleep(2)
    # Wait for all simulations to finish
    for process in processes:
        process.wait()


def create_racecar_model_urdf(env):
    # Loading racecar_model parameters
    RACECAR_MODEL_PARAMS_PATH = os.path.join(F1TENTH_PATH, "racecar.xacro")

    # Converting xacro file to urdf and loading as parameter
    xacro_command = "xacro {} > {}/{}.urdf".format(RACECAR_MODEL_PARAMS_PATH, TEMP_FILES_PATH, "racecar") # sim["namespace"]
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
    
# Yeah... scrap this for now... I mean, it starts but doesn't close, so I'll run without a physics simulator for now.
def run_Gazebo(sim, env):
    # Define the path to your gazebo world file
    gazebo_world_file = os.path.join(F1TENTH_PATH, "worlds/porto.world")
    # Define the path to your robot's URDF file
    robot_urdf_file = os.path.join(TEMP_FILES_PATH, "racecar.urdf")

    # Launch gazebo with your world file
    gazebo_command = "roslaunch gazebo_ros empty_world.launch world_name:={} gui:=true".format(gazebo_world_file)
    process_gazebo = subprocess.Popen(gazebo_command, shell=True, env=env)

    # Load your robot into the gazebo world
    load_model_command = "rosrun gazebo_ros spawn_model -file {} -urdf -z 1 -model my_robot".format(robot_urdf_file)
    process_load_model = subprocess.Popen(load_model_command, shell=True, env=env)

    return process_gazebo, process_load_model


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


def terminate_remaining_ros_nodes():
    try:
        nodes_list_command = "rosnode list"
        nodes_list_process = subprocess.Popen(nodes_list_command.split(), stdout=subprocess.PIPE)
        output, error = nodes_list_process.communicate()
        if error:
            print("Error while fetching node list: {}".format(error))
            return
        
        for node in output.decode().split('\n'):
            print("Attempting to terminate node{}".format(node))
            if node.startswith('/'):
                kill_node_command = "rosnode kill {}".format(node)
                subprocess.Popen(kill_node_command.split())
                print("Terminated node: {}".format(node))
    except Exception as e:
        print("Error while terminating nodes: {}".format(e))

def main_function():
    try:
        # TODO: clean up launch/tmp folder.
        # Create racecar.urdf
        environment = os.environ.copy()
        create_racecar_model_urdf(environment)
        original_points = read_csv_points()
        # Start the initial ROS master
        initial_roscore_process = start_roscore("http://localhost:11311")
        time.sleep(1)

        # Initialize the listener node
        rospy.init_node('main_script')

        # Open map
        process_map_server = start_subprocess(name="map server",
                                          command="rosrun map_server map_server {}".format(MAP_PATH),
                                          environment=environment)
        time.sleep(2)
        # Initialize ResultListeners for each simulation
        # For each simulation, create a separate subscriber
        simulation_handlers = []
        for i in range(1, POPULATION_SIZE+1):
            solutions.append(original_points)
            simulation_handlers.append(SimulationHandler("sim{}".format(i), i-1))
 
        run_simulations(max_running=MAX_RUNNING, numberOfSimulations=POPULATION_SIZE)

        for res in results:
            print(res)
        print("SIMULATIONS COMPLETE, USE CTRL + C TO CLOSE")

        
        # terminate map server
        if process_map_server is not None:
            process_map_server.terminate()
            process_map_server.wait()
        
        #terminate_remaining_ros_nodes()

        # Spin to keep the script alive
        rospy.spin()

        if initial_roscore_process is not None:
            #process.wait()
            initial_roscore_process.terminate()
            initial_roscore_process.wait()

    except:
        print("Process terminated... error in test_simulator code!")
        
        # Ensure termination of roscore and other processes
        if initial_roscore_process is not None:
            initial_roscore_process.terminate()
            initial_roscore_process.wait()

        if process_map_server is not None:
            process_map_server.terminate()
            process_map_server.wait()
        
        # Finally, spin to keep the script alive if needed
        rospy.spin()

if __name__ == '__main__':
    main_function()