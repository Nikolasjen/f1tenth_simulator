#!/usr/bin/env python3


import subprocess
import csv
import random
import copy
import time
import os
import sys
import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from roslaunch import rosparam

# Define csv file name
ORIGINAL_CSV_FILE = "porto_centerline.csv"
POINTS_DESTINATION_CSV_FILE = "temp.csv"
LAB_TIMES_CSV_FILE = "labs.csv"


path_first_line = []

# Evolution parameters
BEST_OUTCOME = 5.4
MUTATION_PERCENTAGE = 0.01 # +- 1% mutation
NUMBER_OF_INITIAL_SOLUTIONS = 2
NUMBER_OF_GENERATIONS = 5
PERCENTAGE_BEST_SOLUTION = 0.2 # 20%
solutions = []

def result_callback(msg, namespace):
    print("Received results from {} finished with result: {}".format(namespace, msg.data))

def read_csv_file(file_name):
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get the absolute path of the directory containing this script
    file_path = os.path.join(script_dir, '..', 'maps', file_name)  # Create the absolute file path
    with open(file_path, 'r') as f:
        csv_reader = csv.reader(f)
        path_points = []
        for i, row in enumerate(csv_reader):
            if i == 0:
                continue  # Skip the first line
            point = {'x': float(row[0]), 'y': float(row[1])}
            if len(row) > 2:
                point['speed'] = float(row[2])
            if len(row) > 3:
                point['direction'] = float(row[3])
            path_points.append(point)
        f.close()
    return path_points

def read_labs_times(file_name):
    this_dir = os.path.dirname(os.path.abspath(__file__))  # Get the absolute path of the directory containing this script
    file_path = os.path.join(this_dir, '..', file_name)  # Create the absolute file path to labs times
    labTimes = []

    # Open file in read mode
    with open(file_path, 'r') as file_handler:
        my_lines = file_handler.readlines()
        
        # Read each line
        for my_line in my_lines:
            # Split line by comma
            my_line = my_line.split(',')
            labTime_str = my_line[0].strip()
            
            # Cast labTime to double
            labTime = float(labTime_str)
            
            # Append labTime to labTimes list
            labTimes.append(labTime)
        file_handler.close()
    return labTimes

def read_csv_points(file_name):
    this_dir = os.path.dirname(os.path.abspath(__file__))  # Get the absolute path of the directory containing this script
    file_path = os.path.join(this_dir, '..', 'maps', file_name)  # Create the absolute file path to map points
    points = []
    
    # Open file in read mode
    with open(file_path, 'r') as file_handler:
        my_lines = file_handler.readlines()
        
        # Ignore first line
        #path_first_line = my_lines[0]
        #print(path_first_line)
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

def write_csv_file(file_name, path_points):
    this_dir = os.path.dirname(os.path.abspath(__file__))  # Get the absolute path of the directory containing this script
    file_path = os.path.join(this_dir, '..', 'maps', file_name)  # Create the absolute file path
    
    with open(file_path, 'w') as file_handler:
        csv_writer = csv.writer(file_handler)
        csv_writer.writerow(["x_m, y_m"]) # write path_first_line to first line
        
        # write each point as a row
        for point in path_points:
            csv_writer.writerow(point)
        
        file_handler.close()

def mutate_path_points(path_points, mutation_percentage):
    c_path_points = copy.deepcopy(path_points)
    mutated_path_points = []
    for x, y in c_path_points:
        mutation_x = random.uniform(-mutation_percentage, mutation_percentage)
        mutation_y = random.uniform(-mutation_percentage, mutation_percentage)
        mutated_path_points.append([x + x * mutation_x, y + y * mutation_y])
    return mutated_path_points

def run_simulations(simulation_duration_seconds, numberOfSimulations):
    # Launch command
    #command = "roslaunch f1tenth_simulator headless_simulator.launch"
    command = "roslaunch f1tenth_simulator simulator.launch"

    # Define the namespaces and master URIs for each simulation
    simulations = []

    for i in range(1, numberOfSimulations+1):
        print("simulation {} has started".format(i))
        simulations.append({"namespace": "sim{}".format(i), 
                            "master_uri": "http://localhost:{}".format(11311 + i)})

    # List to store subscribers
    subscribers = []

    # For each simulation, create a separate subscriber
    for sim in simulations:
        namespace = sim["namespace"]
        topic = "{}/simulation_results".format(namespace)
        subscriber = rospy.Subscriber(topic, String, result_callback, callback_args=namespace)
        subscribers.append(subscriber)
        print("Subscribing to {}".format(sim["namespace"]))

    
    # Run each simulation in a separate subprocess
    processes = []
    for sim in simulations:
        # Set the environment variables for the simulation
        env = os.environ.copy()
        env["ROS_NAMESPACE"] = sim["namespace"]
        env["ROS_MASTER_URI"] = sim["master_uri"]
        #rospy.ServiceProxy('/{}/gazebo/set_physics_properties'.format(sim["namespace"]))

        print("Starting subprocess {}".format(env["ROS_MASTER_URI"]))
        # Start the simulation in a subprocess
        process = subprocess.Popen(command, shell=True, env=env)
        processes.append(process)

        # Wait a moment before starting the next simulation
        time.sleep(2)

    # Wait for all simulations to finish
    for process in processes:
        process.wait()

    # Spin to keep the script alive
    rospy.spin()

    ## Use subprocess to launch the simulator file in the f1tenth_simulator package
    #simulator_process = None
    ##simulator_process = subprocess.Popen(['roslaunch', 'f1tenth_simulator', 'headless_simulator.launch'])
    #time.sleep(2)  # wait for simulator to launch
#
    #simulator_start_time = time.time()
    #while time.time() - simulator_start_time < simulation_duration_seconds:
    #    time.sleep(0.1)
#
    ## Terminate the simulator process
    #if (simulator_process is not None):
    #    simulator_process.terminate()
#
    #time.sleep(1.0) # Sleep for 1 seconds to allow simulation to terminate
    #
    ### Collect times to run a lab
    #labTimes = read_labs_times(LAB_TIMES_CSV_FILE)
    #return labTimes[0]
    return -1

# Define evaluation / fitness function
def fitness(labTime):

    penalty = 200 * abs(labTime - BEST_OUTCOME)

    # Add penalty
    # In this case we want a result close to BEST_OUTCOME
    if labTime <= BEST_OUTCOME:
        return BEST_OUTCOME
    else:
        return penalty

# Evolutional method - genetic evolution
def evolution():
    # Read path points from csv file
    original_path_points = read_csv_points(ORIGINAL_CSV_FILE)

    ## Initial population
    #for _ in range(NUMBER_OF_INITIAL_SOLUTIONS):
    #        solutions.append(
    #            mutate_path_points(original_path_points, MUTATION_PERCENTAGE)
    #        )

    times = []
    times.append(run_simulations(10, 2))
    #for i in range(NUMBER_OF_GENERATIONS -1):
    #    rankedSolutions = []
    #    for s_points in solutions:
    #        ## run simulation for s
    #        # Write mutated path points to csv file
    #        write_csv_file(POINTS_DESTINATION_CSV_FILE, s_points)
    #        time.sleep(0.1)  # wait for CSV to be written
#
    #        labTime = run_simulation(BEST_OUTCOME*2)
#
    #        # place calculated fitness and solution in ranking
    #        rankedSolutions.append( (fitness(labTime), s_points) )
#
    #    rankedSolutions.sort()
    #    times.append(rankedSolutions[0][0])

        #for time, coords in rankedSolutions:
        #    times.append(time)
        #    print(time)

        #print(f"=== Gen {i} best solution === ")
        #print(rankedSolutions[0])
#
        #if rankedSolutions[0][0] > BEST_OUTCOME/2:
        #    break
#
        ## Selection
        #bestSolutions = rankedSolutions[:int(NUMBER_OF_GENERATIONS*PERCENTAGE_BEST_SOLUTION)]
#
        #bestParameters = []
        #for s in bestSolutions:
        #    # Extract parameters (x,y,z) from best solutions
        #    bestParameters.append(s[1][0]) # x
        #    bestParameters.append(s[1][1]) # y
        #    bestParameters.append(s[1][2]) # z
#
        ## Variations / crossing genes + mutation
        #newGen = []
        #
        #for _ in range(NUMBER_OF_GENERATIONS):
        #    mutationFactor = rng.uniform(1-(mutationPercentage/2), 1+(mutationPercentage/2))
        #    p1 = rng.choice(bestParameters) * mutationFactor
        #    p2 = rng.choice(bestParameters) * mutationFactor
        #    p3 = rng.choice(bestParameters) * mutationFactor
#
        #    newGen.append( (p1, p2, p3) )
#
        ## redefine solutions
        #solutions = newGen
    for t in times:
        print(t)


if __name__ == '__main__':
    try:
        # Initialize the listener node
        rospy.init_node('main_script')

        # Read path points from csv file
        #original_path_points = read_csv_points(original_csv_file)
        
        random.seed(0) # Set seed for consistent testing...
        #evolution()
        run_simulations(simulation_duration_seconds=10, numberOfSimulations=1)
 
        #meh = subprocess.Popen(['roslaunch', 'f1tenth_simulator', 'headless_simulator.launch'], env={"ROS_MASTER_URI": "http://localhost:{}".format(11311+1)})
        #listOfProcesses = []
        #for i in range(3):
        #    listOfProcesses.append(
        #        1
        #    )
        

        #time.sleep(10)
        #for p in listOfProcesses:
        #    if (p != 1):
        #        p.terminate()
#
        #meh.terminate()
        
        
        ## Mutate path points
        #original_population = mutate_path_points(original_path_points, MUTATION_PERCENTAGE)
        #mutated_path_points = original_population
        ## Write mutated path points to csv file
        #write_csv_file(new_csv_file, original_population)
        #time.sleep(0.1)  # wait for CSV to be written
#
        ############################################################# times = []
        ### Collect times to run a lab
        ##labTimes = read_labs_times(lab_times_csv_file)
        ##for lt in labTimes:
        ##    print(lt)
        #
        ## Run simulation loop for 60 seconds ~1 min.
        #start_time = time.time()
        #while time.time() - start_time < 180:
#
        #    # Use subprocess to launch the simulator file in the f1tenth_simulator package
        #    simulator_process = None
        #    simulator_process = subprocess.Popen(['roslaunch', 'f1tenth_simulator', 'simulator.launch'])
        #    time.sleep(2)  # wait for simulator to launch
#
        #    simulator_start_time = time.time()
        #    while time.time() - simulator_start_time < 8:
        #        time.sleep(0.1)
#
        #    # Terminate the simulator process
        #    if (simulator_process is not None):
        #        simulator_process.terminate()
#
        #    time.sleep(1.0) # Sleep for 1 seconds to allow simulation to terminate
        #    
        #    # Mutate path points
        #    mutated_path_points = mutate_path_points(original_path_points, MUTATION_PERCENTAGE)
#
        #    # Write mutated path points to csv file
        #    write_csv_file(new_csv_file, mutated_path_points)
        #    time.sleep(0.1)  # wait for CSV to be written
        #
        ## Collect times to run a lab
        #labTimes = read_labs_times(lab_times_csv_file)
        #for lt in labTimes:
        #    print(lt)
        
    except:
        print("Process terminated... error in my_simulator code!")
        pass
