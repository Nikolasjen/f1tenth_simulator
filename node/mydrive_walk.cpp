#include <ros/ros.h> // Ros

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


#include <iostream> // std
#include <std_msgs/String.h>

// PID + read file
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tf/transform_datatypes.h>

// read map
#include <geometry_msgs/PoseArray.h>
#include <chrono>
#include <thread>



using namespace std; 

class MydriveWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, max_decel;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Listen for scan messages
    ros::Subscriber scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    // Distance from wall
    double WALL_KEEPAWAY_DIST = 0.2;
    double scan_field_of_view;
    double COLLISION_DIST = 0.01;
    int collisions = 0;
    bool breaking = false;
    double WALL_BREAK_DIST = 7.0;
    double breaking_speed = 0.0;
    double current_speed = 0.0;

    double CHECK_BREAK_RANGE = 45/3.5; // degrees
    ros::Time starting_time;
    ros::Time last_lap_time;

    ros::Duration time_to_pass = ros::Duration(1);
    ros::Time last_passed_time;
    bool has_started_tester = false;

    struct Point {
        double x;
        double y;
    };
    vector<Point> ppts;
    Point prev_pos = Point{0,0};

    // PID controller parameters
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double integral = 0.0;
    double prev_error = 0.0;

    // ROS publish lap results to EVO-ALG
    ros::Publisher results_pub;

    // Read CSV file
    string csv_name = "testMapPoints.csv";
    vector<Point> points;
    double distance_threshold = 1.0; // distance threshold for changing to next point
    Point current_point;
    Point next_point;

    // Write CSV file
    //string csv_laps_name = "/home/aleksander/catkin_ws/src/f1tenth_simulator/laps.csv";
    ros::Subscriber map_data_sub;
    ros::Publisher map_request_pub;
    bool map_received = false;



    void publish_speed_and_steering_angle(
        ackermann_msgs::AckermannDriveStamped &drive_st_msg,
        ackermann_msgs::AckermannDrive &drive_msg,
        double new_drive_angle = 0.0
        ) {
            // set steering angle
            drive_msg.steering_angle = new_drive_angle;
            
            // reset previous desired angle
            prev_angle = drive_msg.steering_angle;

            // set drive message in drive stamped message
            drive_st_msg.drive = drive_msg;

            // publish AckermannDriveStamped message to drive topic
            drive_pub.publish(drive_st_msg);
        }

    double round(double value, int numOfDecimalPlaces = 0) {
        int decimalDisplacement = std::pow(10, numOfDecimalPlaces);
        double temp = value * decimalDisplacement;
        return std::roundf(temp) / decimalDisplacement;
    }

    void reset_times() {
        starting_time = ros::Time::now();
        last_lap_time = starting_time;
        last_passed_time = starting_time;
    }

public:
    MydriveWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        string mydrive_drive_topic, odom_topic, scan_topic, results_topic, map_request_topic, mad_data_points_topic;
        n.getParam("mydrive_drive_topic", mydrive_drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("simulation_results_topic", results_topic);
        n.getParam("map_request_topic", map_request_topic);
        n.getParam("mad_data_points_topic", mad_data_points_topic);
        
        // get params
        n.getParam("scan_field_of_view", scan_field_of_view);

        std::string ns = ros::this_node::getNamespace();    // My stuff
        mydrive_drive_topic = ns + mydrive_drive_topic;     // My stuff
        odom_topic = ns + odom_topic;                       // My stuff
        scan_topic = ns + scan_topic;  
        results_topic = ns + results_topic;                 // My stuff
        map_request_topic = ns + map_request_topic;         // My stuff
        mad_data_points_topic = ns + mad_data_points_topic; // My stuff

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle); // 0.4189 rad
        n.getParam("max_decel", max_decel);
        breaking_speed = 1.38; // m/s ~ 4.968 km/hr i.e. roughly my average walking speed ;)

        // get PID parameters
        n.getParam("Kp", Kp);
        n.getParam("Ki", Ki);
        n.getParam("Kd", Kd);

        // set initial point and next point to go to
        //points = ppts;
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/temp.csv");
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/porto_centerline.csv");
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/Austin_centerline.csv");
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/IMS_centerline.csv");
        //ppts = points;

        

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(mydrive_drive_topic, 10);

        // Make a publisher for lap results
        results_pub = n.advertise<std_msgs::String>(results_topic, 10);

        map_request_pub = n.advertise<std_msgs::String>(map_request_topic, 10);
        std_msgs::String msg;
        msg.data = ns + " requesting map";
        std::this_thread::sleep_for(chrono::seconds(1));  // Wait for network connections to be established
        map_request_pub.publish(msg);
        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &MydriveWalker::odom_callback, this);
        // Start a subscriber to listen to scan messages
        scan_sub = n.subscribe(scan_topic, 1, &MydriveWalker::laser_callback, this);
        map_data_sub = n.subscribe(mad_data_points_topic, 1000, &MydriveWalker::mapData_Callback, this);

        reset_times();
    }

    void mapData_Callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        // Get datapoints from PoseArray message data
        points.clear(); // reset points
        for (const auto& pose : msg->poses) {
            double x = pose.position.x;
            double y = pose.position.y;

            points.emplace_back(Point{x,y});
        }
        ppts = points;
        map_received = true;  // set the flag
        next_point = points.front(); // Initialise next point
    }


// --------------------------------------------------------------------------------------------------------------

    double calculate_braking_distance() {
        double target_speed = breaking_speed;
        double decel = max_decel;

        // If the current speed is already lower than the target speed, no need to brake
        if (current_speed <= target_speed) {
            return 0.0;
        }

        // Calculate the braking distance based on current speed and target speed
        double braking_distance = ((current_speed * current_speed) - (target_speed * target_speed)) / (2 * decel) * 1.2;
        if (braking_distance < 0.0) {
            braking_distance = 0.0;
        }
        return braking_distance;
    }

    // Method related to and called on each laser scan call
    void laser_callback(const sensor_msgs::LaserScan & msg) {
        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        
        double rangeSize = msg.ranges.size(); // this should be 1080 - 3*360
        double leftCheck = (rangeSize / 2) - ((CHECK_BREAK_RANGE / 2) * 3);
        double rightCheck = (rangeSize / 2) + ((CHECK_BREAK_RANGE / 2) * 3);

        // Initialize the shortest distance
        double shortest_distance = std::numeric_limits<double>::infinity();
        
        for (int i = (int)leftCheck; i < (int)rightCheck; i++) {
            /*
            if (msg.ranges[i] < WALL_BREAK_DIST) {
                shortest_distance = msg.ranges[i];
            }
            */
            if (msg.ranges[i] < shortest_distance) {
                shortest_distance = msg.ranges[i];
                
            } 

            if (msg.ranges[i] < WALL_KEEPAWAY_DIST) {
                collisions++;
            }

        }
        double braking_distance = calculate_braking_distance();
        braking_distance *= 1.12;
        //cout << "shortest distance: " << shortest_distance << " braking_distance: " << braking_distance << endl;
        if (braking_distance > 0.0) {
            distance_threshold = 1.0 + 0.45 * braking_distance;
        }
    
        // Apply breaks if a wall is in close proximity
        if (shortest_distance < braking_distance || shortest_distance < 1.0) {
            breaking = true;
            
        } else {
            breaking = false;
        }
    }
// --------------------------------------------------------------------------------------------------------------

    void odom_callback(const nav_msgs::Odometry & msg) {
        // wait for map to be received before continuing.
        // give the map a chance to arrive
        if (!map_received) {
            return;
        }

        // Store current speed of the car
        current_speed = msg.twist.twist.linear.x;
        //cout << "current speed: " << current_speed << endl;
        
        // publishing is done in odom callback just so it's at the same rate as the sim
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        if (!breaking)
            drive_msg.speed = max_speed;
        else
            drive_msg.speed = breaking_speed;


        // Get the current position and heading of the robot
        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        double theta = tf::getYaw(msg.pose.pose.orientation);

        // Calculate the distance to the next point on the trajectory
        //double dx = max(next_point.x,x) - min(next_point.x,x);
        //double dy = max(next_point.y,y) - min(next_point.y,y);
        double dx = next_point.x - x;
        double dy = next_point.y - y;
        double distance_to_next_point = sqrt(dx*dx + dy*dy);

        // If the robot has reached the next point on the trajectory, update the current and next points
        if (distance_to_next_point < distance_threshold) {
            if (points.size() > 2) {
                points.erase(points.begin());
                next_point = points.front();
            } else {
                points = ppts;
                //ros::WallDuration meh_total = ros::WallTime::now() - meh;
                //ros::WallDuration meh_lap = ros::WallTime::now() - meh_last_lap_time;
                //meh_last_lap_time = ros::WallTime::now();

                //ros::Duration total_time = (ros::Time::now() - starting_time);
                ros::Duration lap_time = ros::Time::now() - last_lap_time;
                last_lap_time = ros::Time::now();
                /*
                cout << "- - - - - - -" << endl;
                //cout << "Total... m: " << round(total_time.toSec()/60) << ", s: " << total_time.toSec() - (round(total_time.toSec()/60) * 60) << endl;
                cout << "Lap...   m: " << round(lap_time.toSec()/60) << ", s: " << lap_time.toSec() - (round(lap_time.toSec()/60) * 60) << endl;
                cout << endl;
                //cout << "meh Total... m: " << round(meh_total.toSec()/60) << ", s: " << meh_total.toSec() - (round(meh_total.toSec()/60) * 60) << endl;
                //cout << "meh Lap...   m: " << round(meh_lap.toSec()/60) << ", s: " << meh_lap.toSec() - (round(meh_lap.toSec()/60) * 60) << endl;
                cout << "- - - - - - -" << endl;
                */

                std_msgs::String msg;
                msg.data = to_string(lap_time.toSec());
                results_pub.publish(msg);
                //write_CSV_laps(csv_laps_name, to_string(lap_time.toSec()));
            }

        }

        // Calculate the desired heading angle
        double desired_heading_angle = atan2(next_point.y - y, next_point.x - x);

        // Calculate the error between the current heading angle and the desired heading angle
        double error = desired_heading_angle - theta;

        // Normalize the error to be within [-pi, pi]
        if (error > M_PI) {
            error -= 2 * M_PI;
        } else if (error < -M_PI) {
            error += 2 * M_PI;
        }

        // Calculate the proportional term
        double P = Kp * error;

        // Calculate the integral term
        integral += error;

        double max_integral = max_steering_angle/2;
        double min_integral = -max_steering_angle/2;
        if (integral > max_integral) {
            integral = max_integral;
        } else if (integral < min_integral) {
            integral = min_integral;
        }
        double I = Ki * integral;

        // Calculate the derivative term
        double derivative = error - prev_error;
        double D = Kd * derivative;

        // Calculate the new steering angle
        double new_drive_angle = P + I + D;

        // Limit the steering angle to the maximum steering angle
        if (new_drive_angle > max_steering_angle) {
            new_drive_angle = max_steering_angle;
        } else if (new_drive_angle < -max_steering_angle) {
            new_drive_angle = -max_steering_angle;
        }
        

        //new_drive_angle = max(-max_steering_angle, min(max_steering_angle, new_drive_angle));
        bool isDriving = (abs(prev_pos.x - x) > 0.01 && abs(prev_pos.y - y) > 0.01);

        if (!has_started_tester && isDriving && ppts.size() > 2) {
            reset_times();
            has_started_tester = true;
        }

        // Publish the new speed and steering angle
        publish_speed_and_steering_angle(drive_st_msg, drive_msg, new_drive_angle);

        // Store the current error for the next iteration
        prev_error = error;
        prev_pos = Point{x, y};
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "mydrive_walker");
    MydriveWalker mdw;
    ros::spin();
    return 0;
}