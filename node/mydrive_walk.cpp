#include <ros/ros.h> // Ros

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


#include <iostream> // std

// PID + read file
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tf/transform_datatypes.h>


using namespace std; 

class MydriveWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

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
    double WALL_BREAK_DIST = 1.5;
    double normal_speed = 0.0;
    double breaking_speed = 0.0;

    double CHECK_BREAK_RANGE = 45/2; // degrees
    ros::Time starting_time = ros::Time::now();
    ros::Time last_lab_time = ros::Time::now();

    ros::Duration time_to_pass = ros::Duration(1);
    ros::Time last_passed_time = ros::Time::now();
    ros::WallTime meh = ros::WallTime::now();
    ros::WallTime meh_last_lab_time = ros::WallTime::now();
    bool meh_tester = false;

    struct Point {
        double x;
        double y;
    };
    vector<Point> ppts;
    Point prev_pos = Point{0,0};
/*
    vector<Point> ppts = {
        Point{0.0, 0.0},
        Point{0.519, -0.716},
        Point{1.76, -1.49},
        Point{2.75, -1.86},
        Point{3.85, -1.49},
        Point{4.4, -0.362},
        Point{4.18, 0.662},
        Point{3.05, 1.36},
        Point{1.37, 1.51},
        Point{-0.0212, 1.66},
        Point{-1.23, 1.74},
        Point{-2.65, 1.9},
        Point{-4.7, 1.9},
        Point{-6.35, 1.9},
        Point{-8.03, 1.25},
        Point{-8.48, 0.116},
        Point{-7.97, -1.17},
        Point{-6.62, -1.5},
        Point{-5.26, -1.32},
        Point{-3.72, -0.739},
        Point{-2.44, -0.195},
        Point{-1.31, -0.016},
        Point{-0.173, -0.239}
        };
*/
    // PID controller parameters
    double Kp = 0.5;
    double Ki = 0.1;
    double Kd = 0.2;
    double integral = 0.0;
    double prev_error = 0.0;

    // Read CSV file
    string csv_name = "testMapPoints.csv";
    vector<Point> points;
    const double DISTANCE_THRESHOLD = 1.0; // distance threshold for changing to next point
    Point current_point;
    Point next_point;

    // Write CSV file
    string csv_labs_name = "/home/aleksander/catkin_ws/src/f1tenth_simulator/labs.csv";


    void publish_speed_and_steering_angle(
        ackermann_msgs::AckermannDriveStamped &drive_st_msg,
        ackermann_msgs::AckermannDrive &drive_msg,
        double new_drive_angle = 0.0
        ) {
            // set angle (add random change to previous angle)
            drive_msg.steering_angle = new_drive_angle;
            
            // reset previous desired angle
            prev_angle = drive_msg.steering_angle;

            // set drive message in drive stamped message
            drive_st_msg.drive = drive_msg;

            // publish AckermannDriveStamped message to drive topic
            drive_pub.publish(drive_st_msg);
        }


    void write_CSV_labs(string filePathAndName, string labTime) {
        ofstream myFile_Handler;
        string myLine;

        // File Open in the Read Mode
        myFile_Handler.open(filePathAndName, ios::out); // TODO: Change to ios::app if I want more than one value (don't delete preious times)

        if(myFile_Handler.is_open()) {
            myFile_Handler << labTime << endl;
            myFile_Handler.close();
        } else {
            cout << "Unable to open the file! - write_CSV_labs" << endl;
        }
    }
        
    void read_CSV_points(string filePathAndName) {
        ifstream myFile_Handler;
        string myLine;

        // File Open in the Read Mode
        myFile_Handler.open(filePathAndName, ios::in);

        if(myFile_Handler.is_open())
        {
            points.clear(); // reset points
            getline(myFile_Handler, myLine); // ignore first line
            
            // Read each line
            while(getline(myFile_Handler, myLine)) {

                stringstream ss(myLine);
                string x_str, y_str;
                getline(ss, x_str, ','); // x = first column
                getline(ss, y_str, ','); // y = second column

                // remove any white space characters from x_str and y_str
                x_str.erase(remove_if(x_str.begin(), x_str.end(), ::isspace), x_str.end());
                y_str.erase(remove_if(y_str.begin(), y_str.end(), ::isspace), y_str.end());

                // Cast to double
                double x = std::stod(x_str);
                double y = std::stod(y_str);

                points.emplace_back(Point{x,y});
            }

            // Close File
            myFile_Handler.close();
        
        } else {
            cout << "Unable to open the file! - read_CSV_points" << endl;
        }
    }

    double round(double value, int numOfDecimalPlaces = 0) {
        int decimalDisplacement = std::pow(10, numOfDecimalPlaces);
        double temp = value * decimalDisplacement;
        return std::roundf(temp) / decimalDisplacement;
    }

    void reset_times() {
        starting_time = ros::Time::now();
        last_lab_time = ros::Time::now();

        last_passed_time = ros::Time::now();
        meh = ros::WallTime::now();
        meh_last_lab_time = ros::WallTime::now();
    }

public:
    MydriveWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        string drive_topic, odom_topic, scan_topic;
        n.getParam("mydrive_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("scan_field_of_view", scan_field_of_view);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle); // 0.4189 rad
        normal_speed = max_speed;
        breaking_speed = normal_speed / 4;

        // get PID parameters
        n.getParam("Kp", Kp);
        n.getParam("Ki", Ki);
        n.getParam("Kd", Kd);

        cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;

        // set initial point and next point to go to
        //points = ppts;
        read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/temp.csv");
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/porto_centerline.csv");
        //read_CSV_points("/home/aleksander/catkin_ws/src/f1tenth_simulator/maps/Austin_centerline.csv");
        ppts = points;

        //current_point = points.front();
        next_point = points.front();

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &MydriveWalker::odom_callback, this);
        // Start a subscriber to listen to scan messages
        scan_sub = n.subscribe(scan_topic, 1, &MydriveWalker::laser_callback, this);
        starting_time = ros::Time::now();
    }

// --------------------------------------------------------------------------------------------------------------
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
            if (msg.ranges[i] < WALL_BREAK_DIST) {
                shortest_distance = msg.ranges[i];
            }

            if (msg.ranges[i] < WALL_KEEPAWAY_DIST) {
                collisions++;
            }

        }
        // cout << "shortest dist: " << shortest_distance << endl;

        if (shortest_distance < WALL_BREAK_DIST) {
            breaking = true;
            //cout << "BREAKING!" << endl;
        } else {
            breaking = false;
        }
    }
// --------------------------------------------------------------------------------------------------------------

    void odom_callback(const nav_msgs::Odometry & msg) {
        
        // publishing is done in odom callback just so it's at the same rate as the sim
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        if (!breaking)
            drive_msg.speed = normal_speed;
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
        if (distance_to_next_point < DISTANCE_THRESHOLD) {
            if (points.size() > 2) {
                points.erase(points.begin());
                next_point = points.front();
            } else {
                points = ppts;
                ros::WallDuration meh_total = ros::WallTime::now() - meh;
                ros::WallDuration meh_lab = ros::WallTime::now() - meh_last_lab_time;
                meh_last_lab_time = ros::WallTime::now();

                ros::Duration total_time = (ros::Time::now() - starting_time);
                ros::Duration lab_time = ros::Time::now() - last_lab_time;
                last_lab_time = ros::Time::now();
                cout << "- - - - - - -" << endl;
                cout << "Total... m: " << round(total_time.toSec()/60) << ", s: " << total_time.toSec() - (round(total_time.toSec()/60) * 60) << endl;
                cout << "Lab...   m: " << round(lab_time.toSec()/60) << ", s: " << lab_time.toSec() - (round(lab_time.toSec()/60) * 60) << endl;
                cout << endl;
                cout << "meh Total... m: " << round(meh_total.toSec()/60) << ", s: " << meh_total.toSec() - (round(meh_total.toSec()/60) * 60) << endl;
                cout << "meh Lab...   m: " << round(meh_lab.toSec()/60) << ", s: " << meh_lab.toSec() - (round(meh_lab.toSec()/60) * 60) << endl;
                cout << "- - - - - - -" << endl;
                write_CSV_labs(csv_labs_name, to_string(lab_time.toSec()));
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

        if (!meh_tester && isDriving) {
            reset_times();
            meh_tester = true;
        }

        if (collisions < 1 && isDriving) {
            /*
            ros::Time current_time = ros::Time::now();
            double current_speed = msg.twist.twist.linear.x;
            double current_angle = theta;
            */


            ros::Duration total_time = (ros::Time::now() - starting_time);
            ros::Duration check_time = (ros::Time::now() - last_passed_time);
            double time = total_time.toSec();
            if (check_time > time_to_pass) {
                cout << "Total time spent: " << round(time, 2) << "s" << ", (~" << round(time/60, 1) << "m)" << endl;
                last_passed_time = ros::Time::now();
            }
            /*
            cout << "[" << current_time << "]" << ", Speed: " << round(current_speed, 3) << ", Steering: " << round(current_angle, 3) << endl;
            cout << " -- -- -- -- " << endl;
            cout << "desired_heading_angle: " << desired_heading_angle << endl;
            cout << "Current angle: " << theta << endl;
            cout << "error: " << error << endl;
            cout << "New drive angle: " << new_drive_angle << endl;
            cout << "integral: " << integral << endl;
            */
            //cout << " -- -- " << endl;
            //cout << "Current pos: x: " << x << ", y: " << y << endl;
            //cout << "Desired pos: x: " << next_point.x << ", y: " << next_point.y << endl;
        }
        /*
        if (abs(error) > 1) {
            cout << "Error too great!: " << error << endl;
        }
        */
        

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