/****************************************************************************
 * Optimizing Path Following with Industry-Calibrated PID Control for Turtlebot3 in ROS2 Foxy with GNUplot
 * Copyright (C) 2022-2023 Quang Nhat Le
 * 
 * Implementing a Calibrated PID for Path Tracking with Turtlebot3: 
 * Leveraging ROS2 Foxy and GNUplot in Industrial Applications
 *
 * @author Quang Nhat Le - quangle@umich.edu
 * @date 2023-Nov-2
 ****************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

#define CLAMP(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))
// #define RAD2DEG(x) (x * 180.0/M_PI)
#define RAD2DEG (180 / M_PI)
#define DEG2RAD(x) (x * M_PI / 180.0)
#define PRINTCMD(x) (std::cout << x << std::endl)

/**
 * @brief Robot history trajectory
 * 
 */
struct Point
{
	float x;  //x coordinate
	float y;  // y-coordinate
    float v;  //linear velocity
    float w;
	float t;  // time
};

class TurtleBotController : public rclcpp::Node {

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // Define a class member variable for storing the node start time
  rclcpp::Time node_start_time_;

  // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
  tf2::Quaternion tf_quat;
  
  // PID Gain
  double Kpel = 0.3, Kdel = 0.5, Kiel = 0.0;
  double Kpet = 1.5, Kdet = 1.0, Kiet = 0.1;

  // Desired position
  double desired_x_;
  double desired_y_;

  // PID errors
  double error_el_;
  double error_et_;
  double integral_el_;
  double integral_et_;
  double previous_error_el_;
  double previous_error_et_;
  
  //constant parameters
  double vmax_ = 0.5;
  double lookaheadDist_ = 0.3;

  //config parameters for obstacle avoidance
  double v_linear_ = 0.5;
  double w_angular_ = 1.0; // should be 1 ?
  double laserAngle__ = 40.0; //deg
  double responseDist_ = 0.55;
  bool moving_ ;

  //warning flags to avoid obs
  int right_warning;
  int left_warning;
  int front_warning;

  double dt = 0.1;  // Time step (0.1 seconds) 

  std::vector<Point> Trajectory;
  std::string fname="/home/quang_le/Documents/Michigan/Third_semester/ME590/ros2_mod_ws/src/hello/scripts/pid_X3_obs_avoid_2911_1.txt";  //just a directory, please change

  bool obs_close;

public:
  TurtleBotController();
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void savePath(std::string fname, std::vector<Point> &path);
  void animationPlot(std::string fname);

  geometry_msgs::msg::Twist cur_velocity_msg; //current velocity
  sensor_msgs::msg::LaserScan cur_scan_;

  double control_signal_v;
  double control_signal_w;



};

TurtleBotController::TurtleBotController() : Node("pid_obs_avoid_X3") {
  // Create the velocity publisher
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
   // Subscribe to odom topic for current robot state
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TurtleBotController::poseCallback, this, std::placeholders::_1));
  // Subscribe to laser scan topic for obstacle detection
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&TurtleBotController::laserCallback, this, std::placeholders::_1));

  // Initialize the node start time
  node_start_time_ = this->now();

  // Set the initial desired position
  desired_x_ = 6.0;
  desired_y_ = 6.0;

  //Init flags
  right_warning = 0;
  left_warning = 0;
  front_warning = 0;

  //robot move or not
  moving_ = false;

  // Initialize PID errors
  error_el_ = 0.0;
  error_et_ = 0.0;
  integral_el_ = 0.0;
  integral_et_ = 0.0;
  previous_error_el_ = 0.0;
  previous_error_et_ = 0.0;

  obs_close = false;
}

void TurtleBotController::poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose) {
    
    //get current time
    rclcpp::Time current_time = this->get_clock()->now();

    // Calculate the elapsed time in seconds
    float tt = (current_time - node_start_time_).seconds();

    // Calculate the errors
    double error_x_ =  desired_x_ - pose->pose.pose.position.x ;
    double error_y_ =  desired_y_ - pose->pose.pose.position.y ;
    double desired_dis = sqrt(pow(error_x_,2) + pow(error_y_,2));
    double theta_error = atan2(error_y_ ,error_x_);

    while (theta_error < -M_PI)
        theta_error += 2.0 * M_PI;
    while (theta_error > M_PI)
        theta_error -= 2.0 * M_PI;

    const geometry_msgs::msg::Quaternion& quat = pose->pose.pose.orientation;

    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    error_el_ = desired_dis;
    error_et_ = theta_error -yaw ;

    while (error_et_ < -M_PI)
        error_et_ += 2.0 * M_PI;
    while (error_et_ > M_PI)
        error_et_ -= 2.0 * M_PI;

    // Calculate the integrals
    integral_el_ += error_el_ * dt;
    integral_et_ += error_et_ * dt;

    // Calculate the derivatives
    double derivative_el_ = ( error_el_ - previous_error_el_) / dt;
    double derivative_et_ = (error_et_ - previous_error_et_) ;

    while (derivative_et_ < -M_PI)
        derivative_et_ += 2.0 * M_PI;
    while (derivative_et_ > M_PI)
        derivative_et_  -= 2.0 * M_PI;

    // Calculate the control signals using PID equation
    control_signal_v = Kpel * error_el_ + Kdel * derivative_el_  + Kiel * integral_el_;
    control_signal_w = Kpet * error_et_ + Kdet * (derivative_et_ /dt) + Kiet * integral_et_ ;
    
    while (control_signal_v > vmax_)
      control_signal_v = vmax_;
    while (control_signal_v < -vmax_)
      control_signal_v = -vmax_;
    
    // // Create the Twist message for publishing velocity commands
    // geometry_msgs::msg::Twist cmd_vel;
    // cmd_vel.linear.x = control_signal_v;
    // cmd_vel.angular.z = control_signal_w;
    
    // if(!obs_close){
    //     cur_velocity_msg.linear.x = control_signal_v ; // Go forward (linear velocity)
    //     cur_velocity_msg.angular.z = control_signal_w;
    // }

    // std::cout << "Input linear v :"  << control_signal_v << " angular u: " << control_signal_w 
    //  << " error dis: " << desired_dis << std::endl;

    static bool isGoal = false;

    if (desired_dis <= lookaheadDist_)
    {
      cur_velocity_msg.linear.x = 0.0;
      cur_velocity_msg.angular.z = 0.0;
      velocity_pub_->publish(cur_velocity_msg);
      RCLCPP_INFO(this->get_logger(), "TurtleBot reached the goal!");
      isGoal = true;
    

      savePath(fname, Trajectory);
      animationPlot(fname);

      // Shutdown the ROS 2 node to terminate the program
      rclcpp::shutdown();
      return;
    }

    // if (!isGoal)
    // {
    //   // Publish the velocity command
    //   velocity_pub_->publish(cur_velocity_msg); 
    //   std::cout << "Input linear v :"  <<  cur_velocity_msg.linear.x << " Angular u: " << cur_velocity_msg.angular.z << std::endl;
    // }

    // Update previous errors
    previous_error_el_ = error_el_;
    previous_error_et_ = error_et_;
    
    // saving the robot path using points
    Point  p;
    p.x= (float) pose->pose.pose.position.x;
    p.y= (float) pose->pose.pose.position.y;
    p.v = (float) pose->twist.twist.linear.x ;
    p.w = (float) pose->twist.twist.angular.z ;
    p.t = (float) tt;

    Trajectory.push_back(p);

}


void TurtleBotController::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
      cur_scan_ =  *scan;

      // RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
      // RCLCPP_INFO(this->get_logger(), "Range data at 0 deg:   %f", cur_scan_ .ranges[0]);
      // RCLCPP_INFO(this->get_logger(), "Range data at 15 deg:  %f", cur_scan_ .ranges[15]);
      // RCLCPP_INFO(this->get_logger(), "Range data at 345 deg: %f", cur_scan_ .ranges[345]);
      // RCLCPP_INFO(this->get_logger(), "-------------------------------------------");

    std::vector<float> ranges = scan->ranges;
    
    //reset flags
    right_warning = 0;
    left_warning = 0;
    front_warning = 0;

    for(size_t i = 0; i < ranges.size();i++){
        double angle_deg = RAD2DEG * (scan->angle_min + scan->angle_increment * static_cast<float>(i));
        // double angle_deg = static_cast<double>(RAD2DEG(scan->angle_min + i * scan->angle_increment));

        // if(static_cast<double>(160) > angle_deg && angle_deg > static_cast<double>(180 - laserAngle___)){
        //     if(ranges[i] < responseDist_ * 1.5){
        //         right_warning++;
        //     }
        // }

        // if(static_cast<double>(-160) < angle_deg && angle_deg < static_cast<double>(laserAngle___ - 180)){
        //     if(ranges[i] < responseDist_ * 1.5){
        //         left_warning++;
        //     }
        // }

        // if(std::abs(angle_deg) > static_cast<double>(160)){
        //     if(ranges[i] <= responseDist_ * 1.5){
        //         front_warning++;
        //     }
        // }
        if (160 > angle_deg && angle_deg > 180 - laserAngle__ && ranges[i] < responseDist_  * 1.5)
        {
            right_warning++;
        }
        else if (-160 < angle_deg && angle_deg < laserAngle__ - 180 && ranges[i] < responseDist_  * 1.5)
        {
            left_warning++;
        }
        else if (abs(angle_deg) > 160 && ranges[i] <= responseDist_  * 1.5)
        {
            front_warning++;
        }
    }

    moving_ = true; //robot starts moving

    if(front_warning >10 && left_warning > 10 && right_warning > 10){
        PRINTCMD("1, there are obstacles in the left and right, turn right");
        obs_close = true;
        cur_velocity_msg.linear.x = v_linear_;
        cur_velocity_msg.angular.z = -w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
    }else if(front_warning >10 && left_warning <= 10 && right_warning > 10){
        PRINTCMD("2, there is an obstacle in the middle right, turn left");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
        if(left_warning > 10 && right_warning <= 10){
            cur_velocity_msg.linear.x = 0.0;
            cur_velocity_msg.angular.z = -w_angular_;
            velocity_pub_->publish(cur_velocity_msg);
            // rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }else if(front_warning >10 && left_warning > 10 && right_warning <= 10){
        PRINTCMD("4. There is an obstacle in the middle left, turn right");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = -w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
        if(left_warning <= 10 && right_warning > 10){
            cur_velocity_msg.linear.x = 0.0;
            cur_velocity_msg.angular.z = w_angular_;
            velocity_pub_->publish(cur_velocity_msg);
            // rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }else if(front_warning >10 && left_warning < 10 && right_warning < 10){
        PRINTCMD("6, there is an obstacle in the middle, turn left");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
    }else if(front_warning <10 && left_warning > 10 && right_warning > 10){
        PRINTCMD("7. There are obstacles on the left and right, turn right");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = -w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(400));
    }else if(front_warning <10 && left_warning > 10 && right_warning <= 10){
        PRINTCMD("8, there is an obstacle on the left, turn right");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = -w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
    }else if(front_warning <10 && left_warning <= 10 && right_warning > 10){
        PRINTCMD("9, there is an obstacle on the right, turn left");
        obs_close = true;
        cur_velocity_msg.linear.x = 0.0;
        cur_velocity_msg.angular.z = w_angular_;
        velocity_pub_->publish(cur_velocity_msg);
        // rclcpp::sleep_for(std::chrono::milliseconds(200));
    }else if(front_warning <=10 && left_warning <= 10 && right_warning <= 10){
        PRINTCMD("10, no obstacles, go forward/ Activate PID");
        // obs_close = false;
        // cur_velocity_msg.linear.x = v_linear_;
        // cur_velocity_msg.angular.z = 0.0;
        cur_velocity_msg.linear.x = control_signal_v;
        cur_velocity_msg.angular.z = control_signal_w;
        velocity_pub_->publish(cur_velocity_msg);
        // continue;
    }
    
}

/**
 * @brief Save the trajectory and all states of robots to file to plot later
 * 
 * @param fname 
 * @param path 
 */
void TurtleBotController::savePath(std::string fname, std::vector<Point> &path)
{
    std::ofstream MyFile(fname);

    for (size_t i = 0; i < path.size(); i++)
    {
        MyFile << path[i].x << " " << path[i].y << " " << path[i].v << " " << path[i].w << " " << path[i].t << std::endl;
    }

    MyFile.close();
}

/**
 * @brief Plotting using Gnuplot right after finishing navigation
 * 
 * @param fname 
 */
void TurtleBotController::animationPlot(std::string fname){
    static FILE *gp;
    if(gp == NULL)
    {
        gp = popen("gnuplot -persist","w");
        fprintf(gp, "set colors classic\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "set xlabel Time [s]\n");
        fprintf(gp, "set tics font \"Arial, 14\"\n");
        fprintf(gp, "set xlabel font \"Arial, 14\"\n");
        fprintf(gp, "set ylabel font \"Arial, 14\"\n");
    }

    // Plot x versus y
    fprintf(gp, "set title 'Position (x vs. y)'\n");
    fprintf(gp, "plot \"%s\" using 1:2 with lines title 'x vs. y'\n", fname.c_str());
    fflush(gp);

    // Wait for user input (press Enter) before plotting the next graph
    printf("Press Enter to continue...\n");
    getchar();

    // Plot v versus t
    fprintf(gp, "set title 'Linear Velocity (v vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:3 with lines title 'v vs. t'\n", fname.c_str());
    fflush(gp);

    // Wait for user input (press Enter) before plotting the next graph
    printf("Press Enter to continue...\n");
    getchar();

    //Plot w versus t
    fprintf(gp, "set title 'Angular Velocity (w vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:4 with lines title 'w vs. t'\n", fname.c_str());
    fflush(gp);

    // Close gnuplot
    // fclose(gp);  // Do not close gnuplot here to keep the window open

    // Uncomment the line below if you want to automatically close gnuplot
    // int retVal = system("killall -9 gnuplot\n");
}



/**
 * @brief Calling the class node object or whatever it is, not sure make_shared means
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


