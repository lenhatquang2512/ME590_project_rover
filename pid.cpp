/****************************************************************************
 * Advanced PID : Adaptive Fine-tuned PID Path Follow using Turtlebot3 with ROS2 and Gnuplot
 * Copyright (C) MIT 2022-2023 Quang Nhat Le
 *
 * @author Quang Nhat Le - quangle@umich.edu
 * @date   2023-Sep-19 (Last updated)
 ****************************************************************************/

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>

#include <math.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;

template <typename PointType>
void savePath(std::string fname, std::vector<PointType>& path) {
    std::ofstream MyFile(fname);

    for (size_t i = 0; i < path.size(); i++) {
        MyFile << path[i].x << " " << path[i].y << " " << path[i].v << " " << path[i].w << " " << path[i].t
               << std::endl;
    }

    MyFile.close();
}

void animationPlot(std::string fname) {
    static FILE* gp;
    if (gp == NULL) {
        gp = popen("gnuplot -persist", "w");
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

    // Plot w versus t
    fprintf(gp, "set title 'Angular Velocity (v vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:4 with lines title 'v vs. t'\n", fname.c_str());
    fflush(gp);

    // Close gnuplot
    // fclose(gp);  // Do not close gnuplot here to keep the window open

    // Uncomment the line below if you want to automatically close gnuplot
    // int retVal = system("killall -9 gnuplot\n");
}

void TurtleBotController::poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose) {
    // get current time
    rclcpp::Time current_time = this->get_clock()->now();

    // Calculate the elapsed time in seconds
    float tt = (current_time - node_start_time_).seconds();

    // desired yaw angle
    //  double desired_theta = atan2(desired_y_,desired_x_);

    // Calculate the errors
    double error_x_ = desired_x_ - pose->pose.pose.position.x;
    double error_y_ = desired_y_ - pose->pose.pose.position.y;
    // double yy = error_x_ * sin(-desired_theta) + error_y_ * cos(-desired_theta);
    double desired_dis = sqrt(pow(error_x_, 2) + pow(error_y_, 2));
    double theta_error = atan2(error_y_, error_x_);

    while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
    while (theta_error > M_PI) theta_error -= 2.0 * M_PI;

    const geometry_msgs::msg::Quaternion& quat = pose->pose.pose.orientation;

    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    // error_el_ = -yy;
    error_el_ = desired_dis;
    // error_et_ = desired_theta - yaw;
    error_et_ = theta_error - yaw;


    while (error_et_ < -M_PI) error_et_ += 2.0 * M_PI;
    while (error_et_ > M_PI) error_et_ -= 2.0 * M_PI;

    // Calculate the integrals

    integral_el_ += error_el_ * dt;
    integral_et_ += error_et_ * dt;

    integral_el_ = CLAMP(integral_el_, -vmax_, vmax_);

    // Calculate the derivatives
    double derivative_el_ = (error_el_ - previous_error_el_) / dt;
    double derivative_et_ = (error_et_ - previous_error_et_);

    while (derivative_et_ < -M_PI) derivative_et_ += 2.0 * M_PI;
    while (derivative_et_ > M_PI) derivative_et_ -= 2.0 * M_PI;

    // Calculate the control signals using PID equation
    // double control_signal_v = vmax_ - Kvel_ * fabs(error_el_) - Kvet_ * fabs(error_et_);
    double control_signal_v = Kp_el * error_el_ + Kd_el * derivative_el_ + Ki_el * integral_el_;
    // double control_signal_w =  Kp_et * error_et_ + Kd_et * (derivative_et_ /dt) + Ki_et * integral_et_ ;
    // control_signal_w += Kp_el * error_el_ + Kd_el * derivative_el_  + Ki_el * integral_el_;
    double control_signal_w = Kp_et * error_et_ + Kd_et * (derivative_et_ / dt) + Ki_et * integral_et_;

    while (control_signal_v > vmax_) control_signal_v = vmax_;
    while (control_signal_v < -vmax_) control_signal_v = -vmax_;

    // Create the Twist message for publishing velocity commands
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_signal_v;
    cmd_vel.angular.z = control_signal_w;

    // for debugging
    std::cout << "Input linear v :" << control_signal_v << " angular u: " << control_signal_w
              << " error dis: " << desired_dis << std::endl;

    if (desired_dis <= lookaheadDist_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        publisher_->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "TurtleBot reached the goal!");

        // Check if there are more waypoints
        if (current_waypoint_ < waypoints_.size()) {
            // Move to the next waypoint
            desired_x_ = waypoints_[current_waypoint_].first;
            desired_y_ = waypoints_[current_waypoint_].second;
            current_waypoint_++;
        } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached!");
            // Stop the robot
            desired_x_ = pose->pose.pose.position.x;
            desired_y_ = pose->pose.pose.position.y;

            savePath(fname, Trajectory);
            animationPlot(fname);

            // Shutdown the ROS 2 node to terminate the program
            rclcpp::shutdown();
            return;
        }

        // savePath(fname, Trajectory);
        // animationPlot(fname);

        // // Shutdown the ROS 2 node to terminate the program
        // rclcpp::shutdown();
        // return;
    }

    // Publish the velocity command
    publisher_->publish(cmd_vel);

    // Update previous errors
    previous_error_el_ = error_el_;
    previous_error_et_ = error_et_;

    // saving the robot path using points
    Point p;
    p.x = (float) pose->pose.pose.position.x;
    p.y = (float) pose->pose.pose.position.y;
    p.v = (float) pose->twist.twist.linear.x;
    p.w = (float) pose->twist.twist.angular.z;
    p.t = (float) tt;

    Trajectory.push_back(p);
}

void TurtleBotController::controlLoop() {
    // This function is called periodically for control loop updates
    // You can add any additional logic here
}

TurtleBotController::TurtleBotController()
    : Node("pid_naive_path_follow") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&TurtleBotController::poseCallback, this, std::placeholders::_1));

    // Initialize the node start time
    node_start_time_ = this->now();

    // Initialize waypoints
    // waypoints_ = {{-5.0, 3.0}, {3.0, 5.0}, {0.0, 0.0}}; // Add more waypoints as needed
    waypoints_ = {
        {0.0, 0.0},
        {3.0, 0.0},
        {3.0, 3.0},
        {6.0, 3.0},
        {6.0, 6.0}
    };   // Add more waypoints as needed
    // waypoints_ = {{-5.0, 3.0}}; // Add more waypoints as needed
    // for (double x = -5.0; x <= 5.0; x += 0.5) {
    //     // Calculate y using a polynomial function (e.g., y = ax^2 + bx + c)
    //     double a = 0.1;
    //     double b = 0.0;
    //     double c = 0.0;
    //     double y = a * x * x + b * x + c;

    //     // Add the (x, y) point as a waypoint
    //     waypoints_.push_back({x, y});
    // }
    current_waypoint_ = 0;

    // Set the initial desired position
    desired_x_ = waypoints_[current_waypoint_].first;
    desired_y_ = waypoints_[current_waypoint_].second;


    // Set the desired position
    // desired_x_ = -5.0;
    // desired_y_ = 3.0;

    // Initialize PID errors
    error_el_ = 0.0;
    error_et_ = 0.0;
    integral_el_ = 0.0;
    integral_et_ = 0.0;
    previous_error_el_ = 0.0;
    previous_error_et_ = 0.0;

    // Control loop timer
    timer_ = this->create_wall_timer(100ms, std::bind(&TurtleBotController::controlLoop, this));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleBotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
