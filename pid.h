/*
 * pid.h - interface for the PID control abstractions for rover navigation
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

#define CLAMP(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))

/**
 * @brief Robot history trajectory
 * 
 */
struct Point
{
    float x; // x-coordinate
    float y; // y-coordinate
    float v; // linear velocity
    float w; // angular velocity
    float t; // time
};

class TurtleBotController : public rclcpp::Node {
private:
    void controlLoop();

public:
    TurtleBotController();
    ~TurtleBotController();

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time node_start_time_; // Stores the node start time

    // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
    tf2::Quaternion tf_quat;

    // PID Gain
    double Kpel = 0.5, Kdel = 0.5, Kiel = 0.1;
    double Kpet = 3.0, Kdet = 1.0, Kiet = 0.1;

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
    
    //clamping parameters
    double vmax_ = 1.0;
    double Kvet_ = 1.0, Kvel_ = 1.0;

    double lookaheadDist_ = 0.2; //threshold or goal tolerance

    double dt = 0.1;  // Time step (0.1 seconds)

    std::vector<Point> Trajectory; //to save rover trajectory
    std::string fname="/home/pgurram/ros2_ws/src/hello/scripts/rover_state.txt";  //just a directory, please change

    // List of waypoints (desired positions)
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_waypoint_;
}