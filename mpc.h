/*
 * mpc_obstacle_avoidance.h - interface for Model Predictive Control (MPC) for rover obstacle avoidance
 */

#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"

struct MPCProblem {
    double x_init, y_init, yaw_init;
    double x_goal, y_goal;
    const std::vector<std::vector<double>> obs_list;
    // const std::vector<double>& scan_ranges;

    MPCProblem(double x_init, double y_init, double yaw_init, double x_goal, double y_goal,
               const std::vector<std::vector<double>> obs_list);

    template <typename T>
    bool operator()(const T* const v, const T* const w, T* residual);
};

class MPCController : public rclcpp::Node {
public:
  MPCController(double x_goal, double y_goal);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
  
  void process(void);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    double x_goal_, y_goal_;
    std::vector<std::vector<double>> obs_list;
    // sensor_msgs::msg::LaserScan scan;
    // geometry_msgs::msg::Twist current_velocity;

};