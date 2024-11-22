#ifndef POTENTIAL_FIELD_NODE_HPP__
#define POTENTIAL_FIELD_NODE_HPP__

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>

#include "pf_mpc/potential_field.hpp"

#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PotentialFieldNode : public rclcpp::Node{
 public:
    explicit PotentialFieldNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions(), 
                                geometry_msgs::msg::Point target = geometry_msgs::msg::Point(),
                                double att_gain=0.1,
                                double rep_gain=0.1,
                                double obs_range=0.1);
    virtual ~PotentialFieldNode(){};

 private:
    PotentialField potential_field;

    void CallBackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void CallBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void CallBackTimer();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr timer_;

    double att_gain_;
    double rep_gain_;
    double obs_range_;
    geometry_msgs::msg::Point target_;

    bool pose_update_=false;

};

#endif // POTENTIAL_FIELD_NODE_HPP__