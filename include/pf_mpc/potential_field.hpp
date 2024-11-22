#ifndef POTENTIAL_FIELD_HPP__
#define POTENTIAL_FIELD_HPP__

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <iostream>

class PotentialField{
 public:
   PotentialField(){};

   ~PotentialField(){};

   void SetAttGain(double att_gain){this->att_gain_=att_gain;};
   void SetRepGain(double rep_gain){this->rep_gain_=rep_gain;};\
   void SetObsRange(double obs_range){this->obs_range_=obs_range;};

   void SetTarget(geometry_msgs::msg::Point::SharedPtr target);
   void UpdateRobotPose(geometry_msgs::msg::Pose::SharedPtr robot_pose);
   void UpdateObstacles(const double obs_range, const sensor_msgs::msg::LaserScan::SharedPtr scan);

   geometry_msgs::msg::Twist ComputeVelocity();

 private:
   geometry_msgs::msg::Point AttractiveField(const double att_gain, const geometry_msgs::msg::Point::SharedPtr target, const geometry_msgs::msg::Pose::SharedPtr robot_pose);
   geometry_msgs::msg::Point RepulsiveField(const double rep_gain, const double obs_range, const std::vector<geometry_msgs::msg::Point> obstacles,  const geometry_msgs::msg::Pose::SharedPtr robot_pose);

   geometry_msgs::msg::Point TotalForce();


   double EuclideanDistance(double x1, double y1, double x2, double y2);

   double att_gain_;
   double rep_gain_;
   double obs_range_;

   geometry_msgs::msg::Point::SharedPtr target_;
   geometry_msgs::msg::Pose::SharedPtr robot_pose_;
   geometry_msgs::msg::Twist velocity_ = geometry_msgs::msg::Twist();
   std::vector<geometry_msgs::msg::Point> obstacles_;

};

#endif // POTENTIAL_FIELD_HPP__