#include "pf_mpc/potential_field_node.hpp"

PotentialFieldNode::PotentialFieldNode(const rclcpp::NodeOptions &node_options, geometry_msgs::msg::Point target, double att_gain, double rep_gain, double obs_range) 
: Node("Potential_Field", node_options),
  target_(target),
  att_gain_(att_gain),
  rep_gain_(rep_gain),
  obs_range_(obs_range)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    
    this->pub_cmd_vel_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",qos_profile);
    this->sub_odom_=this->create_subscription<nav_msgs::msg::Odometry>("odom",qos_profile,
                                                                        std::bind(&PotentialFieldNode::CallBackOdom, this, _1));
    this->sub_scan_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan",qos_profile,
                                                                            std::bind(&PotentialFieldNode::CallBackScan, this, _1));
    this->timer_=this->create_wall_timer(100ms, std::bind(&PotentialFieldNode::CallBackTimer, this));
    
    this->potential_field.SetAttGain(this->att_gain_);
    this->potential_field.SetRepGain(this->rep_gain_);  
    this->potential_field.SetObsRange(this->obs_range_);
    this->potential_field.SetTarget(std::make_shared<geometry_msgs::msg::Point>(this->target_));
}

void PotentialFieldNode::CallBackOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = msg->pose.pose.position.x;
    robot_pose.position.y = msg->pose.pose.position.y;
    robot_pose.position.z = msg->pose.pose.position.z;

    robot_pose.orientation.x = msg->pose.pose.orientation.x;
    robot_pose.orientation.y = msg->pose.pose.orientation.y;
    robot_pose.orientation.z = msg->pose.pose.orientation.z;
    robot_pose.orientation.w = msg->pose.pose.orientation.w;

    this->potential_field.UpdateRobotPose(std::make_shared<geometry_msgs::msg::Pose>(robot_pose));
    this->pose_update_=true;
}

void PotentialFieldNode::CallBackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    this->potential_field.UpdateObstacles(this->obs_range_, msg);
}

void PotentialFieldNode::CallBackTimer(){
    if(pose_update_){
        geometry_msgs::msg::Twist cmd_vel = this->potential_field.ComputeVelocity();
        pub_cmd_vel_->publish(cmd_vel);
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"NOT RECEIVED POSE DATA");
    }
    
}

int main(int argc, char* argv[])
{
    geometry_msgs::msg::Point target_point;
    target_point.x=-1;
    target_point.y=-2;
    double obs_range=0.3;
    double att_gain = 0.1;
    double rep_gain = 0.2;

    const char * x_value_str = rcutils_cli_get_option(argv, argv+argc, "-x");
    const char * y_value_str = rcutils_cli_get_option(argv, argv+argc, "-y");
    const char * att_value_str = rcutils_cli_get_option(argv, argv+argc, "-a");
    const char * rep_value_str = rcutils_cli_get_option(argv, argv+argc, "-r");
    const char * range_value_str = rcutils_cli_get_option(argv, argv+argc, "-o");

    // target_point.x = atof(x_value_str);
    // target_point.y = atof(y_value_str);
    // att_gain = atof(att_value_str);
    // rep_gain = atof(rep_value_str);
    // obs_range = atof(range_value_str);

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions();

    auto potential_field_node = std::make_shared<PotentialFieldNode>(node_options, target_point, att_gain, rep_gain, obs_range);
    rclcpp::spin(potential_field_node);

    rclcpp::shutdown();

    return 0;
}