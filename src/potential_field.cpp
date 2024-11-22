#include "pf_mpc/potential_field.hpp"

void PotentialField::SetTarget(geometry_msgs::msg::Point::SharedPtr target){
    this->target_ = target;
}

void PotentialField::UpdateRobotPose(geometry_msgs::msg::Pose::SharedPtr robot_pose){
    this->robot_pose_ = robot_pose;
}

void PotentialField::UpdateObstacles(const double obs_range ,const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    this->obstacles_.clear();
    if(this->velocity_.linear.x>=0)
    {
        for (size_t i = 0; i < 91; ++i) {
            double distance = scan->ranges[i];
            if (distance < obs_range) {
                double angle = scan->angle_min + i * scan->angle_increment;
                geometry_msgs::msg::Point obstacle;
                obstacle.x = robot_pose_->position.x + distance * cos(angle);
                obstacle.y = robot_pose_->position.y + distance * sin(angle);
                this->obstacles_.push_back(obstacle);
            }
        }

        for (size_t i = 271; i < 360; ++i) {
            double distance = scan->ranges[i];
            if (distance < obs_range) {
                double angle = scan->angle_min + i * scan->angle_increment;
                geometry_msgs::msg::Point obstacle;
                obstacle.x = robot_pose_->position.x + distance * cos(angle);
                obstacle.y = robot_pose_->position.y + distance * sin(angle);
                this->obstacles_.push_back(obstacle);
            }
        }
    }
    else
    {
        for (size_t i = 91; i < 270; ++i) {
            double distance = scan->ranges[i];
            if (distance < obs_range) {
                double angle = scan->angle_min + i * scan->angle_increment;
                geometry_msgs::msg::Point obstacle;
                obstacle.x = robot_pose_->position.x + distance * cos(angle);
                obstacle.y = robot_pose_->position.y + distance * sin(angle);
                this->obstacles_.push_back(obstacle);
            }
        }
    }
}

geometry_msgs::msg::Twist PotentialField::ComputeVelocity(){
    geometry_msgs::msg::Point force_att = this->AttractiveField(this->att_gain_, this->target_, this->robot_pose_);
    geometry_msgs::msg::Point force_rep = this->RepulsiveField(this->rep_gain_, this->obs_range_, this->obstacles_, this->robot_pose_);

    geometry_msgs::msg::Point force_total = geometry_msgs::msg::Point();
    force_total.x = force_att.x + force_rep.x;
    force_total.y = force_att.y + force_rep.y;

    double distance = this->EuclideanDistance(this->robot_pose_->position.x, this->robot_pose_->position.y, this->target_->x, this->target_->y);
    if((this->robot_pose_->position.x - this->target_->x)<0 || (this->robot_pose_->position.y - this->target_->y)<0){
        this->velocity_.linear.x=0.1*(distance);
    }
    else{
        this->velocity_.linear.x=0.1*(-distance);
    }

    if(this->velocity_.linear.x>0.1)
    {
        this->velocity_.linear.x=0.1;
    }
    else if(this->velocity_.linear.x<-0.1)
    {
        this->velocity_.linear.x=-0.1;
    }

    tf2::Quaternion quat(this->robot_pose_->orientation.x, this->robot_pose_->orientation.y, this->robot_pose_->orientation.z, this->robot_pose_->orientation.w);
    tf2::Matrix3x3 m(quat);  
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  
    std::cout<<"Yaw : "<<yaw<<", "<<"atan2(force_total.y, force_total.x) : "<<atan2(force_total.y, force_total.x)<<std::endl;
    this->velocity_.angular.z = 0.4*(atan2(force_total.y, force_total.x) - yaw);

    return this->velocity_;
}

geometry_msgs::msg::Point PotentialField::AttractiveField(const double att_gain, const geometry_msgs::msg::Point::SharedPtr target, const geometry_msgs::msg::Pose::SharedPtr robot_pose){
    geometry_msgs::msg::Point force_att = geometry_msgs::msg::Point();
    force_att.x = -att_gain*(robot_pose->position.x - target->x);
    force_att.y = -att_gain*(robot_pose->position.y - target->y);

    return force_att;
}

geometry_msgs::msg::Point PotentialField::RepulsiveField(const double rep_gain, const double obs_range, const std::vector<geometry_msgs::msg::Point> obstacles, const geometry_msgs::msg::Pose::SharedPtr robot_pose){
    geometry_msgs::msg::Point force_rep = geometry_msgs::msg::Point();
    for(auto &obstacle : obstacles)
    {
        double distance = this->EuclideanDistance(robot_pose->position.x, robot_pose->position.y, obstacle.x, obstacle.y);
        if(distance < obs_range)
        {
            std::cout<<"장애물 발견"<<std::endl;
            force_rep.x += rep_gain * (1/distance - 1/obs_range) * pow(distance,-2) * (robot_pose->position.x - obstacle.x) / distance;
            force_rep.y += rep_gain * (1/distance - 1/obs_range) * pow(distance,-2) * (robot_pose->position.y - obstacle.y) / distance;
        }
    }
    return force_rep;
}

geometry_msgs::msg::Point PotentialField::TotalForce(){

}

double PotentialField::EuclideanDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}