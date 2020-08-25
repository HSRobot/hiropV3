#pragma once
#include <string>
#include <vector>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class adjuster
{
private:
    double velocity;
    double accelerated;
    std::string move_group_name;
    moveit::planning_interface::MoveGroupInterface* _moveGroup;
    std::vector<moveit_msgs::RobotTrajectory> targetTraject;
    moveit_msgs::RobotTrajectory adjustmentPoints(moveit_msgs::RobotTrajectory& traject);

public:
    adjuster();
    ~adjuster();

    int setMoveGroupName(std::string name);
    int setVelocityAcclerated(double v, double a);
    int adjustmentTrajecotry(std::vector<moveit_msgs::RobotTrajectory>& tra);
    int getTrajectory(std::vector<moveit_msgs::RobotTrajectory>& tra);
};


