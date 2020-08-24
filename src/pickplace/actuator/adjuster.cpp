#include "adjuster.h"

adjuster::adjuster()
{
    velocity = 1.0;
    accelerated = 1.0;
}

adjuster::~adjuster()
{
}

int adjuster::setMoveGroupName(std::string name)
{
    move_group_name = name;
    _moveGroup = new moveit::planning_interface::MoveGroupInterface("arm");
    return 0;
}

int adjuster::setVelocityAcclerated(double v, double a)
{
    velocity = v;
    accelerated = a;
    return 0;
}

int adjuster::adjustmentTrajecotry(std::vector<moveit_msgs::RobotTrajectory>& traject)
{
    std::vector<moveit_msgs::RobotTrajectory>().swap(targetTraject);
    targetTraject.resize(traject.size());
    _moveGroup->setStartStateToCurrentState();
    robot_trajectory::RobotTrajectory rt(_moveGroup->getCurrentState()->getRobotModel(), _moveGroup->getName());
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    std::vector<moveit_msgs::RobotTrajectory> tras;
    tras.resize(traject.size());
    for(int i=0; i<traject.size(); i++)
    {
        moveit_msgs::RobotTrajectory tra;
        tra = adjustmentPoints(traject[i]);
        rt.setRobotTrajectoryMsg(*(_moveGroup->getCurrentState()), tra);
        iptp.computeTimeStamps(rt, velocity, accelerated);
        rt.getRobotTrajectoryMsg(targetTraject[i]);
    }
    return 0;
}

moveit_msgs::RobotTrajectory adjuster::adjustmentPoints(moveit_msgs::RobotTrajectory& traject)
{
    moveit_msgs::RobotTrajectory tra;
    tra.joint_trajectory.joint_names = traject.joint_trajectory.joint_names;
    tra.joint_trajectory.header.frame_id = traject.joint_trajectory.header.frame_id;
    for(int i=0; i<traject.joint_trajectory.points.size(); i++)
    {
        if(i%4 != 0 && i != traject.joint_trajectory.points.size() - 1) 
            continue;
        tra.joint_trajectory.points.push_back(traject.joint_trajectory.points[i]);
    }
    return tra;
}

int adjuster::getTrajectory(std::vector<moveit_msgs::RobotTrajectory>& tra)
{
    tra = targetTraject;
    return 0;
}
