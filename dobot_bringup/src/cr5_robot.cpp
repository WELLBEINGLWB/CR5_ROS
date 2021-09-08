/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>

CR5Robot::CR5Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false), goal_{}, control_nh_(nh)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

CR5Robot::~CR5Robot()
{
    ROS_INFO("~CR5Robot");
}

void CR5Robot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovJ", &CR5Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MoveJog", &CR5Robot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovL", &CR5Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovJ", &CR5Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovL", &CR5Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoJ", &CR5Robot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoP", &CR5Robot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/JointMovJ", &CR5Robot::jointMovJ, this));

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ClearError", &CR5Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ResetRobot", &CR5Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DisableRobot", &CR5Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/EnableRobot", &CR5Robot::enableRobot, this));

    registerGoalCallback(boost::bind(&CR5Robot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CR5Robot::cancelHandle, this, _1));
    start();
}

void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++)
    {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size())
    {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++)
        {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        commander_->servoJ(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        index_++;
    }
    else
    {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL))
        {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++)
    {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_), boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CR5Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CR5Robot::clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response)
{
    try
    {
        commander_->clearError();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response)
{
    try
    {
        commander_->enableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(dobot_bringup::DisableRobot::Request& request, dobot_bringup::DisableRobot::Response& response)
{
    try
    {
        commander_->disableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::isEnable() const
{
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const
{
    return commander_->isConnected();
}

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

bool CR5Robot::movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response)
{
    try
    {
        commander_->movJ(request.x, request.y, request.z, request.z, request.b, request.c);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response)
{
    try
    {
        commander_->movL(request.x, request.y, request.z, request.z, request.b, request.c);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response)
{
    try
    {
        commander_->servoJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response)
{
    try
    {
        commander_->servoP(request.x, request.y, request.z, request.a, request.b, request.c);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response)
{
    try
    {
        commander_->relMovJ(request.offset1, request.offset2, request.offset3, request.offset4, request.offset5,
                            request.offset6);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response)
{
    try
    {
        commander_->relMovL(request.x, request.y, request.z);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response)
{
    try
    {
        commander_->jointMovJ(request.j1, request.j2, request.j3, request.j4, request.j5, request.j6);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

bool CR5Robot::moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response)
{
    try
    {
        commander_->moveJog(request.axisID);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = 0;
        return false;
    }
}

bool CR5Robot::resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response)
{
    try
    {
        commander_->resetRobot();
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
