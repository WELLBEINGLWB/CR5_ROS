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
#include <cr5_bringup/cr5_robot.h>

CR5Robot::CR5Robot(ros::NodeHandle &nh, std::string name)
        : ActionServer<FollowJointTrajectoryAction>(nh, name, false), control_nh_(nh) {
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

CR5Robot::~CR5Robot() {
    ROS_INFO("~CR5Robot");
}

void CR5Robot::init() {
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    subscriber_tbl_.push_back(control_nh_.subscribe("msg/MovJ", 100, &CR5Robot::movJ, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/MovL", 100, &CR5Robot::movL, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/RelMovJ", 100, &CR5Robot::relMovJ, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/RelMovL", 100, &CR5Robot::relMovL, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/ServoJ", 100, &CR5Robot::servoJ, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/ServoP", 100, &CR5Robot::servoP, this));
    subscriber_tbl_.push_back(control_nh_.subscribe("msg/JointMovJ", 100, &CR5Robot::jointMovJ, this));

    server_tbl_.push_back(control_nh_.advertiseService("srv/ClearError", &CR5Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/DisableRobot", &CR5Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("srv/EnableRobot", &CR5Robot::enableRobot, this));

    registerGoalCallback(boost::bind(&CR5Robot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CR5Robot::cancelHandle, this, _1));
    start();
}

void CR5Robot::feedbackHandle(const ros::TimerEvent &tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle) {
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++) {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const ros::TimerEvent &tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle) {
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size()) {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++) {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        commander_->servoJ(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        index_++;
    } else {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL)) {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle) {
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++) {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(0.30), boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle) {
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CR5Robot::getJointState(double *point) {
    commander_->getCurrentJointStatus(point);
}

void CR5Robot::movJ(const cr5_bringup::MovJConstPtr &point) {
    commander_->movJ(point->x, point->y, point->z, point->a, point->b, point->c);
}

void CR5Robot::movL(const cr5_bringup::MovLConstPtr &point) {
    commander_->movL(point->x, point->y, point->z, point->a, point->b, point->c);
}

void CR5Robot::servoJ(const cr5_bringup::ServoJConstPtr &points) {
    commander_->servoJ(points->j1, points->j2, points->j3, points->j4, points->j5, points->j6);
}

void CR5Robot::servoP(const cr5_bringup::ServoPConstPtr &point) {
    commander_->servoP(point->x, point->y, point->z, point->a, point->b, point->c);
}

void CR5Robot::relMovJ(const cr5_bringup::RelMovJConstPtr &point) {
    commander_->relMovJ(point->offset1, point->offset2, point->offset3, point->offset4, point->offset5, point->offset6);
}

void CR5Robot::relMovL(const cr5_bringup::RelMovLConstPtr &point) {
    commander_->relMovL(point->x, point->y, point->z);
}

void CR5Robot::jointMovJ(const cr5_bringup::JointMovJConstPtr &point) {
    commander_->jointMovJ(point->j1, point->j2, point->j3, point->j4, point->j5, point->j6);
}

bool CR5Robot::clearError(cr5_bringup::ClearError::Request &request, cr5_bringup::ClearError::Response &response) {
    try {
        commander_->clearError();
        response.res = 0;
        return true;
    }
    catch (const std::exception &err) {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::enableRobot(cr5_bringup::EnableRobot::Request &request, cr5_bringup::EnableRobot::Response &response) {
    try {
        commander_->enableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception &err) {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool
CR5Robot::disableRobot(cr5_bringup::DisableRobot::Request &request, cr5_bringup::DisableRobot::Response &response) {
    try {
        commander_->disableRobot();
        response.res = 0;
        return true;
    }
    catch (const std::exception &err) {
        commander_->clearError();
        response.res = -1;
        return false;
    }
}

bool CR5Robot::isEnable() const {
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const {
    return commander_->isConnected();
}

void CR5Robot::getToolVectorActual(double *val) {
    commander_->getToolVectorActual(val);
}
