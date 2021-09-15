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

#include <string>
#include <memory>
#include <ros/ros.h>
#include <dobot_bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <dobot_bringup/MovJ.h>
#include <dobot_bringup/MovL.h>
#include <dobot_bringup/ServoJ.h>
#include <dobot_bringup/MoveJog.h>
#include <dobot_bringup/ServoP.h>
#include <dobot_bringup/RelMovJ.h>
#include <dobot_bringup/RelMovL.h>
#include <dobot_bringup/JointMovJ.h>
#include <dobot_bringup/ResetRobot.h>
#include <dobot_bringup/RobotStatus.h>

#include <dobot_bringup/ClearError.h>
#include <dobot_bringup/EnableRobot.h>
#include <dobot_bringup/SpeedFactor.h>
#include <dobot_bringup/DisableRobot.h>

using namespace actionlib;
using namespace control_msgs;

/**
 * CR5Robot
 */
class CR5Robot : protected ActionServer<FollowJointTrajectoryAction>
{
private:
    double goal_[6];
    uint32_t index_;
    ros::Timer timer_;
    ros::Timer movj_timer_;
    double trajectory_duration_;
    ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<ros::ServiceServer> server_tbl_;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    CR5Robot(ros::NodeHandle& nh, std::string name);

    /**
     * CR5Robot
     */
    ~CR5Robot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response);
    bool movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response);
    bool servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response);
    bool servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response);
    bool relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response);
    bool relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response);
    bool moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response);
    bool jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response);

    bool resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response);
    bool clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response);
    bool enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response);
    bool speedFactor(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
    bool disableRobot(dobot_bringup::DisableRobot::Request& request, dobot_bringup::DisableRobot::Response& response);

private:
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
};
