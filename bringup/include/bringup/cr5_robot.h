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
#include <bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <bringup/MovJ.h>
#include <bringup/MovL.h>
#include <bringup/ServoJ.h>
#include <bringup/MoveJog.h>
#include <bringup/ServoP.h>
#include <bringup/RelMovJ.h>
#include <bringup/RelMovL.h>
#include <bringup/JointMovJ.h>
#include <bringup/ResetRobot.h>
#include <bringup/RobotStatus.h>

#include <bringup/ClearError.h>
#include <bringup/EnableRobot.h>
#include <bringup/DisableRobot.h>

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
    bool movJ(bringup::MovJ::Request& request, bringup::MovJ::Response& response);
    bool movL(bringup::MovL::Request& request, bringup::MovL::Response& response);
    bool servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response);
    bool servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response);
    bool relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response);
    bool relMovL(bringup::RelMovL::Request& request, bringup::RelMovL::Response& response);
    bool moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response);
    bool jointMovJ(bringup::JointMovJ::Request& request, bringup::JointMovJ::Response& response);

    bool resetRobot(bringup::ResetRobot::Request& request, bringup::ResetRobot::Response& response);
    bool clearError(bringup::ClearError::Request& request, bringup::ClearError::Response& response);
    bool enableRobot(bringup::EnableRobot::Request& request, bringup::EnableRobot::Response& response);
    bool disableRobot(bringup::DisableRobot::Request& request, bringup::DisableRobot::Response& response);

private:
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
};
