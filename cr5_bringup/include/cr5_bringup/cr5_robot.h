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
#include <cr5_bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <cr5_bringup/MovJ.h>
#include <cr5_bringup/MovL.h>
#include <cr5_bringup/ServoJ.h>
#include <cr5_bringup/ServoP.h>
#include <cr5_bringup/RelMovJ.h>
#include <cr5_bringup/RelMovL.h>
#include <cr5_bringup/JointMovJ.h>

#include <cr5_bringup/ClearError.h>
#include <cr5_bringup/EnableRobot.h>
#include <cr5_bringup/DisableRobot.h>


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
    std::vector<ros::Subscriber> subscriber_tbl_;
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
    ~CR5Robot();

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

protected:

    void movJ(const cr5_bringup::MovJConstPtr & point);
    void movL(const cr5_bringup::MovLConstPtr & point);
    void servoJ(const cr5_bringup::ServoJConstPtr& points);
    void servoP(const cr5_bringup::ServoPConstPtr& point);
    void relMovJ(const cr5_bringup::RelMovJConstPtr & point);
    void relMovL(const cr5_bringup::RelMovLConstPtr & point);
    void jointMovJ(const cr5_bringup::JointMovJConstPtr & point);

    bool clearError(cr5_bringup::ClearError::Request& request, cr5_bringup::ClearError::Response& response);
    bool enableRobot(cr5_bringup::EnableRobot::Request& request, cr5_bringup::EnableRobot::Response& response);
    bool disableRobot(cr5_bringup::DisableRobot::Request& request, cr5_bringup::DisableRobot::Response& response);

private:
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm,
                    actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
};
