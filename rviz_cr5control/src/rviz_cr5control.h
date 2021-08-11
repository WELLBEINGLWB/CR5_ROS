/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/09
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <cr5_bringup/EnableRobot.h>
#include <cr5_bringup/DisableRobot.h>

using namespace rviz;

QT_BEGIN_NAMESPACE
namespace Ui { class ControlMenu; }
QT_END_NAMESPACE

namespace rviz_cr5control
{
class CR5Control : public rviz::Panel
{
    Q_OBJECT

public:
    static constexpr const char* ENABLE_ROBOT_TOPIC_KEY = "EnableRobotTopicKey";
    static constexpr const char* DISABLE_ROBOT_TOPIC_KEY = "DisableRobotTopicKey";
    static constexpr const char* ROBOT_STATUS_TOPIC_KEY = "RobotStatusTopicKey";

private:
    Ui::ControlMenu *ui;
    ros::NodeHandle nh_;
    QString enable_robot_topic_;
    QString disable_robot_topic_;
    QString robot_status_topic_;
    ros::ServiceClient enable_robot_client_;
    ros::ServiceClient disable_robot_client_;
//    ros::ServiceClient enable_robot_client_;

public Q_SLOTS:
    void enableRobot();
    void disableRobot();
    void enableRobotTopicEditFinished();
    void disableRobotTopicEditFinished();
    void robotStatusTopicEditFinished();

public:
    CR5Control(QWidget* parent = nullptr);

    /** @brief Override to load configuration data.  This version loads the name of the panel. */
    virtual void load( const Config& config ) override;

    /** @brief Override to save configuration data.  This version saves the name and class ID of the panel. */
    virtual void save( Config config ) const override;
};
}