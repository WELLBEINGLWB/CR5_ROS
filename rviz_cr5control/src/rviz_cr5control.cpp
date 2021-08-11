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

#include "rviz_cr5control.h"
#include "ui_control_menu.h"

namespace rviz_cr5control
{
CR5Control::CR5Control(QWidget* parent)
    : rviz::Panel(parent)
    , ui(new Ui::ControlMenu)
    , enable_robot_topic_("/CR5Robot/srv/EnableRobot")
    , disable_robot_topic_("/CR5Robot/srv/DisableRobot")
    , robot_status_topic_("/CR5Robot/cmd/RobotStatus")
{
    ui->setupUi(this);

    ui->is_robot_enable->setText("Disable");
    ui->is_robot_connected->setText("Disconnect");
    ui->enable_robot_topic->setText(enable_robot_topic_);
    ui->disable_robot_topic->setText(disable_robot_topic_);
    ui->robot_status_topic->setText(robot_status_topic_);

    enable_robot_client_ =
        nh_.serviceClient<cr5_bringup::EnableRobot>(ui->enable_robot_topic->text().toStdString(), 100);
    disable_robot_client_ =
        nh_.serviceClient<cr5_bringup::DisableRobot>(ui->disable_robot_topic->text().toStdString(), 100);
    //    disable_robot_pub_ = nh_.advertise<cr5_bringup::MovJ>(ui->disable_robot_topic->text().toStdString(), 100);
    //    robot_status_sub_ = nh_.advertise<cr5_bringup::MovJ>(ui->robot_status_topic->text().toStdString(), 100);

    QObject::connect(ui->enable_robot_btn, &QPushButton::clicked, this, &CR5Control::enableRobot);
    QObject::connect(ui->disable_robot_btn, &QPushButton::clicked, this, &CR5Control::disableRobot);
    QObject::connect(ui->enable_robot_topic, &QLineEdit::editingFinished, this,
                     &CR5Control::enableRobotTopicEditFinished);
    QObject::connect(ui->disable_robot_topic, &QLineEdit::editingFinished, this,
                     &CR5Control::disableRobotTopicEditFinished);
    QObject::connect(ui->robot_status_topic, &QLineEdit::editingFinished, this,
                     &CR5Control::robotStatusTopicEditFinished);
}

void CR5Control::load(const Config& config)
{
    QString str;
    if (config.mapGetString(ENABLE_ROBOT_TOPIC_KEY, &str))
    {
        enable_robot_topic_ = str;
    }

    if (config.mapGetString(DISABLE_ROBOT_TOPIC_KEY, &str))
    {
        disable_robot_topic_ = str;
    }

    if (config.mapGetString(ROBOT_STATUS_TOPIC_KEY, &str))
    {
        robot_status_topic_ = str;
    }
}

void CR5Control::save(Config config) const
{
    config.mapSetValue(ENABLE_ROBOT_TOPIC_KEY, ui->enable_robot_topic->text());
    config.mapSetValue(DISABLE_ROBOT_TOPIC_KEY, ui->disable_robot_topic->text());
    config.mapSetValue(ROBOT_STATUS_TOPIC_KEY, ui->robot_status_topic->text());
}

void CR5Control::enableRobot()
{
    cr5_bringup::EnableRobot srv;
    if (enable_robot_client_.call(srv))
    {
        ROS_INFO("enableRobot %d", srv.response.res);
    }
    else
    {
        ROS_ERROR("enableRobot failed");
    }
}

void CR5Control::disableRobot()
{
    cr5_bringup::DisableRobot srv;
    if (disable_robot_client_.call(srv))
    {
        ROS_INFO("disableRobot %d", srv.response.res);
    }
    else
    {
        ROS_ERROR("disableRobot failed");
    }
}

void CR5Control::enableRobotTopicEditFinished()
{
    if (enable_robot_topic_ != ui->enable_robot_topic->text())
    {
        ROS_INFO("enableRobotTopicEditFinished");
    }
}

void CR5Control::disableRobotTopicEditFinished()
{
    if (disable_robot_topic_ != ui->enable_robot_topic->text())
    {
        ROS_INFO("disableRobotTopicEditFinished");
    }
}

void CR5Control::robotStatusTopicEditFinished()
{
    if (robot_status_topic_ != ui->enable_robot_topic->text())
    {
        ROS_INFO("robotStatusTopicEditFinished");
    }
}
}    // namespace rviz_cr5control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cr5control::CR5Control, rviz::Panel)
// END_TUTORIAL