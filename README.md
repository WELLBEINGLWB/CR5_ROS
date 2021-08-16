# <center>CR5Robot</center>

# 源码编译
## ubuntu16.04

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/CR5_ROS.git -b kinetic-devel

cd $HOME/catkin_ws

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

## ubuntu18.04

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/CR5_ROS.git -b melodic-devel

cd $HOME/catkin_ws

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

# 示例演示

## 在仿真环境下使用

1. ## rviz 显示

    ```
    roslaunch cr5_description display.launch
    ```

    可通过 joint_state_publisher_gui 调节各关节的角度，在 rviz 上看到其显示效果

    ![rviz显示](./rviz.jpg)


2. ## moveit 控制
    * 使用如下命令启动 moveit
    ```
    roslaunch cr5_moveit demo.launch
    ```
    * 鼠标将关节拖到任意的角度，点击 "Plan and Execute" 即可看到运行效果

    ![moveit显示](./moveit.gif)


## 控制真实机械臂

* **使用如下命令连接机械臂, robot_ip 为实际机械臂所对应的IP地址**
    ```
    roslaunch cr5_bringup cr5_bringup.launch robot_ip:=192.168.5.1
    ```

* **使用如下命令启动 Moveit**
    ```
    roslaunch cr5_moveit cr5_moveit.launch
    ```

* **在 rviz 中添加 CR5Control 插件控制面板，用来 使能机械臂**
    1. 点击 rviz 工具栏上的 Panels --> "Add New Panel"
    2. 选中 CR5Control, 再点击 “OK”
    3. 点击 “EnableRobot” 使机械臂
    4. 当状态样上显示 “Enable” “Connected” 表示机械臂已连接和使能，即可通过 Moveit 控制机械臂

    ![CR5Control](./cr5control.jpg)


# 自定义功能开发

    cr5_bringup 中定义了 msg 和 srv，可户通过这些底层 msg 和 srv 完成对机械臂的控制