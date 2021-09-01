# <center>CR5Robot</center>

Chinese version of the README -> please [click here](./README-CN.md)

# Building
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

# Example Demonstration

## Apply in simulation environment

1. ## rviz display

    ```
    roslaunch cr5_description display.launch
    ```

    User can adjust the angle of each joint by joint_state_publisher_gui, and see the result from rviz

    ![rviz display](./rviz.jpg)


2. ## moveit control
    * Active moveit by the following commands
    ```
    roslaunch cr5_moveit demo.launch
    ```
    * Drag the joint to any direction, then click "Plan and Excute" to see the result

    ![moveit display](./moveit.gif)


## Controlling real robotic arm

* **Connect the robotic arm with following command, and robot_ip is the IP address that the real arm locates**
    ```
    roslaunch cr5_bringup cr5_bringup.launch robot_ip:=192.168.5.1
    ```

* **Active Moveit with following command**
    ```
    roslaunch cr5_moveit cr5_moveit.launch
    ```

* **Install CR5Control Plugin to enable the robotic arm**
    
    1. Press Panels on the tool bar of rviz --> "Add New Panel"
    2. Choose CR5Control, then press "OK"
    3. Press "EnableRobot" to enable the arm
    4. When "Connected" and "Enable" is displayed on the status bar, it means the robotic arm is connected and enabled, and users can control the robotic arm via Moveit

    ![CR5Control](./cr5control.jpg)


# Custom Function Development

    Msg and scr is defined in cr5_bringup. Users can control the robotic arm via those underlying commands