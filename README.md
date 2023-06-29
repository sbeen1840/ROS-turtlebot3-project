# ROS-turtlebot3-project
2023 지능로봇실습, 터틀봇3 파이널 프로젝트


https://studio.youtube.com/video/QZNm-e-1x_o/edit

https://www.youtube.com/watch?v=5jZof51nL0c

https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/0ca45621-06e3-47cd-91da-28485cdde2d1


https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/e8f51b08-1360-4fb2-b8a7-659ed6a59768


---
# Map

<p align="center">
	<img src="https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/17a6d7f4-06b2-4953-bc53-22d3c93cba10" alt="img" width="50%" height="50%"/>
</p>
The diagram represents the path that the TurtleBot should follow in the project, along with its starting point and destination.

---

# Missions
<p align="center">
	<img src="https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/f0ac8ebd-8681-409d-9ac9-a9a7ae2df192" alt="img" width="50%" height="50%"/>
</p>
The diagram depicts a list of missions that the TurtleBot should perform while traversing the path.

---
# Algorithm
<p align="center">
	<img src="https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/9a54c633-23fe-4d02-87e3-9003c2995310" alt="img" width="50%" height="50%"/>
</p>
Take a look at this simple diagram illustrating three important nodes and the topics they exchange.

---

# How to run
<p align="center">
	<img src="https://github.com/sbeen1840/ROS-turtlebot3-project/assets/108644811/820b3ba7-b088-47c2-a8cc-9f3c1cdc07f3" alt="img"  height="50%"/>
</p>


⚠️⚠️ If you don't follow the correct execution order of the nodes may lead to errors. ⚠️⚠️

---

# Commands
### Remote PC

```
roscore
```
```
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
```
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
```
roslaunch maze_escape maze_escape.launch --screen
```
```
rosrun sawyer_catching_ball_2 detect_color
```
```
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
### Raspberry Pi

```
ssh ubuntu@192.168.54.252 // IP is example

roslaunch raspicam_node camerav1_1280x720.launch
```
```
ssh ubuntu@192.168.54.252 // IP is example

roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
