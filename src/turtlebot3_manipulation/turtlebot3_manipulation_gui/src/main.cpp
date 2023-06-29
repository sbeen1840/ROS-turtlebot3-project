

#include <QtGui>
#include <QApplication>
#include "../include/turtlebot3_manipulation_gui/main_window.hpp"
#include "ros/ros.h"
#include "std_msgs/Int8.h"

int8_t action = 3;

void action_callback(const std_msgs::Int8::ConstPtr& msg)
{
    action = msg->data;
    ROS_INFO("I heard: [%d]", msg->data);
    ROS_INFO("action flag =  [%d]", action);
}

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    turtlebot3_manipulation_gui::MainWindow w(argc,argv);
    w.on_btn_timer_start_clicked();
    ros::init(argc, argv, "turtlebot3_manipulation_gui");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/action", 1, action_callback);
    ros::Publisher pub = nh.advertise<std_msgs::Int8>("/grasp",1);
    ros::Rate r(10);
    std_msgs::Int8 grasp_msg;
    grasp_msg.data = 0;

    while(ros::ok())
    {

        if (action == 1)
        {
            ROS_INFO("[동작1] - 1/4 - GRIPPER OPEN");
            w.on_btn_gripper_open_clicked();
            ros::Duration(3).sleep();

            ROS_INFO("[동작1] - 2/4 - PICK");
            w.manipulator_pick();
            ros::Duration(8.6).sleep();//6.5

            ROS_INFO("[동작1] - 3/4 - GRIPPER CLOSE");
            w.on_btn_gripper_close_clicked();
            ros::Duration(2.5).sleep();//6.5

            ROS_INFO("[동작1] - 4/4 - HOME");
            w.on_btn_home_pose_clicked();
            ros::Duration(7.2).sleep();//7.2

            action = 0;
            grasp_msg.data = 1;
            pub.publish(grasp_msg);
        }


        if (action == 2)
        {
            ROS_INFO("[동작2] - 1/6 - PUT");
            w.manipulator_put();
            ros::Duration(8.6).sleep();


            ROS_INFO("action2 - [파랑] - 2/6 - GRIPPER OPEN");
            w.on_btn_gripper_open_clicked();
            ros::Duration(2.5).sleep();

            ROS_INFO("action2 - [파랑] - 3/6 - GRIPPER CLOSE");
            w.on_btn_gripper_close_clicked();
            ros::Duration(2.5).sleep();


            ROS_INFO("action2 - [파랑] - 4/6 - RIGHT");
            w.manipulator_right();
            ros::Duration(5.2).sleep();

            ROS_INFO("action2 - [파랑] - 5/6 - LEFT");
            w.manipulator_left();
            ros::Duration(5.2).sleep();


            ROS_INFO("action2 - [파랑] - 6/6 - HOME");
            w.on_btn_home_pose_clicked();
            ros::Duration(7.2).sleep();

            action = 0;
            grasp_msg.data = 2;
            pub.publish(grasp_msg);
        }

        if (action == 3)
        {
            ROS_INFO("action3 - [셋업] - 1/1 - HOME");
            w.on_btn_init_pose_clicked();
            ros::Duration(5).sleep();

            action = 0;
            grasp_msg.data = 3;
            pub.publish(grasp_msg);
        }


        ros::spinOnce();
    }

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
	return result;

}