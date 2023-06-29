
/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

/*****************************************************************************
* Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/turtlebot3_manipulation_gui/main_window.hpp"

namespace turtlebot3_manipulation_gui { //this closure is needed at the end of the file

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode(argc,argv)
{
  ui.setupUi(this);
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  qnode.init();
}

MainWindow::~MainWindow() {}


void MainWindow::on_btn_timer_start_clicked(void)
{
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
  timer->start(100);

  writeLog("QTimer start : 100ms");
  ui.btn_timer_start->setEnabled(false);
  ui.btn_home_pose->setEnabled(true);
  ui.btn_init_pose->setEnabled(true);
  ui.btn_read_joint_angle->setEnabled(true);
  ui.btn_read_kinematic_pose->setEnabled(true);
  ui.btn_send_joint_angle->setEnabled(true);
  ui.btn_send_kinematic_pose->setEnabled(true);
  ui.btn_gripper_close->setEnabled(true);
  ui.btn_gripper_open->setEnabled(true);
  ui.btn_set_gripper->setEnabled(true);
}


void MainWindow::writeLog(QString str)
{
  ui.plainTextEdit_log->moveCursor (QTextCursor::End);
  ui.plainTextEdit_log->appendPlainText(str);
}


// ! manipulator_pick
void MainWindow::manipulator_pick(void)
{
  std::vector<double> joint_angle;
  double path_time = 4.0;

  joint_angle.push_back(0.0);
  joint_angle.push_back(0.5);
  joint_angle.push_back(-0.4);
  joint_angle.push_back(0.8);

  if(!qnode.setJointSpacePath(joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}

// ! manipulator_put
void MainWindow::manipulator_put(void)
{
  std::vector<double> joint_angle;
  double path_time = 4.0;

  joint_angle.push_back(0.0);
  joint_angle.push_back(0.3);
  joint_angle.push_back(-0.1);
  joint_angle.push_back(-0.7);

  if(!qnode.setJointSpacePath(joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}

// ! manipulator_right
void MainWindow::manipulator_right(void)
{
  std::vector<double> joint_angle;
  double path_time = 2.0;

  joint_angle.push_back(-0.5);
  joint_angle.push_back(0.4);
  joint_angle.push_back(-0.1);
  joint_angle.push_back(-0.7);

  if(!qnode.setJointSpacePath(joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}

// ! manipulator_left
void MainWindow::manipulator_left(void)
{
  std::vector<double> joint_angle;
  double path_time = 2.0;

  joint_angle.push_back(0.5);
  joint_angle.push_back(0.4);
  joint_angle.push_back(-0.1);
  joint_angle.push_back(-0.7);

  if(!qnode.setJointSpacePath(joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}

//! on_btn_gripper_open_clicked
void MainWindow::on_btn_gripper_open_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(0.015);

  if(!qnode.setToolControl(joint_angle))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}


//! on_btn_gripper_close_clicked
void MainWindow::on_btn_gripper_close_clicked(void)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(-0.012);

  if(!qnode.setToolControl(joint_angle))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

}

//! init pose
void MainWindow::on_btn_init_pose_clicked(void)
{
  std::vector<double> joint_angle;
  double path_time = 2.0;
  joint_angle.push_back(0.0);
  joint_angle.push_back(0.0);
  joint_angle.push_back(0.0);
  joint_angle.push_back(0.0);

  if(!qnode.setJointSpacePath(joint_angle, path_time))
  {
    writeLog("[ERR!!] Failed to send service");
    return;
  }

  writeLog("Send joint angle to initial pose");
}


//! home pose
void MainWindow::on_btn_home_pose_clicked(void)
{
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_angle.push_back(0.0);
    joint_angle.push_back(-1.0);
    joint_angle.push_back(0.3);
    joint_angle.push_back(0.7);

    if(!qnode.setJointSpacePath(joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
}}  // namespace turtlebot3_manipulation_gui