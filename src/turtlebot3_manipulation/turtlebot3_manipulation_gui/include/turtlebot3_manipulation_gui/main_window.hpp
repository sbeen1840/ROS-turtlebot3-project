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

#ifndef TURTLEBOT3_MANIPULATION_GUI_MAIN_WINDOW_H
#define TURTLEBOT3_MANIPULATION_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Int8.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace turtlebot3_manipulation_gui { //this closure is needed at the end of this file

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();
  void writeLog(QString str);

public Q_SLOTS:
  void on_btn_timer_start_clicked(void);

  void on_btn_init_pose_clicked(void);
  void on_btn_home_pose_clicked(void);

  void on_btn_gripper_open_clicked(void);
  void on_btn_gripper_close_clicked(void);

  void manipulator_pick(void);
  void manipulator_put(void);

  void manipulator_right(void);
  void manipulator_left(void);


private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer *timer;
};

}  // namespace turtlebot3_manipulation_gui

#endif