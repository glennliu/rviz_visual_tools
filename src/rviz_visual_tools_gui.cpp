/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>

#include <QGroupBox>
//#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
//#include <QVBoxLayout>
#include <QtGui/QPalette>
#include <QtWidgets/QtWidgets>

#include "rviz_visual_tools_gui.h"

namespace rviz_visual_tools {
//    auto *layout = new QVBoxLayout;

    RvizVisualToolsGui::RvizVisualToolsGui(QWidget *parent) : rviz::Panel(parent) {
        //*********** Init Buttons *************//
        // Create a push button
        btn_mapping = new QPushButton(this);
        btn_mapping->setText("Mapping");
        connect(btn_mapping, SIGNAL(clicked()), this, SLOT(moveMapping()));

        // Create a push button
        btn_teach = new QPushButton(this);
        btn_teach->setText("Teach");
        connect(btn_teach, SIGNAL(clicked()), this, SLOT(moveTeach()));

        // Create a push button
        btn_optimize = new QPushButton(this);
        btn_optimize->setText("Optimize");
        connect(btn_optimize, SIGNAL(clicked()), this, SLOT(moveOptimize()));

        // Create a push button
        btn_repeat = new QPushButton(this);
        btn_repeat->setText("Repeat");
        connect(btn_repeat, SIGNAL(clicked()), this, SLOT(moveRepeat()));

        // Horizontal Layout1: Menu
        menuLayout = new QHBoxLayout;
        menuLayout->addWidget(btn_mapping);
        menuLayout->addWidget(btn_teach);
        menuLayout->addWidget(btn_optimize);
        menuLayout->addWidget(btn_repeat);

        // Verticle layout
//        auto *layout = new QVBoxLayout;
        mainLayout = new QVBoxLayout;
        mainLayout->addLayout(menuLayout);
        setLayout(mainLayout);

        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_optimize->setEnabled(true);
        btn_repeat->setEnabled(true);

//        delete(menuLayout);



    }

    void RvizVisualToolsGui::moveMapping() {
//  remote_receiver.publishNext();

    }

    void RvizVisualToolsGui::moveTeach() {
        remote_receiver.EnterTeach();

        //
        btn_mapping->setDisabled(true);
        btn_optimize->setDisabled(true);
        btn_repeat->setDisabled(true);

        btn_teach_load_path = new QPushButton(this);
        btn_teach_load_path->setText("LoadPath");
        connect(btn_teach_load_path, SIGNAL(clicked()), this, SLOT(moveTeachLoadPath()));

        btn_teach_joyinit = new QPushButton(this);
        btn_teach_joyinit->setText("JoyInit");
        connect(btn_teach_joyinit, SIGNAL(clicked()), this, SLOT(moveTeachJoyInit()));

        btn_teach_joyfinish = new QPushButton(this);
        btn_teach_joyfinish->setText("JoyFinish");
        connect(btn_teach_joyfinish, SIGNAL(clicked()), this, SLOT(moveTeachJoyFinish()));

        btn_teach_reset = new QPushButton(this);
        btn_teach_reset->setText("Reset");
        connect(btn_teach_reset, SIGNAL(clicked()), this, SLOT(moveTeachJoystick()));


        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));



        // Horizontal Layout2: Teach
//        auto *teachLayout = new QHBoxLayout;
        teachLayout = new QHBoxLayout;
        teachLayout->addWidget(btn_teach_load_path);
        teachLayout->addWidget(btn_teach_joyinit);
        teachLayout->addWidget(btn_teach_joyfinish);
        teachLayout->addWidget(btn_teach_reset);
        teachLayout->addWidget(btn_back2main);
        mainLayout->addLayout(teachLayout);
        setLayout(mainLayout);

        btn_teach_load_path->setEnabled(true);
        btn_teach_joyinit->setEnabled(true);
        btn_teach_joyfinish->setEnabled(true);
        btn_teach_reset->setEnabled(true);
        btn_back2main->setEnabled(true);
    }

    void RvizVisualToolsGui::moveOptimize(){
//  remote_receiver.publishBreak();
    }

    void RvizVisualToolsGui::moveRepeat() {
        remote_receiver.EnterRepeat();

        btn_repeat_go = new QPushButton(this);
        btn_repeat_go->setText("Go");
        connect(btn_repeat_go, SIGNAL(clicked()), this, SLOT(moveRepeatGo()));

        btn_repeat_land = new QPushButton(this);
        btn_repeat_land->setText("Land");
        connect(btn_repeat_land, SIGNAL(clicked()), this, SLOT(moveRepeatLand()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));

        repeatLayout = new QHBoxLayout;
        repeatLayout->addWidget(btn_repeat_go);
        repeatLayout->addWidget(btn_repeat_land);
        repeatLayout->addWidget(btn_back2main);
        mainLayout->addLayout(repeatLayout);
        setLayout(mainLayout);
    }

    void RvizVisualToolsGui::moveRepeatGo() {
        remote_receiver.RepeatGo();
    }

    void RvizVisualToolsGui::moveRepeatLand(){
        remote_receiver.RepeatLand();
    }

    void RvizVisualToolsGui::moveMain() {

        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_optimize->setEnabled(true);
        btn_repeat->setEnabled(true);

        std_msgs::String gui_state_;
        gui_state_ = remote_receiver.check_gui_state();
        ROS_INFO("%f",gui_state_.data);

        if (gui_state_.data == "TEACH_INIT" || gui_state_.data == "TEACH_FINISHED"){

            // remove teach layout
            delete (btn_teach_load_path);
            delete (btn_teach_joyinit);
            delete (btn_teach_joyfinish);
            delete (btn_teach_reset);
            delete (teachLayout);
        }
        else if(gui_state_.data == "REPEAT_INIT" || gui_state_.data == "REPEAT_LAND")
        {
            // remove repeat layout
            delete(btn_repeat_go);
            delete(btn_repeat_land);
            delete(repeatLayout);
        }


        //
        delete(btn_back2main);
        setLayout(mainLayout);
        remote_receiver.back2main();


    }

    void RvizVisualToolsGui::moveTeachJoyInit(){
        remote_receiver.TeachJoyInit();
    }

    void RvizVisualToolsGui::moveTeachJoyFinish() {
        remote_receiver.TeachJoyFinish();
    }

    void RvizVisualToolsGui::save(rviz::Config config) const {
        rviz::Panel::save(config);
    }

    void RvizVisualToolsGui::load(const rviz::Config &config) {
        rviz::Panel::load(config);
    }
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsGui, rviz::Panel)
