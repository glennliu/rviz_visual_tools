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
        menuLayout->addWidget(btn_mapping);
        menuLayout->addWidget(btn_teach);
        menuLayout->addWidget(btn_optimize);
        menuLayout->addWidget(btn_repeat);

        // Verticle layout
//        auto *layout = new QVBoxLayout;
        mainLayout->addLayout(menuLayout);
        setLayout(mainLayout);

        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_optimize->setEnabled(true);
        btn_repeat->setEnabled(true);


    }

    void RvizVisualToolsGui::moveMapping() {
//  remote_reciever_.publishNext();


    }

    void RvizVisualToolsGui::moveTeach() {
//  remote_reciever_.publishContinue();

        //
        btn_mapping->setDisabled(true);
        btn_optimize->setDisabled(true);
        btn_repeat->setDisabled(true);


        btn_teach_load_path = new QPushButton(this);
        btn_teach_load_path->setText("LoadPath");
        connect(btn_teach_load_path, SIGNAL(clicked()), this, SLOT(moveTeachStart()));

        btn_teach_handheld = new QPushButton(this);
        btn_teach_handheld->setText("Handheld");
        connect(btn_teach_handheld, SIGNAL(clicked()), this, SLOT(moveTeachFinish()));

        btn_teach_joystick = new QPushButton(this);
        btn_teach_joystick->setText("Joystick");
        connect(btn_teach_joystick, SIGNAL(clicked()), this, SLOT(moveTeachReset()));

        btn_teach_reset = new QPushButton(this);
        btn_teach_reset->setText("Reset");
        connect(btn_teach_reset, SIGNAL(clicked()), this, SLOT(moveTeachReset()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));

        // Horizontal Layout2: Teach
        auto *teachLayout = new QHBoxLayout;
        teachLayout->addWidget(btn_teach_load_path);
        teachLayout->addWidget(btn_teach_handheld);
        teachLayout->addWidget(btn_teach_joystick);
        teachLayout->addWidget(btn_teach_reset);
        teachLayout->addWidget(btn_back2main);
        mainLayout->addLayout(teachLayout);
        setLayout(mainLayout);

        btn_teach_load_path->setEnabled(true);
        btn_teach_handheld->setEnabled(true);
        btn_teach_joystick->setEnabled(true);
        btn_teach_reset->setEnabled(true);
        btn_back2main->setEnabled(true);
    }

    void RvizVisualToolsGui::moveOptimize(){
//  remote_reciever_.publishBreak();
    }

    void RvizVisualToolsGui::moveRepeat() {
//  remote_reciever_.publishStop();
    }

    void RvizVisualToolsGui::moveMain() {
        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_optimize->setEnabled(true);
        btn_repeat->setEnabled(true);
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
