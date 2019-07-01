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
#include "include/rviz_visual_tools/mav_cmd_enum.h"

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
        btn_repeat = new QPushButton(this);
        btn_repeat->setText("Repeat");
        connect(btn_repeat, SIGNAL(clicked()), this, SLOT(moveRepeat()));

        // Create a push button
        btn_airborne = new QPushButton(this);
        btn_airborne->setText("Airborne");
        connect(btn_airborne, SIGNAL(clicked()), this, SLOT(moveAirborne()));

        // Horizontal Layout1: Menu
        menuLayout = new QHBoxLayout;
        menuLayout->addWidget(btn_mapping);
        menuLayout->addWidget(btn_teach);
        menuLayout->addWidget(btn_repeat);
        menuLayout->addWidget(btn_airborne);

        // Verticle layout
//        auto *layout = new QVBoxLayout;
        mainLayout = new QVBoxLayout;
        mainLayout->addLayout(menuLayout);
        setLayout(mainLayout);

        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_repeat->setEnabled(true);
        btn_airborne->setEnabled(true);
//        delete(menuLayout);
    }

    void RvizVisualToolsGui::moveMapping() {
        remote_receiver.EnterMap();

        btn_teach->setDisabled(true);
        btn_repeat->setDisabled(true);

        btn_map_init = new QPushButton(this);
        btn_map_init->setText("Start");
        connect(btn_map_init,SIGNAL(clicked()),this,SLOT(moveMapStart()));

        btn_map_finish = new QPushButton(this);
        btn_map_finish->setText("Finish");
        connect(btn_map_finish,SIGNAL(clicked()),this,SLOT(moveMapFinished()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));

        mapLayout = new QHBoxLayout;
        mapLayout->addWidget(btn_map_init);
        mapLayout->addWidget(btn_map_finish);
        mapLayout->addWidget(btn_back2main);
        mainLayout->addLayout(mapLayout);
        setLayout(mainLayout);

        btn_map_init->setEnabled(true);
        btn_back2main->setEnabled(true);
        btn_map_finish->setDisabled(true);
    }

    void RvizVisualToolsGui::moveTeach() {
        remote_receiver.EnterTeach();

        //
        btn_mapping->setDisabled(true);
        btn_repeat->setDisabled(true);

        btn_teach_joyinit = new QPushButton(this);
        btn_teach_joyinit->setText("JoyInit");
        connect(btn_teach_joyinit, SIGNAL(clicked()), this, SLOT(moveTeachJoyInit()));

        btn_teach_joyfinish = new QPushButton(this);
        btn_teach_joyfinish->setText("JoyFinish");
        connect(btn_teach_joyfinish, SIGNAL(clicked()), this, SLOT(moveTeachJoyFinish()));

        btn_teach_reset = new QPushButton(this);
        btn_teach_reset->setText("Reset");
        connect(btn_teach_reset, SIGNAL(clicked()), this, SLOT(moveTeachJoyReset()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));


        // Horizontal Layout2: Teach
//        auto *teachLayout = new QHBoxLayout;
        teachLayout = new QHBoxLayout;
        teachLayout->addWidget(btn_teach_joyinit);
        teachLayout->addWidget(btn_teach_joyfinish);
        teachLayout->addWidget(btn_teach_reset);
        teachLayout->addWidget(btn_back2main);
        mainLayout->addLayout(teachLayout);
        setLayout(mainLayout);

        btn_teach_joyinit->setEnabled(true);
        btn_teach_joyfinish->setDisabled(true);
        btn_teach_reset->setDisabled(true);
        btn_back2main->setEnabled(true);
    }



    void RvizVisualToolsGui::moveRepeat() {
        remote_receiver.EnterRepeat();
        btn_mapping->setDisabled(true);
        btn_teach->setDisabled(true);

        btn_repeat_load = new QPushButton(this);
        btn_repeat_load->setText("LoadPath");
        connect(btn_repeat_load, SIGNAL(clicked()), this, SLOT(moveRepeatLoad()));

        btn_repeat_go = new QPushButton(this);
        btn_repeat_go->setText("Go");
        connect(btn_repeat_go, SIGNAL(clicked()), this, SLOT(moveRepeatGo()));

        btn_repeat_land = new QPushButton(this);
        btn_repeat_land->setText("Land");
        connect(btn_repeat_land, SIGNAL(clicked()), this, SLOT(moveRepeatLand()));

        btn_repeat_reset = new QPushButton(this);
        btn_repeat_reset->setText("Reset");
        connect(btn_repeat_reset, SIGNAL(clicked()), this, SLOT(moveRepeatReset()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));

        repeatLayout = new QHBoxLayout;
        repeatLayout->addWidget(btn_repeat_load);
        repeatLayout->addWidget(btn_repeat_go);
        repeatLayout->addWidget(btn_repeat_land);
        repeatLayout->addWidget(btn_repeat_reset);
        repeatLayout->addWidget(btn_back2main);
        mainLayout->addLayout(repeatLayout);
        setLayout(mainLayout);

        btn_repeat_load->setEnabled(true);
        btn_back2main->setEnabled(true);
        btn_repeat_go->setDisabled(true);
        btn_repeat_land->setDisabled(true);
        btn_repeat_reset->setDisabled(true);

    }

    void RvizVisualToolsGui::moveAirborne() {
        remote_receiver.EnterAirborne();

        btn_mapping->setDisabled(true);
        btn_teach->setDisabled(true);
        btn_repeat->setDisabled(true);

        btn_airborne_takeoff = new QPushButton(this);
        btn_airborne_takeoff->setText("Takeoff");
        connect(btn_airborne_takeoff, SIGNAL(clicked()), this, SLOT(moveAirborneTakeoff()));

        btn_airborne_marker = new QPushButton(this);
        btn_airborne_marker->setText("Marker");
        connect(btn_airborne_marker, SIGNAL(clicked()), this, SLOT(moveAirborneMarker()));

        btn_airborne_joy = new QPushButton(this);
        btn_airborne_joy->setText("Joy");
        connect(btn_airborne_joy, SIGNAL(clicked()), this, SLOT(moveAirborneJoy()));

        btn_airborn_finished = new QPushButton(this);
        btn_airborn_finished->setText("Finish");
        connect(btn_airborn_finished, SIGNAL(clicked()), this, SLOT(moveAirborneFinish()));

        btn_back2main = new QPushButton(this);
        btn_back2main->setText("GoBack");
        connect(btn_back2main, SIGNAL(clicked()), this, SLOT(moveMain()));

        airborneLayout = new QHBoxLayout;
        airborneLayout->addWidget(btn_airborne_takeoff);
        airborneLayout->addWidget(btn_airborne_marker);
        airborneLayout->addWidget(btn_airborne_joy);
        airborneLayout->addWidget(btn_airborn_finished);
        airborneLayout->addWidget(btn_back2main);
        mainLayout->addLayout(airborneLayout);
        setLayout(mainLayout);
        btn_airborne_takeoff->setEnabled(true);
        btn_airborne_marker->setDisabled(true);
        btn_airborne_joy->setDisabled(true);
        btn_airborn_finished->setDisabled(true);
        btn_back2main->setEnabled(true);

    }

    void RvizVisualToolsGui::moveMapStart() {
        btn_map_finish->setEnabled(true);
        btn_map_init->setDisabled(true);
        remote_receiver.MapBuilding();
    }

    void RvizVisualToolsGui::moveMapFinished(){
        btn_map_init->setEnabled(true);
        btn_map_finish->setDisabled(true);

        remote_receiver.MapFinished();
    }

    void RvizVisualToolsGui::moveRepeatLoad(){
        remote_receiver.TeachLoadPath();
        btn_back2main->setDisabled(true);
        btn_repeat_load->setDisabled(true);
        btn_repeat_reset->setEnabled(true);
    }

    void RvizVisualToolsGui::moveRepeatGo() {
        btn_back2main->setDisabled(true);
        btn_repeat_land->setEnabled(true);
        btn_repeat_go->setDisabled(true);
        remote_receiver.RepeatGo();

    }

    void RvizVisualToolsGui::moveRepeatLand(){
        btn_back2main->setEnabled(true);
        btn_repeat_go->setEnabled(true);
        btn_repeat_land->setDisabled(true);
        remote_receiver.RepeatLand();
    }

    void RvizVisualToolsGui::moveRepeatReset(){
        btn_back2main->setEnabled(true);
        btn_repeat_load->setEnabled(true);
        btn_repeat_go->setDisabled(true);
        btn_repeat_land->setDisabled(true);
        remote_receiver.RepeatReset();
    }

    void RvizVisualToolsGui::moveMain() {

        btn_mapping->setEnabled(true);
        btn_teach->setEnabled(true);
        btn_repeat->setEnabled(true);

        std_msgs::String gui_state_;
        gui_state_ = remote_receiver.check_gui_state();
        ROS_INFO("%f",gui_state_.data);

        if (gui_state_.data == "TEACH_INIT"
                               || gui_state_.data == "TEACH_FINISHED" || gui_state_.data =="TEACH_LOAD_FILE"){

            // remove teach layout
            delete (btn_teach_joyinit);
            delete (btn_teach_joyfinish);
            delete (btn_teach_reset);
            delete (teachLayout);
        }
        else if(gui_state_.data == "REPEAT_INIT" || gui_state_.data == "REPEAT_LAND") {
            // remove repeat layout
            delete(btn_repeat_load);
            delete(btn_repeat_go);
            delete(btn_repeat_land);
            delete(btn_repeat_reset);
            delete(repeatLayout);
        }
        else if(gui_state_.data =="MAP_INIT" || gui_state_.data =="MAP_BUILDING"
                ||gui_state_.data =="MAP_FINISHED"){
            delete(btn_map_init);
            delete(btn_map_finish);
            delete(mapLayout);
        }
        else if(gui_state_.data == "AIRBORNE" ||gui_state_.data == "AIRBORNE_MARKER"
                       || gui_state_.data == "AIRBORNE_JOY"){
            delete(btn_airborne_takeoff);
            delete(btn_airborne_marker);
            delete(btn_airborne_joy);
            delete(btn_airborn_finished);
            delete(airborneLayout);
        }

        delete(btn_back2main);
        setLayout(mainLayout);
        remote_receiver.back2main();
    }

    void RvizVisualToolsGui::moveTeachJoyInit(){
        remote_receiver.TeachJoyInit();
        btn_teach_joyinit->setDisabled(true);
        btn_back2main->setDisabled(true);
        btn_teach_reset->setEnabled(true);
        btn_teach_joyfinish->setEnabled(true);
    }

    void RvizVisualToolsGui::moveTeachJoyFinish() {
        remote_receiver.TeachJoyFinish();
        btn_back2main->setEnabled(true);
    }

    void RvizVisualToolsGui::moveTeachJoyReset() {
        btn_back2main->setEnabled(true);
        btn_teach_joyinit->setEnabled(true);
        btn_teach_joyfinish->setDisabled(true);
        btn_teach_reset->setDisabled(true);

        remote_receiver.TeachJoyReset();
    }

    void RvizVisualToolsGui::moveAirborneTakeoff(){
        remote_receiver.airborneTakeoff();
        btn_airborne_takeoff->setDisabled(true);
        btn_airborne_marker->setEnabled(true);
        btn_airborne_joy->setEnabled(true);
        btn_airborn_finished->setEnabled(true);
    }

    void RvizVisualToolsGui::moveAirborneMarker() {
        remote_receiver.airborneMarker();
        btn_airborne_marker->setDisabled(true);
        btn_airborne_joy->setEnabled(true);
        btn_airborn_finished->setEnabled(true);
    }

    void RvizVisualToolsGui::moveAirborneJoy() {
        remote_receiver.airborneJoy();
        btn_airborne_marker->setEnabled(true);
        btn_airborne_joy->setDisabled(true);
        btn_airborn_finished->setEnabled(true);
    }


    void RvizVisualToolsGui::moveAirborneFinish() {
        remote_receiver.airborneFinished();
//        btn_airborne_marker->setEnabled(true);
        btn_airborn_finished->setDisabled(true);

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
