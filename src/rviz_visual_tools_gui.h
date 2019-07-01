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

// TODO(dave): convert to flow layout:
// http://doc.qt.io/qt-5/qtwidgets-layouts-flowlayout-example.html

#ifndef RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H
#define RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>

#include "../include/rviz_visual_tools/remote_receiver.h"
//#include <rviz_visual_tools/teach_cmd.h>
//#include <rviz_visual_tools/teach_cmd.h>

#include <std_msgs/Int16.h>

class QLineEdit;
class QSpinBox;

namespace rviz_visual_tools
{
    class RvizVisualToolsGui : public rviz::Panel
    {
    Q_OBJECT
    public:

//    auto* layout = new QVBoxLayout;
//    auto* hlayout1 = new QHBoxLayout;
        explicit RvizVisualToolsGui(QWidget* parent = 0);

        virtual void load(const rviz::Config& config);
        virtual void save(rviz::Config config) const;
        void enableTakeoff(){
            btn_airborne_takeoff->setEnabled(true);
        }

    public Q_SLOTS:

    protected Q_SLOTS:

        void moveMapping();
        void moveTeach();
//        void moveOptimize();
        void moveRepeat();
        void moveAirborne();
        void moveMain();

        void moveMapStart();
        void moveMapFinished();

        void moveTeachLoadPath();
        void moveTeachJoyInit();
        void moveTeachJoyFinish();
        void moveTeachJoyReset();

        void moveRepeatLoad();
        void moveRepeatGo();
        void moveRepeatLand();
        void moveRepeatReset();

        void moveAirborneTakeoff();
        void moveAirborneMarker();
        void moveAirborneJoy();
        void moveAirborneFinish();

        void disableMainMenu();

//  void repeat_init_check_callback(const std_msgs::Int16 &msg){
//      ROS_INFO("FLAG!!!");
//  }

    protected:
        QHBoxLayout* menuLayout;// = new QHBoxLayout;
        QHBoxLayout* mapLayout;
        QHBoxLayout* teachLayout; // = new QHBoxLayout;
        QHBoxLayout* repeatLayout;
        QHBoxLayout* airborneLayout;
        QVBoxLayout* mainLayout;// = new QVBoxLayout;

        QPushButton* btn_mapping;
        QPushButton* btn_teach;
        QPushButton* btn_repeat;
        QPushButton* btn_airborne;
        QPushButton* btn_back2main;

        QPushButton* btn_map_init;
        QPushButton* btn_map_finish;

        QPushButton* btn_teach_joyinit;
        QPushButton* btn_teach_joyfinish;
        QPushButton* btn_teach_reset;

        QPushButton* btn_repeat_load;
        QPushButton* btn_repeat_go;
        QPushButton* btn_repeat_land;
        QPushButton* btn_repeat_reset;

        QPushButton* btn_airborne_takeoff;
        QPushButton* btn_airborne_marker;
        QPushButton* btn_airborne_joy;
        QPushButton* btn_airborn_finished;

//  ros::Subscriber repeat_init_check;

        RemoteReciever remote_receiver;
    };

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H
