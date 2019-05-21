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

public Q_SLOTS:

protected Q_SLOTS:

  void moveMapping();
  void moveTeach();
  void moveOptimize();
  void moveRepeat();
  void moveMain();

  void moveTeachLoadPath();
  void moveTeachJoyInit();
  void moveTeachJoyFinish();

  void moveRepeatGo();
  void moveRepeatLand();

protected:
    QHBoxLayout* menuLayout;// = new QHBoxLayout;
    QHBoxLayout* teachLayout; // = new QHBoxLayout;
    QHBoxLayout* repeatLayout;
    QVBoxLayout* mainLayout;// = new QVBoxLayout;



    QPushButton* btn_mapping;
  QPushButton* btn_teach;
  QPushButton* btn_optimize;
  QPushButton* btn_repeat;
  QPushButton* btn_back2main;

  QPushButton* btn_teach_load_path;
  QPushButton* btn_teach_joyinit;
  QPushButton* btn_teach_joyfinish;
  QPushButton* btn_teach_reset;

  QPushButton* btn_repeat_go;
  QPushButton* btn_repeat_land;

  RemoteReciever remote_receiver;
};

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H
