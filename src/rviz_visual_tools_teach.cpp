//
// Created by lch on 19-7-5.
//

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

#include "rviz_visual_tools_teach.h"

namespace rviz_visual_tools {
    RvizVisualToolsTeach::RvizVisualToolsTeach(QWidget *parent) : rviz::Panel(parent){

        remote_receiver.EnterTeach();

        btn_teach_joyinit = new QPushButton(this);
        btn_teach_joyinit->setText("Joy Init");
        connect(btn_teach_joyinit,SIGNAL(clicked()), this, SLOT(moveTeachJoyInit()));

        btn_teach_joyfinish = new QPushButton(this);
        btn_teach_joyfinish->setText("Joy Finish");
        connect(btn_teach_joyfinish,SIGNAL(clicked()), this, SLOT(moveTeachJoyFinish()));

        btn_teach_reset = new QPushButton(this);
        btn_teach_reset->setText("Reset");
        connect(btn_teach_reset,SIGNAL(clicked()), this, SLOT(moveTeachJoyReset()));

        menuLayout = new QHBoxLayout;
        menuLayout->addWidget(btn_teach_joyinit);
        menuLayout->addWidget(btn_teach_joyfinish);
        menuLayout->addWidget(btn_teach_reset);

        mainLayout = new QVBoxLayout;
        mainLayout->addLayout(menuLayout);
        setLayout(mainLayout);

        btn_teach_joyinit->setEnabled(true);
        btn_teach_joyfinish->setDisabled(true);
        btn_teach_reset->setDisabled(true);
    }

    void RvizVisualToolsTeach::moveTeachJoyInit(){
        remote_receiver.TeachJoyInit();

        btn_teach_joyinit->setDisabled(true);
        btn_teach_joyfinish->setEnabled(true);
        btn_teach_reset->setEnabled(true);
    }

    void RvizVisualToolsTeach::moveTeachJoyFinish() {
        remote_receiver.TeachJoyFinish();

        btn_teach_joyfinish->setDisabled(true);
    }

    void RvizVisualToolsTeach::moveTeachJoyReset() {
        remote_receiver.TeachJoyReset();

        btn_teach_joyinit->setEnabled(true);
        btn_teach_joyfinish->setDisabled(true);
        btn_teach_reset->setDisabled(true);
    }

}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsTeach, rviz::Panel)
