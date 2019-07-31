//
// Created by lch on 7/23/19.
//



#include <cstdio>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QtGui/QPalette>
#include <QtWidgets/QtWidgets>

#include "rviz_visual_tools_repeat.h"


namespace rviz_visual_tools
{
    RvizVisualToolsRepeat::RvizVisualToolsRepeat(QWidget *parent) : rviz::Panel(parent)
    {
        remote_receiver.EnterRepeat();

        const std::size_t button_size = 10;
        sub_state_monitor= nh_.subscribe<std_msgs::Int16>("/demo/state", button_size,
                &RvizVisualToolsRepeat::stateCallback, this);

        btn_load = new QPushButton(this);
        btn_load->setText("LoadPath");
        connect(btn_load,SIGNAL(clicked()),this,SLOT(moveLoad()));

        btn_takeoff = new QPushButton(this);
        btn_takeoff->setText("Takeoff");
        connect(btn_takeoff,SIGNAL(clicked()),this,SLOT(moveTakeoff()));

        btn_land = new QPushButton(this);
        btn_land->setText("Land");
        connect(btn_land,SIGNAL(clicked()),this,SLOT(moveLand()));

        btn_reset = new QPushButton(this);
        btn_reset->setText("Reset");
        connect(btn_reset,SIGNAL(clicked()),this,SLOT(reset()));

        //Honrizontal Layout
        auto* hlayout1 = new QHBoxLayout;
        hlayout1->addWidget(btn_load);
        hlayout1->addWidget(btn_takeoff);
        hlayout1->addWidget(btn_land);
        hlayout1->addWidget(btn_reset);

        // Verticle Layout
        auto* layout = new QVBoxLayout;
        layout->addLayout(hlayout1);
        setLayout(layout);

        btn_load->setEnabled(true);
        btn_takeoff->setDisabled(true);
        btn_land->setDisabled(true);
        btn_reset->setDisabled(true);
    }

    void RvizVisualToolsRepeat::stateCallback(const std_msgs::Int16::ConstPtr &msg) {
//        ROS_INFO("haha");
        flight_state_value = msg->data;
        switch (flight_state_value){
            case 2:
                btn_takeoff->setEnabled(true);
                break;
        }
    }


    void RvizVisualToolsRepeat::moveLoad() {
        int32_t tmp_int;

        remote_receiver.TeachLoadPath();
//        tmp_int= remote_control().get_button_value();
//        ROS_INFO("BUTTON value: %d",tmp_int);

        btn_load->setDisabled(true);
//        btn_takeoff->setEnabled(true);
        btn_land->setDisabled(true);
        btn_reset->setEnabled(true);
    }

    void RvizVisualToolsRepeat::moveTakeoff() {
        remote_receiver.RepeatGo();

        btn_load->setDisabled(true);
        btn_takeoff->setDisabled(true);
        btn_land->setEnabled(true);
        btn_reset->setDisabled(true);
    }

    void RvizVisualToolsRepeat::moveLand() {
        remote_receiver.RepeatLand();

        btn_takeoff->setDisabled(true);
        btn_land->setDisabled(true);
        btn_reset->setEnabled(true);
    }

    void RvizVisualToolsRepeat::reset() {
        remote_receiver.RepeatReset();

        btn_load->setEnabled(true);
        btn_takeoff->setDisabled(true);
        btn_land->setDisabled(true);
        btn_reset->setEnabled(true);
    }



    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsRepeat, rviz::Panel)
