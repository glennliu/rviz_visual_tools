//
// Created by innozone on 10/8/2019.
//

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

#include "rviz_visual_tools_inspect.h"


namespace rviz_visual_tools
{
    RvizVisualToolsInspect::RvizVisualToolsInspect(QWidget *parent) : rviz::Panel(parent)
    {
//        remote_reciever_.enterMap();
//        remote_reciever_.EnterAirborne();
//        remote_reciever_.reinitAirborne();
//        remote_reciever_.MapBuilding();

        const std::size_t button_size = 10;
//        sub_.state_monitor= nh_.subscribe<std_msgs::Int16>("/demo/state", button_size,
//                                                           &RvizVisualToolsMap::stateCallback, this);
        sub_.drone_states = nh_.subscribe<ground_station_msgs::DroneHeartbeat>("/demo/heartbeat",
                                                                               button_size,&RvizVisualToolsInspect::dronestateCallback,this);

        // indicators
        indicator_.led = new QPushButton(this);
        indicator_.led->setFixedSize(20,20);
        indicator_.led->setText(" ");
        indicator_.led->setStyleSheet("background-color:red");
        indicator_.led->update();

        indicator_.text = new QPushButton(this);
        indicator_.text->setText(" ");
        indicator_.text->setFixedSize(400,30);
        indicator_.text->update();
        indicator_.text->setDisabled(true);

        gui_state = GUI_MAP_AIRBORNE;
//
//        btn_airborne_takeoff = new QPushButton(this);
//        btn_airborne_takeoff->setText("Takeoff");
//        connect(btn_airborne_takeoff,SIGNAL(clicked()),this,SLOT(airborneTakeoff()));

        btn_takeoff = new QPushButton(this);
        btn_takeoff->setText("Takeoff");
        connect(btn_takeoff,SIGNAL(clicked()),this,SLOT(takeoff()));

        btn_land = new QPushButton(this);
        btn_land->setText("Reset");
        connect(btn_land,SIGNAL(clicked()),this,SLOT(land()));



        mainLayout = new QVBoxLayout;
        inspectLayout = new QHBoxLayout;
//        inspectLayout->addWidget(btn_airborne_takeoff);
        inspectLayout->addWidget(btn_takeoff);
        inspectLayout->addWidget(btn_land);

        indicatorLayout = new QHBoxLayout;
        indicatorLayout->addWidget(indicator_.led);
        indicatorLayout->addWidget(indicator_.text);

        mainLayout->addLayout(indicatorLayout);
        mainLayout->addLayout(inspectLayout);
        setLayout(mainLayout);

//        btn_airborne_takeoff->setEnabled(true);
        btn_takeoff->setEnabled(true);
        btn_land->setDisabled(true);
    }

    ////////////////// ROS Callback ///////////////////////////////

    void RvizVisualToolsInspect::dronestateCallback(const ground_station_msgs::DroneHeartbeat::ConstPtr &msg){
        QString error_message=" ";

        if (msg->djiros_state && msg->vo_state && msg->loop_state
            && msg->n1ctrl_state && msg->planner_state && msg->takeoff_state
            && msg->gear_state && msg->rc_state){
            indicator_.led->setStyleSheet("background-color:green");
        }
        else indicator_.led->setStyleSheet("background-color:red");

        if (!msg->djiros_state){
            error_message = error_message + "djiros;";
        }
        if(!msg->vo_state){
            error_message = error_message + " vo;";
        }
        if(!msg->loop_state){
            error_message = error_message + " loop;";
        }
        if(!msg->n1ctrl_state){
            error_message = error_message + " n1ctrl;";
        }
        if(!msg->planner_state){
            error_message = error_message + " planner;";
        }
        if(!msg->takeoff_state){
            error_message = error_message + " takeoff;";
        }
        if(!msg->gear_state){
            error_message = error_message + " gear;";
        }
        if(!msg->rc_state){
            error_message = error_message + " rc;";
        }
        indicator_.text->setText(error_message);


    }


    ///////////////// Button Functions ///////////////////////
//
//    void RvizVisualToolsInspect::airborneLand() {
//
//    }

    void RvizVisualToolsInspect::takeoff() {
        remote_reciever_.inspectBegin();
        btn_land->setEnabled(true);
        btn_takeoff->setDisabled(true);
    }

    void RvizVisualToolsInspect::land() {
        btn_takeoff->setEnabled(true);
    }
//
//    void RvizVisualToolsInspect::airborneLand(){
////        remote_reciever_.airborneFinished();
////        remote_reciever_.MapFinished();
//
//    }




    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsInspect, rviz::Panel)
