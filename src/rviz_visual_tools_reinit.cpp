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

#include "rviz_visual_tools_reinit.h"


namespace rviz_visual_tools
{
    RvizVisualToolsReinit::RvizVisualToolsReinit(QWidget *parent) : rviz::Panel(parent)
    {
        remote_reciever_.enterMap();
        remote_reciever_.EnterAirborne();
        remote_reciever_.reinitAirborne();


//        remote_reciever_.MapBuilding();

        const std::size_t button_size = 10;
//        sub_.state_monitor= nh_.subscribe<std_msgs::Int16>("/demo/state", button_size,
//                                                           &RvizVisualToolsMap::stateCallback, this);
        sub_.drone_states = nh_.subscribe<ground_station_msgs::DroneHeartbeat>("/demo/heartbeat",
                                                                               button_size,&RvizVisualToolsReinit::dronestateCallback,this);

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

        btn_airborne_marker = new QPushButton(this);
        btn_airborne_marker->setText("Marker");
        connect(btn_airborne_marker,SIGNAL(clicked()),this,SLOT(airborneMarker()));

        btn_airborne_joy = new QPushButton(this);
        btn_airborne_joy->setText("Joy");
        connect(btn_airborne_joy,SIGNAL(clicked()),this,SLOT(airborneJoy()));

        btn_airborne_land = new QPushButton(this);
        btn_airborne_land->setText("Land");
        connect(btn_airborne_land,SIGNAL(clicked()),this,SLOT(airborneLand()));

        mainLayout = new QVBoxLayout;
        airborneLayout = new QHBoxLayout;
//        airborneLayout->addWidget(btn_airborne_takeoff);
        airborneLayout->addWidget(btn_airborne_marker);
        airborneLayout->addWidget(btn_airborne_joy);
        airborneLayout->addWidget(btn_airborne_land);

        indicatorLayout = new QHBoxLayout;
        indicatorLayout->addWidget(indicator_.led);
        indicatorLayout->addWidget(indicator_.text);

        mainLayout->addLayout(indicatorLayout);
        mainLayout->addLayout(airborneLayout);
        setLayout(mainLayout);

//        btn_airborne_takeoff->setEnabled(true);
        btn_airborne_marker->setEnabled(true);
        btn_airborne_joy->setEnabled(true);
        btn_airborne_land->setEnabled(true);
    }

    ////////////////// ROS Callback ///////////////////////////////

    void RvizVisualToolsReinit::dronestateCallback(const ground_station_msgs::DroneHeartbeat::ConstPtr &msg){
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
//    void RvizVisualToolsReinit::airborneLand() {
//
//    }

//    void RvizVisualToolsReinit::airborneMarker() {
////        remote_reciever_.airborneMarker();
//
//    }
//
//    void RvizVisualToolsReinit::airborneJoy() {
////        remote_reciever_.airborneJoy();
//    }
//
//    void RvizVisualToolsReinit::airborneLand(){
////        remote_reciever_.airborneFinished();
////        remote_reciever_.MapFinished();
//
//    }




    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsReinit, rviz::Panel)
