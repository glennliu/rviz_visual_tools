//
// Created by lch on 19-5-16.
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

#include "rviz_visual_tools_map.h"


namespace rviz_visual_tools
{
    RvizVisualToolsMap::RvizVisualToolsMap(QWidget *parent) : rviz::Panel(parent)
    {
        remote_reciever_.enterMap();

        btn_handheld = new QPushButton(this);
        btn_handheld->setText("Handheld");
        connect(btn_handheld,SIGNAL(clicked()),this,SLOT(enterHandheld()));

        btn_airborne = new QPushButton(this);
        btn_airborne->setText("Airborne");
        connect(btn_airborne,SIGNAL(clicked()),this,SLOT(enterAirborne()));


        //Honrizontal Layout
        auto* hlayout1 = new QHBoxLayout;
        hlayout1->addWidget(btn_handheld);
        hlayout1->addWidget(btn_airborne);

        // Verticle Layout
        mainLayout = new QVBoxLayout;
        mainLayout->addLayout(hlayout1);
        setLayout(mainLayout);

        btn_handheld->setEnabled(true);
        btn_airborne->setEnabled(true);
    }

    void RvizVisualToolsMap::enterHandheld(){
        remote_reciever_.enterHandheld();
        btn_handheld_start = new QPushButton(this);
        btn_handheld_start->setText("Start");
        connect(btn_handheld_start,SIGNAL(clicked()),this,SLOT(handheldStart()));

        btn_handheld_finish = new QPushButton(this);
        btn_handheld_finish->setText("Finish");
        connect(btn_handheld_finish,SIGNAL(clicked()),this,SLOT(handheldFinish()));

        btn_goback = new QPushButton(this);
        btn_goback->setText("Goback");
        connect(btn_goback,SIGNAL(clicked()),this,SLOT(goback()));

        handheldLayout = new QHBoxLayout;
        handheldLayout->addWidget(btn_handheld_start);
        handheldLayout->addWidget(btn_handheld_finish);
        handheldLayout->addWidget(btn_goback);

        mainLayout->addLayout(handheldLayout);
        setLayout(mainLayout);

        btn_airborne->setDisabled(true);
        btn_handheld_start->setEnabled(true);
        btn_handheld_finish->setDisabled(true);
        btn_goback->setEnabled(true);

    }


    void RvizVisualToolsMap::handheldStart(){
        remote_reciever_.MapBuilding();
        btn_handheld_start->setDisabled(true);
        btn_handheld_finish->setEnabled(true);
    }

    void RvizVisualToolsMap::handheldFinish(){
        remote_reciever_.MapFinished();
        btn_handheld_start->setEnabled(true);
        btn_handheld_finish->setDisabled(true);

    }

/////////////////////////// Airborne ////////////////////////////////
    void RvizVisualToolsMap::enterAirborne(){
        remote_reciever_.enterAirborne();

        btn_airborne_takeoff = new QPushButton(this);
        btn_airborne_takeoff->setText("Takeoff");
        connect(btn_airborne_takeoff,SIGNAL(clicked()),this,SLOT(airborneTakeoff()));

        btn_airborne_marker = new QPushButton(this);
        btn_airborne_marker->setText("Marker");
        connect(btn_airborne_marker,SIGNAL(clicked()),this,SLOT(airborneMarker()));

        btn_airborne_joy = new QPushButton(this);
        btn_airborne_joy->setText("Joy");
        connect(btn_airborne_joy,SIGNAL(clicked()),this,SLOT(airborneJoy()));

        btn_airborne_land = new QPushButton(this);
        btn_airborne_land->setText("Land");
        connect(btn_airborne_land,SIGNAL(clicked()),this,SLOT(airborneLand()));

        btn_goback = new QPushButton(this);
        btn_goback->setText("Goback");
        connect(btn_goback,SIGNAL(clicked()),this,SLOT(goback()));

        airborneLayout = new QHBoxLayout;
        airborneLayout->addWidget(btn_airborne_takeoff);
        airborneLayout->addWidget(btn_airborne_marker);
        airborneLayout->addWidget(btn_airborne_joy);
        airborneLayout->addWidget(btn_airborne_land);
        airborneLayout->addWidget(btn_goback);

        mainLayout->addLayout(airborneLayout);
        setLayout(mainLayout);

        btn_airborne_takeoff->setEnabled(true);
        btn_airborne_marker->setDisabled(true);
        btn_airborne_joy->setDisabled(true);
        btn_airborne_land->setDisabled(true);
        btn_goback->setEnabled(true);

    }

    void RvizVisualToolsMap::airborneTakeoff() {
        remote_reciever_.airborneTakeoff();

        btn_airborne_takeoff->setDisabled(true);
        btn_airborne_marker->setEnabled(true);
        btn_airborne_joy->setDisabled(true);
        btn_airborne_land->setEnabled(true);
    }

    void RvizVisualToolsMap::airborneMarker() {
        remote_reciever_.airborneMarker();

        btn_airborne_marker->setDisabled(true);
        btn_airborne_joy->setEnabled(true);
    }

    void RvizVisualToolsMap::airborneJoy() {
        remote_reciever_.airborneJoy();
        btn_airborne_marker->setEnabled(true);
        btn_airborne_joy->setDisabled(true);
    }

    void RvizVisualToolsMap::airborneLand(){
        remote_reciever_.airborneFinished();
        btn_airborne_takeoff->setEnabled(true);
        btn_airborne_marker->setDisabled(true);
        btn_airborne_joy->setDisabled(true);
        btn_airborne_land->setDisabled(true);
    }


    void RvizVisualToolsMap::goback() {
        std_msgs::Int16 gui_code;
        gui_code = remote_reciever_.check_gui_code();
        switch(gui_code.data){
            case GUI_MAP_HANDHELD:
                delete(btn_handheld_start);
                delete(btn_handheld_finish);
                delete(handheldLayout);
                break;

            case GUI_MAP_AIRBORNE:
                delete(btn_airborne_takeoff);
                delete(btn_airborne_marker);
                delete(btn_airborne_joy);
                delete(btn_airborne_land);
                delete(airborneLayout);
                break;
        }

        delete(btn_goback);
        setLayout(mainLayout);

        btn_handheld->setEnabled(true);
        btn_airborne->setEnabled(true);

    }

    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsMap, rviz::Panel)
