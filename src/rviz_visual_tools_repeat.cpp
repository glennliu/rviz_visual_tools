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
        sub_.state_monitor= nh_.subscribe<std_msgs::Int16>("/demo/state", button_size,
                &RvizVisualToolsRepeat::stateCallback, this);

        sub_.drone_states = nh_.subscribe<ground_station_msgs::DroneHeartbeat>("/demo/heartbeat",
                button_size,&RvizVisualToolsRepeat::dronestateCallback,this);

        sub_.bat_state = nh_.subscribe<geometry_msgs::PointStamped>("/djiros/battery_status",button_size,
                &RvizVisualToolsRepeat::dronebatCallback, this);

        // command buttons
        btn_load = new QPushButton(this);
        btn_load->setText("LoadPath");
        connect(btn_load,SIGNAL(clicked()),this,SLOT(moveLoad()));

        btn_takeoff = new QPushButton(this);
        btn_takeoff->setText("Takeoff");
        connect(btn_takeoff,SIGNAL(clicked()),this,SLOT(moveTakeoff()));

        btn_reset = new QPushButton(this);
        btn_reset->setText("Reset");
        connect(btn_reset,SIGNAL(clicked()),this,SLOT(reset()));

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

        indicator_.battery = new QPushButton(this);
        indicator_.battery->setText(" Battery ");
        indicator_.battery->update();
        indicator_.battery->setDisabled(true);

        //Honrizontal Layout
        auto* layout_cmd = new QHBoxLayout;
        layout_cmd->addWidget(btn_load);
        layout_cmd->addWidget(btn_takeoff);
        layout_cmd->addWidget(btn_reset);

        auto* layout_indicator = new QHBoxLayout;
        layout_indicator->addWidget(indicator_.led);
        layout_indicator->addWidget(indicator_.text);
        layout_indicator->addWidget(indicator_.battery);

        // Verticle Layout
        auto* layout = new QVBoxLayout;
        layout->addLayout(layout_indicator);
        layout->addLayout(layout_cmd);
        setLayout(layout);

        btn_load->setEnabled(true);
        btn_takeoff->setDisabled(true);
        btn_reset->setDisabled(true);
    }

    ////////////////// ROS Callback ///////////////////////////////

    void RvizVisualToolsRepeat::stateCallback(const std_msgs::Int16::ConstPtr &msg) {
        flight_state_value = msg->data;
        switch (flight_state_value){
            case READY_TO_TAKE_OFF:
                if (path_loaded_flag &!retakeoff){
                    btn_takeoff->setEnabled(true);
                }
                break;
        }
    }

    void RvizVisualToolsRepeat::dronestateCallback(const ground_station_msgs::DroneHeartbeat::ConstPtr &msg){
        QString error_message=" ";
        bool battery_correct =true ;

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
        if(bat_voltage <15.4){
            error_message = error_message +"battery";
            battery_correct = false;
        }
        indicator_.text->setText(error_message);

        if (msg->djiros_state && msg->vo_state && msg->loop_state
            && msg->n1ctrl_state && msg->planner_state && msg->takeoff_state
            && msg->gear_state && msg->rc_state && battery_correct){
            indicator_.led->setStyleSheet("background-color:green");
        }
        else indicator_.led->setStyleSheet("background-color:red");


    }

    void RvizVisualToolsRepeat::dronebatCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
        QString bat_msg;
        bat_msg.push_back("Bat:");

        bat_voltage = msg->point.x;

        std::ostringstream bat_str;
        bat_str<<std::fixed <<std::setprecision(2) <<msg->point.x;
        std::string s(bat_str.str());
        bat_msg.push_back(s.data());
        bat_msg.push_back("V");

        indicator_.battery->setText(bat_msg);
    }

    ///////////////// Button Functions ///////////////////////
    void RvizVisualToolsRepeat::moveLoad() {
        remote_receiver.TeachLoadPath();

        btn_load->setDisabled(true);
//        btn_takeoff->setEnabled(true);
//        btn_land->setDisabled(true);
        btn_reset->setEnabled(true);

        path_loaded_flag = true;
    }

    void RvizVisualToolsRepeat::moveTakeoff() {
        remote_receiver.RepeatGo();

        btn_load->setDisabled(true);
        btn_takeoff->setDisabled(true);
//        btn_land->setEnabled(true);
        btn_reset->setDisabled(true);

        retakeoff = true;
    }


    void RvizVisualToolsRepeat::reset() {
        remote_receiver.RepeatReset();

        btn_load->setEnabled(true);
        btn_takeoff->setDisabled(true);
//        btn_land->setDisabled(true);
        btn_reset->setEnabled(true);
    }



    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsRepeat, rviz::Panel)
