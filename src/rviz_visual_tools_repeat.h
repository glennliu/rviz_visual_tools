//
// Created by lch on 7/23/19.
//

#ifndef SRC_RVIZ_VISUAL_TOOLS_REPEAT_H
#define SRC_RVIZ_VISUAL_TOOLS_REPEAT_H

#ifndef Q_MOC_RUN

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPalette>
#include <QLabel>
#include <QColor>
#include <QWidget>
//#include <QPainter>

#include "../include/rviz_visual_tools/remote_receiver.h"
#include <rviz_visual_tools/remote_control.h>
#include "ground_station_msgs/DroneHeartbeat.h"


namespace rviz_visual_tools
{
    class RvizVisualToolsRepeat : public rviz::Panel
    {
    Q_OBJECT
    public:
        explicit RvizVisualToolsRepeat(QWidget* parent = 0);

        void stateCallback(const std_msgs::Int16::ConstPtr &msg);
        void dronestateCallback(const ground_station_msgs::DroneHeartbeat::ConstPtr &msg);

    public Q_SLOTS:

    protected Q_SLOTS:
        void moveLoad();
        void moveTakeoff();
        void moveLand();
        void reset();

    private:
        ros::NodeHandle nh_;

        struct subscriber{
            ros::Subscriber state_monitor, drone_states;
        }sub_;


        int16_t flight_state_value;

        bool path_loaded_flag = false;

        bool retakeoff = false;

    protected:
        QHBoxLayout* menuLayout;
        QVBoxLayout* mainLayout;

        QPushButton* btn_load;
        QPushButton* btn_takeoff;
        QPushButton* btn_land;
        QPushButton* btn_reset;

        // Indicators
        struct indicator{
            QPushButton* led;
            QPushButton* text;
        }indicator_;





        RemoteReciever remote_receiver;
    };
}




#endif //SRC_RVIZ_VISUAL_TOOLS_REPEAT_H
