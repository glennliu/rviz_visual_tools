//
// Created by innozone on 10/8/2019.
//

#ifndef SRC_RVIZ_VISUAL_TOOLS_REINIT_H
#define SRC_RVIZ_VISUAL_TOOLS_REINIT_H

#endif //SRC_RVIZ_VISUAL_TOOLS_REINIT_H
//
// Created by lch on 19-5-16.
//



#ifndef RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H
#define RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif


#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QComboBox>

#include "../include/rviz_visual_tools/remote_receiver.h"
#include <std_msgs/Int16.h>
#include <ground_station_msgs/DroneHeartbeat.h>
#include <rviz_visual_tools/mav_cmd_enum.h>

class QLineEdit;
class QSpinBox;

namespace rviz_visual_tools
{
    class RvizVisualToolsReinit : public rviz::Panel
    {
    Q_OBJECT
    public:
        explicit RvizVisualToolsReinit(QWidget* parent = 0);
//        void stateCallback(const std_msgs::Int16::ConstPtr &msg);
        void dronestateCallback(const ground_station_msgs::DroneHeartbeat::ConstPtr &msg);


    public Q_SLOTS:

    protected Q_SLOTS:
        void enterAirborne();

        void airborneTakeoff();
        void airborneMarker();
        void airborneJoy();
        void airborneLand();

        void goback();

        void invalid_button();

    private:
        ros::NodeHandle nh_;

        struct subscriber{
            ros::Subscriber state_monitor, drone_states;
        }sub_;

        int16_t gui_state = GUI_MAIN;


    protected:
        QVBoxLayout* mainLayout;
//        QHBoxLayout* handheldLayout;
        QHBoxLayout* airborneLayout;
        QHBoxLayout* indicatorLayout;

        QPushButton* btn_airborne;


        QPushButton* btn_airborne_takeoff;
        QPushButton* btn_airborne_marker;
        QPushButton* btn_airborne_joy;
        QPushButton* btn_airborne_land;


        struct indicator{
            QPushButton* led;
            QPushButton* text;
        }indicator_;

        RemoteReciever remote_reciever_;

    private:

    };

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H
