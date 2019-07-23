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

class QLineEdit;
class QSpinBox;

namespace rviz_visual_tools
{
    class RvizVisualToolsMap : public rviz::Panel
    {
        Q_OBJECT
        public:
            explicit RvizVisualToolsMap(QWidget* parent = 0);
//            virtual void load(const rviz::Config& config);
//            virtual void save(rviz::Config config) const;

        public Q_SLOTS:

        protected Q_SLOTS:
            void enterHandheld();
            void enterAirborne();

            void handheldStart();
            void handheldFinish();

            void airborneTakeoff();
            void airborneMarker();
            void airborneJoy();
            void airborneLand();

            void goback();

        protected:
            QVBoxLayout* mainLayout;
            QHBoxLayout* handheldLayout;
            QHBoxLayout* airborneLayout;

            QPushButton* btn_handheld;
            QPushButton* btn_airborne;

            QPushButton* btn_handheld_start;
            QPushButton* btn_handheld_finish;

            QPushButton* btn_airborne_takeoff;
            QPushButton* btn_airborne_marker;
            QPushButton* btn_airborne_joy;
            QPushButton* btn_airborne_land;

            QPushButton* btn_goback;


        RemoteReciever remote_reciever_;

    private:

    };

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H
