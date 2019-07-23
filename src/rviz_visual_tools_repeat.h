//
// Created by lch on 7/23/19.
//

#ifndef SRC_RVIZ_VISUAL_TOOLS_REPEAT_H
#define SRC_RVIZ_VISUAL_TOOLS_REPEAT_H

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
//#include <rviz_visual_tools/teach_cmd.h>

#include <std_msgs/Int16.h>

namespace rviz_visual_tools
{
    class RvizVisualToolsRepeat : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit RvizVisualToolsRepeat(QWidget* parent = 0);

    public Q_SLOTS:

    protected Q_SLOTS:
        void moveLoad();
        void moveTakeoff();
        void moveLand();
        void reset();

    protected:
        QHBoxLayout* menuLayout;
        QVBoxLayout* mainLayout;

        QPushButton* btn_load;
        QPushButton* btn_takeoff;
        QPushButton* btn_land;
        QPushButton* btn_reset;

        RemoteReciever remote_receiver;
    };
}




#endif //SRC_RVIZ_VISUAL_TOOLS_REPEAT_H
