//
// Created by lch on 19-7-5.
//

#ifndef PROJECT_TEACH_WIDGET_H
#define PROJECT_TEACH_WIDGET_H

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
    class TeachWidget : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit TeachWidget(QWidget* parent = 0);

    public Q_SLOTS:

    protected Q_SLOTS:
        void moveTeachJoyInit();
        void moveTeachJoyFinish();
        void moveTeachJoyReset();

        protected:
        QHBoxLayout* menuLayout;
        QVBoxLayout* mainLayout;

        QPushButton* btn_teach_joyinit;
        QPushButton* btn_teach_joyfinish;
        QPushButton* btn_teach_reset;

        RemoteReciever remote_receiver;
    };
}



#endif //PROJECT_TEACH_WIDGET_H
