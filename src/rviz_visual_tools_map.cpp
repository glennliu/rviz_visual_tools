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
        btn_start = new QPushButton(this);
        btn_start->setText("Start");
        connect(btn_start,SIGNAL(clicked()),this,SLOT(startMapping()));

        btn_finished = new QPushButton(this);
        btn_finished->setText("Finished");
        connect(btn_finished,SIGNAL(clicked()),this,SLOT(finishedMaping()));

        //Honrizontal Layout
        auto* hlayout1 = new QHBoxLayout;
        hlayout1->addWidget(btn_start);
        hlayout1->addWidget(btn_finished);

        // Verticle Layout
        auto* layout = new QVBoxLayout;
        layout->addLayout(hlayout1);
        setLayout(layout);

        btn_start->setEnabled(true);
        btn_finished->setEnabled(true);
    }

    void RvizVisualToolsMap::startMapping()
    {
        ROS_INFO("Start Mapping!");
    }

    void RvizVisualToolsMap::finishedMaping()
    {
        ROS_INFO("Mapping Finished!");
    }

    void RvizVisualToolsMap::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void RvizVisualToolsMap::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }

    // end panel gui
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_visual_tools::RvizVisualToolsMap, rviz::Panel)
