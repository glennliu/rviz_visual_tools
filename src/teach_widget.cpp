//
// Created by lch on 19-7-5.
//

#include <cstdio>

#include <QGroupBox>
//#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
//#include <QVBoxLayout>
#include <QtGui/QPalette>
#include <QtWidgets/QtWidgets>

#include "teach_widget.h"
//#include "rv"

namespace rviz_visual_tools {
    TeachWidget::TeachWidget(QWidget *parent) : rviz::Panel(parent){

        btn_teach_joyinit = new QPushButton(this);
        btn_teach_joyinit->setText("Init");
        connect(btn_teach_joyinit,SIGNAL(clicked()), this, SLOT(moveTeachJoyInit()));

        menuLayout = new QHBoxLayout;
        menuLayout->addWidget(btn_teach_joyinit);

        mainLayout = new QVBoxLayout;
        mainLayout->addLayout(menuLayout);
        setLayout(mainLayout);

        btn_teach_joyinit->setEnabled(true);

    }


}


