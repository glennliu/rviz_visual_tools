//
// Created by lch on 19-5-16.
//



#ifndef RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H
#define RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H

#ifndef Q_MOC_RUN
#include "../../../../../../../opt/ros/kinetic/include/ros/ros.h"

#include "../../../../../../../opt/ros/kinetic/include/rviz/panel.h"
#endif

#include "../../../../../../../usr/include/x86_64-linux-gnu/qt5/QtWidgets/QPushButton"
#include "../../../../../../../usr/include/x86_64-linux-gnu/qt5/QtWidgets/QComboBox"

//#include "rviz_visual_tools/remote_reciever.h"

class QLineEdit;
class QSpinBox;

namespace rviz_visual_tools
{
    class RvizVisualToolsMap : public rviz::Panel
    {
        Q_OBJECT
        public:
            explicit RvizVisualToolsMap(QWidget* parent = 0);
            virtual void load(const rviz::Config& config);
            virtual void save(rviz::Config config) const;

        public Q_SLOTS:

        protected Q_SLOTS:
            void startMapping();
            void finishedMaping();

        protected:
            QPushButton* btn_start;
            QPushButton* btn_finished;

//        RemoteReciever remote_reciever_;
    };

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__MAPPING_PANEL_GUI_H
