//
// Created by lch on 19-5-12.
//

#ifndef PROJECT_MAV_CMD_ENUM_H
#define PROJECT_MAV_CMD_ENUM_H

enum mav_cmder{
//    general command
    MAV_CMD_INIT = 1,
    MAV_CMD_RUN = 2,
    MAV_CMD_PAUSE = 3,
    MAV_CMD_FINISH = 4,
    MAV_CMD_RESET = 9
};

enum gui_state{
    GUI_MAIN = 20,
    GUI_MAP_MAIN = 21,
    GUI_MAP_HANDHELD = 22,
    GUI_MAP_AIRBORNE = 23,
    GUI_TEACH = 25,
    GUI_REPEAT = 26
};

#endif //PROJECT_MAV_CMD_ENUM_H
