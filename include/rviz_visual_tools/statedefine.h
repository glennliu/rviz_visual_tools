//  "statedefine.h"

#define WAIT_FOR_BEGIN      0
// #define ON_PROGRESS         1
// #define FINISH              2
#define RC_A_MODE           0.0
#define RC_P_MODE           1.0
#define GEAR_UP             -1
#define GEAR_DOWN           -0.5
// #define ON_THE_GROUND       10
// #define IN_THE_AIR          11
// #define READY_TO_TAKE_OFF   12
#define REPEAT_MODE         5
#define MAPPING_MODE        6
#define DEMO_MODE           7
// #define TAKE_OFF            5
// #define LANDING             6
#define ALIGN_VIO_WORLD     8
// #define EXECUTE_TRAJ         8
#define EMERGENCY           90

#define VALID               1
#define INVALID             2

#define SYSTEM_INITIALIZE   10
#define READY_TO_TAKE_OFF   11      // [GUI] will send to gui, 
#define TAKE_OFF            12
#define TAKE_OFF_FINISH     13
#define HOVER               14
#define PRE_EXECUTE_TRAJ    15
#define EXECUTE_TRAJ        16      // [GUI] will send to gui. after recieve this, gui can send points
#define TRAJ_FINISH         17
#define FINISH_LANDING      18
#define LANDING_FINISH      19

#define REPEAT_BEGIN        21 
#define MAP_BEGIN           22      // [GUI] get from GUI, then takeoff and hover
#define WAYPOINT_MAPPING    23      // [GUI] get from GUI, then go to waypoint mapping mode
#define JOYSTICK_MAPPING    24      // [GUI] get from GUI. then go to Joystick mapping mode
#define CTRL_CMD_MODE       31      // n1ctrl get in to CMD_CTRL state, double hand shake: demo_main.cpp <----> n1ctrl

#define FINISH              99      // [GUI] get from gui, then it will landing
/*
MAPPING: 

1. Main --> READY_TO_TAKE_OFF --> GUI --> MAP_BEGIN --> Main -->Takeoff .... 
2. Main --> EXECUTE_TRAJ --> GUI --> PoseStamped
3. GUI  --> FINISH --> Main --> Landing...
*/