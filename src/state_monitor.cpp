//
// Created by innozone on 31/7/2019.
//

#include <state_monitor.h>

namespace rviz_visual_tools
{
    StateMonitor::StateMonitor(const ros::NodeHandle& nh) : nh_(nh){
        std::string flight_state_topic = "/demo/state";
        const std::size_t button_queue_size = 10;

//        sub.flight_state_ = nh_.subscribe<std_msgs::Int16>(flight_state_topic,button_queue_size,
//                &StateMonitor::flightstateCallback, this);
    }

    void StateMonitor::flightstateCallback(const std_msgs::Int16& msg) {
        flight_state = msg.data;
    }


}
