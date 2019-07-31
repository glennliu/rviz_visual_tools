//
// Created by innozone on 31/7/2019.
//

#ifndef SRC_STATE_MONITOR_H
#define SRC_STATE_MONITOR_H

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>

namespace rviz_visual_tools
{
    class StateMonitor
    {
    public:
        explicit StateMonitor(const ros::NodeHandle& nh);

        int16_t get_flight_state(){
            return flight_state;
        }

        void flightstateCallback(const std_msgs::Int16 &msg);



    private:
        ros::NodeHandle nh_;

        struct subscriber{
            ros::Subscriber flight_state_;
        }sub;

        int16_t flight_state;


    };

    typedef std::shared_ptr<StateMonitor> StateMonitorPtr;
    typedef std::shared_ptr<const StateMonitor> StateMonitorConstPtr;

}

#endif //SRC_STATE_MONITOR_H
