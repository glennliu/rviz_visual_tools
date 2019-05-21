/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Object for wrapping remote control functionality
*/

#ifndef RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
#define RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H

#include <sensor_msgs/Joy.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <mav_cmd_enum.h>

#include <string.h>

namespace rviz_visual_tools
{
    class RemoteReciever
    {

    public:
        RemoteReciever()
        {
            joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/rviz_visual_tools_gui", 1);
            gui_state_pub_ = nh_.advertise<std_msgs::String>("/gui_state", 1);

            joy_client_ = nh_.serviceClient<std_srvs::SetBool>("/mission_cmd");
            teach_joy_init = nh_.serviceClient<std_srvs::SetBool>("teach_init");
            teach_joy_finish = nh_.serviceClient<std_srvs::SetBool>("teach_finish");

            gui_state_msg.data = "MAIN_MENU";
            gui_state_pub_.publish(gui_state_msg);

        }

        void EnterTeach(){
            gui_state_msg.data = "TEACH_INIT";
            gui_state_pub_.publish(gui_state_msg);
        }

        void EnterRepeat(){
            gui_state_msg.data = "REPEAT_INIT";
            gui_state_pub_.publish(gui_state_msg);
        }

        void TeachJoyInit(){
            gui_state_msg.data = "TEACH_RUNNING";
            gui_state_pub_.publish(gui_state_msg);

            std_srvs::SetBool teach_cmd;
            teach_cmd.request.data = true;
            teach_joy_init.call(teach_cmd);
        }

        void TeachJoyFinish(){
            gui_state_msg.data = "TEACH_FINISHED";
            gui_state_pub_.publish(gui_state_msg);

            std_srvs::SetBool teach_cmd;
            teach_cmd.request.data = true;
            teach_joy_finish.call(teach_cmd);
        }

        void RepeatGo(){
            gui_state_msg.data = "REPEAT_RUNNING";
            gui_state_pub_.publish(gui_state_msg);
        }

        void RepeatLand(){
            gui_state_msg.data = "REPEAT_LAND";
            gui_state_pub_.publish(gui_state_msg);
        }

        void back2main(){
            gui_state_msg.data  = "MAIN_MENU";
            gui_state_pub_.publish(gui_state_msg);

        }

        void publishNext()
        {
            ROS_DEBUG_STREAM_NAMED("gui", "Next");
            sensor_msgs::Joy msg;
            msg.buttons.resize(9);
            msg.buttons[1] = 1;
            joy_publisher_.publish(msg);

            std_srvs::SetBool cmd_bool;
            cmd_bool.request.data = MAV_CMD_READY;
            joy_client_.call(cmd_bool);
            ROS_INFO("READY command called!");
        }

        void publishContinue()
        {
            ROS_DEBUG_STREAM_NAMED("gui", "Continue");
            sensor_msgs::Joy msg;
            msg.buttons.resize(9);
            msg.buttons[2] = 1;
            joy_publisher_.publish(msg);
        }

        void publishBreak()
        {
            ROS_DEBUG_STREAM_NAMED("gui", "Break (not implemented yet)");

            sensor_msgs::Joy msg;
            msg.buttons.resize(9);
            msg.buttons[3] = 1;
            joy_publisher_.publish(msg);
        }

        void publishStop()
        {
            ROS_DEBUG_STREAM_NAMED("gui", "Stop (not implemented yet)");

            sensor_msgs::Joy msg;
            msg.buttons.resize(9);
            msg.buttons[4] = 1;
            joy_publisher_.publish(msg);
        }

        std_msgs::String check_gui_state(){
            return gui_state_msg;
        }

    protected:
        // The ROS publishers
        ros::Publisher joy_publisher_;
        ros::Publisher gui_state_pub_;

        // The ROS Services
        ros::ServiceClient joy_client_, teach_joy_init, teach_joy_finish;
        ros::ServiceServer joy_server_;

        // The ROS node handle.
        ros::NodeHandle nh_;

        //
        std_msgs::String gui_state_msg;


        //
//    bool mission_callback(std_srvs::SetBool::Request &req,
//                          std_srvs::SetBool::Response &res)
//    {
//        ROS_INFO("mission command received!");
//        return true;
//    }


    };

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
