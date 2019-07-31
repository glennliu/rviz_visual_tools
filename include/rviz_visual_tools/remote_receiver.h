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
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <decomp_ros_msgs/cmd.h>
#include <rviz_visual_tools/mav_cmd_enum.h>
#include <rviz_visual_tools/statedefine.h>

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
            gui_code_pub_ = nh_.advertise<std_msgs::Int16>("/gui_code",1);

            traj_start_trigger = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger",1);
            flight_cmd_pub = nh_.advertise<std_msgs::Int16>("/flight_cmd",1);
            save_posegraph_pub = nh_.advertise<std_msgs::Int16>("/demo/save_pg_cmd",1);
            drone_state_sub = nh_.subscribe("/demo/state",1,drone_state_cb);

            joy_client_ = nh_.serviceClient<std_srvs::SetBool>("/mission_cmd");
            teach_node_cmd = nh_.serviceClient<decomp_ros_msgs::cmd>("/teach_cmd");
//            teach_joy_init = nh_.serviceClient<std_srvs::SetBool>("/teach_init");
//            teach_joy_finish = nh_.serviceClient<std_srvs::SetBool>("/teach_finish");
            teach_load_path = nh_.serviceClient<std_srvs::SetBool>("/teach_load_file");
//            teach_joy_reset = nh_.serviceClient<std_srvs::SetBool>("/teach_joy_reset");
            joyCtrl_srv = nh_.serviceClient<decomp_ros_msgs::cmd>("/joyCtrl_cmd");
            odom_visual_reset_srv = nh_.serviceClient<std_srvs::SetBool>("/odom_reset");
            surf_fusion_srv = nh_.serviceClient<decomp_ros_msgs::cmd>("/surf_map_cmd");
            map_server_srv = nh_.serviceClient<std_srvs::SetBool>("/map_server_init");
            map_server_save_srv = nh_.serviceClient<std_srvs::SetBool>("/map_server_save");
            airborne_srv = nh_.serviceClient<decomp_ros_msgs::cmd>("/airborne_cmd");

//            repeat_init_switch = nh_.subscribe("/repeat_go_precheck",1,repeat_init_check_callback);


            gui_code_msg.data = GUI_MAIN;
            gui_code_pub_.publish(gui_code_msg);

//            gui_state_msg.data = "MAIN_MENU";
//            gui_state_pub_.publish(gui_state_msg);

        }

        void enterMap(){
//            gui_state_msg.data = "MAP_INIT";
//            gui_state_pub_.publish(gui_state_msg);

            gui_code_msg.data = GUI_MAP_MAIN;
            gui_code_pub_.publish(gui_code_msg);
        }

/*
        void enterAirborne(){
            gui_code_msg.data = GUI_MAP_AIRBORNE;
            gui_code_pub_.publish(gui_code_msg);
        }
        */

        void enterHandheld(){
            gui_code_msg.data = GUI_MAP_HANDHELD;
            gui_code_pub_.publish(gui_code_msg);
        }

        void MapBuilding(){
//            gui_state_msg.data = "MAP_BUILDING";
//            gui_state_pub_.publish(gui_state_msg);
//            gui_code_msg.data = GUI_MAP_HANDHELD;
//            gui_code_pub_.publish(gui_code_msg);

            std_srvs::SetBool map_cmd;
            map_cmd.request.data = true;
            map_server_srv.call(map_cmd);
            decomp_ros_msgs::cmd surf_cmd;
            surf_cmd.request.cmd_code = MAV_CMD_RUN;
            surf_fusion_srv.call(surf_cmd);

        }

        void MapFinished(){
//            gui_state_msg.data = "MAP_FINISHED";
//            gui_state_pub_.publish(gui_state_msg);
//            gui_code_msg.data = GUI_HANDHELD_FINISHED;
//            gui_code_pub_.publish(gui_code_msg);

            std_srvs::SetBool map_cmd;
            std_msgs::Int16 save_pose_msg;
            map_cmd.request.data = true;
            map_server_save_srv.call(map_cmd);
            decomp_ros_msgs::cmd surf_cmd;
            surf_cmd.request.cmd_code = MAV_CMD_FINISH;
            surf_fusion_srv.call(surf_cmd);
            save_pose_msg.data = SAVE_POSE_GRAPH;
            save_posegraph_pub.publish(save_pose_msg);
        }

        void EnterTeach(){
            gui_state_msg.data = "TEACH_INIT";
            gui_state_pub_.publish(gui_state_msg);

            gui_code_msg.data = GUI_TEACH;
            gui_code_pub_.publish(gui_code_msg);
        }

        void EnterRepeat(){
            gui_state_msg.data = "REPEAT_INIT";
            gui_state_pub_.publish(gui_state_msg);
            gui_code_msg.data = GUI_REPEAT;
            gui_code_pub_.publish(gui_code_msg);

            decomp_ros_msgs::cmd joy_cmd;
            joy_cmd.request.cmd_code = MAV_CMD_INIT;
            joyCtrl_srv.call(joy_cmd);

        }

        void TeachLoadPath(){
            gui_state_msg.data = "TEACH_LOAD_FILE";
            gui_state_pub_.publish(gui_state_msg);

            std_srvs::SetBool teach_cmd;
            teach_cmd.request.data = true;
            teach_load_path.call(teach_cmd);
        }

        void TeachJoyInit(){
            gui_state_msg.data = "TEACH_RUNNING";
            gui_state_pub_.publish(gui_state_msg);

            decomp_ros_msgs::cmd joy_cmd;
            joy_cmd.request.cmd_code = MAV_CMD_INIT;
            joyCtrl_srv.call(joy_cmd);
            teach_node_cmd.call(joy_cmd);
        }

        void TeachJoyFinish(){
            gui_state_msg.data = "TEACH_FINISHED";
            gui_state_pub_.publish(gui_state_msg);

            decomp_ros_msgs::cmd joy_cmd;
            joy_cmd.request.cmd_code = MAV_CMD_FINISH;
            joyCtrl_srv.call(joy_cmd);
            teach_node_cmd.call(joy_cmd);
        }

        void TeachJoyReset(){
            gui_state_msg.data = "TEACH_INIT";
            gui_state_pub_.publish(gui_state_msg);

            std_srvs::SetBool teach_reset_cmd;
            decomp_ros_msgs::cmd joy_cmd_msg;

            teach_reset_cmd.request.data = true;
            joy_cmd_msg.request.cmd_code = MAV_CMD_RESET;

            odom_visual_reset_srv.call(teach_reset_cmd);
            joyCtrl_srv.call(joy_cmd_msg);
            teach_node_cmd.call(joy_cmd_msg);
        }

        void RepeatGo(){
            gui_state_msg.data = "REPEAT_RUNNING";
            gui_state_pub_.publish(gui_state_msg);

            geometry_msgs::PoseStamped pose_msg;
            std_msgs::Int16 flight_cmd_msg;
            pose_msg.header.stamp = ros::Time::now();
            flight_cmd_msg.data = REPEAT_BEGIN;
            flight_cmd_pub.publish(flight_cmd_msg);
            traj_start_trigger.publish(pose_msg);
        }

        void RepeatLand(){
            gui_state_msg.data = "REPEAT_LAND";
            gui_state_pub_.publish(gui_state_msg);
        }

        void RepeatReset(){
            gui_state_msg.data = "REPEAT_INIT";
            gui_state_pub_.publish(gui_state_msg);
            decomp_ros_msgs::cmd teach_cmd_msg;

            teach_cmd_msg.request.cmd_code = MAV_CMD_RESET;
            teach_node_cmd.call(teach_cmd_msg);
        }

        void EnterAirborne(){
            gui_state_msg.data = "AIRBORNE";
            gui_state_pub_.publish(gui_state_msg);

/*
            std_srvs::SetBool map_cmd;
            map_cmd.request.data = true;
            map_server_srv.call(map_cmd);
            decomp_ros_msgs::cmd surf_cmd;
            surf_cmd.request.cmd_code = MAV_CMD_RUN;
            surf_fusion_srv.call(surf_cmd);
            */

            /*
            std_srvs::SetBool map_cmd;
            map_cmd.request.data = true;
            map_server_srv.call(map_cmd);
            decomp_ros_msgs::cmd surf_cmd;
            surf_cmd.request.cmd_code = MAV_CMD_RUN;
            surf_fusion_srv.call(surf_cmd);
            */
        }

        void airborneTakeoff(){
            std_msgs::Int16 cmd;
            cmd.data = MAP_BEGIN;
            flight_cmd_pub.publish(cmd);
        }

        void airborneMarker(){
            std_msgs::Int16 cmd;
            cmd.data = WAYPOINT_MAPPING;
            gui_state_msg.data = "AIRBORNE_MARKER";
            decomp_ros_msgs::cmd cmd_srv;
            decomp_ros_msgs::cmd joy_srv;
            cmd_srv.request.cmd_code = MAV_CMD_INIT;
            joy_srv.request.cmd_code = MAV_CMD_FINISH;
            airborne_srv.call(cmd_srv);
            joyCtrl_srv.call(joy_srv);
            gui_state_pub_.publish(gui_state_msg);
            flight_cmd_pub.publish(cmd);
        }

        void airborneJoy(){
            std_msgs::Int16 cmd;
            cmd.data = JOYSTICK_MAPPING;
            gui_state_msg.data = "AIRBORNE_JOY";
            decomp_ros_msgs::cmd marker_srv;
//            decomp_ros_msgs::cmd joy_srv;
            marker_srv.request.cmd_code = MAV_CMD_PAUSE;
//            joy_srv.request.cmd_code = MAV_CMD_INIT;
            airborne_srv.call(marker_srv);
//            joyCtrl_srv.call(joy_srv);
            gui_state_pub_.publish(gui_state_msg);
            flight_cmd_pub.publish(cmd);
        }

        void airborneFinished(){
            decomp_ros_msgs::cmd cmd_srv;
            std_msgs::Int16 cmd;
            cmd_srv.request.cmd_code = MAV_CMD_FINISH;
            cmd.data = FINISH;
            flight_cmd_pub.publish(cmd);
            airborne_srv.call(cmd_srv);
            joyCtrl_srv.call(cmd_srv);


/*
            std_srvs::SetBool map_cmd;
            map_cmd.request.data = true;
            map_server_save_srv.call(map_cmd);
            decomp_ros_msgs::cmd surf_cmd;
            surf_cmd.request.cmd_code = MAV_CMD_FINISH;
            surf_fusion_srv.call(surf_cmd);
            */
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

//            std_srvs::SetBool cmd_bool;
//            cmd_bool.request.data = MAV_CMD_READY;
//            joy_client_.call(cmd_bool);
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

        std_msgs::Int16 check_gui_code(){
            return gui_code_msg;
        }

//        std_msgs::Int16 check_repeat_flag(){
//            return repeat_flag_msg;
//        }

        static void drone_state_cb(const std_msgs::Int16 &msg){
//            drone_state = msg.data;
        }

        static void repeat_init_check_callback(const std_msgs::Int16 &msg){
            std_msgs::Int16 repeat_flag_msg;
            repeat_flag_msg.data = msg.data;

            ROS_INFO("test!");
        }

    protected:
        // The ROS publishers
        ros::Publisher joy_publisher_;
        ros::Publisher gui_state_pub_,gui_code_pub_;
        ros::Publisher traj_start_trigger, flight_cmd_pub,
                save_posegraph_pub;

        // the ROS subscriber
        ros::Subscriber repeat_init_switch,drone_state_sub;

        // The ROS Services
        ros::ServiceClient joy_client_, teach_load_path, teach_node_cmd,
                odom_visual_reset_srv, joyNode_reset, joyCtrl_srv,
                surf_fusion_srv, map_server_srv, map_server_save_srv,
                airborne_srv;
        ros::ServiceServer joy_server_;

        // The ROS node handle.
        ros::NodeHandle nh_;

        //
        std_msgs::String gui_state_msg;
        std_msgs::Int16 gui_code_msg;
        int16_t repeat_check_flag;
        int16_t drone_state;

        //



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
