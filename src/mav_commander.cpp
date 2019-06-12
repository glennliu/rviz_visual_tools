//
// Created by lch on 19-5-12.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <include/rviz_visual_tools/mav_cmd_enum.h>

ros::Publisher vis_pub;
ros::ServiceServer cmd_server;
ros::Timer pub_timer;

void pub_timer_(const ros::TimerEvent &ev)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package:///pr2_description/meshes/base_v0/base.dae";
//    marker.mesh_resource = "file:///home/lch/code_ws/teach_repeat_ws/src/hector_quadrotor/hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
    vis_pub.publish( marker );


}

bool cmd_server_callback_(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res)
{
//    std_srvs::SetBool req;

    ROS_INFO("mission cmd received: %d \n",req.data);
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mav_cmder");
    ros::NodeHandle node_handle("~");

    vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    cmd_server = node_handle.advertiseService("/mission_cmd",cmd_server_callback_);

    pub_timer = node_handle.createTimer(ros::Duration(0.1),pub_timer_);

    ros::spin();
    return 0;
}