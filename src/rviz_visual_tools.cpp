/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>, Andy McEvoy
   Desc:   Helper functions for displaying basic shape markers in Rviz
*/

#include <rviz_visual_tools/rviz_visual_tools.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace rviz_visual_tools
{

RvizVisualTools::RvizVisualTools(const std::string& base_frame,
                                 const std::string& marker_topic)
  :  nh_("~")
  ,  marker_topic_(marker_topic)
  ,  base_frame_(base_frame)
  ,  batch_publishing_enabled_(false)
{
  initialize();
}

void RvizVisualTools::initialize()
{
  floor_to_base_height_ = 0;
  marker_lifetime_ = ros::Duration(0.0); // 0 - unlimited
  alpha_ = 0.8;
  global_scale_ = 1.0;
  // Cache the reusable markers
  loadRvizMarkers();
}

bool RvizVisualTools::deleteAllMarkers()
{
  // Helper for publishing rviz markers
  return publishMarker( reset_marker_ );
}

void RvizVisualTools::resetMarkerCounts()
{
  arrow_marker_.id++;
  sphere_marker_.id++;
  block_marker_.id++;
  cylinder_marker_.id++;
  mesh_marker_.id++;
  text_marker_.id++;
  cuboid_marker_.id++;
  line_marker_.id++;
  line_list_marker_.id++;
  spheres_marker_.id++;
}

bool RvizVisualTools::loadRvizMarkers()
{
  // Load reset marker -------------------------------------------------
  reset_marker_.header.frame_id = base_frame_;
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.action = 3; // In ROS-J: visualization_msgs::Marker::DELETEALL;

  // Load arrow ----------------------------------------------------

  arrow_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  arrow_marker_.ns = "Arrow";
  // Set the marker type.
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  // Set the marker action.  Options are ADD and DELETE
  arrow_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  arrow_marker_.lifetime = marker_lifetime_;

  // Load cuboid ----------------------------------------------------

  cuboid_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  cuboid_marker_.ns = "Cuboid";
  // Set the marker type.
  cuboid_marker_.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  cuboid_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  cuboid_marker_.lifetime = marker_lifetime_;

  // Load line ----------------------------------------------------

  line_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  line_marker_.ns = "Line";
  // Set the marker type.
  line_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // Set the marker action.  Options are ADD and DELETE
  line_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_marker_.lifetime = marker_lifetime_;

  // Load path ----------------------------------------------------

  line_list_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  line_list_marker_.ns = "Line_List";
  // Set the marker type.
  line_list_marker_.type = visualization_msgs::Marker::LINE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  line_list_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_list_marker_.lifetime = marker_lifetime_;
  // Constants
  line_list_marker_.pose.position.x = 0.0;
  line_list_marker_.pose.position.y = 0.0;
  line_list_marker_.pose.position.z = 0.0;

  line_list_marker_.pose.orientation.x = 0.0;
  line_list_marker_.pose.orientation.y = 0.0;
  line_list_marker_.pose.orientation.z = 0.0;
  line_list_marker_.pose.orientation.w = 1.0;

  // Load sphers ----------------------------------------------------

  spheres_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  spheres_marker_.ns = "Spheres";
  // Set the marker type.
  spheres_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  spheres_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  spheres_marker_.lifetime = marker_lifetime_;
  // Constants
  spheres_marker_.pose.position.x = 0.0;
  spheres_marker_.pose.position.y = 0.0;
  spheres_marker_.pose.position.z = 0.0;

  spheres_marker_.pose.orientation.x = 0.0;
  spheres_marker_.pose.orientation.y = 0.0;
  spheres_marker_.pose.orientation.z = 0.0;
  spheres_marker_.pose.orientation.w = 1.0;

  // Load Block ----------------------------------------------------
  block_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  block_marker_.ns = "Block";
  // Set the marker action.  Options are ADD and DELETE
  block_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  block_marker_.type = visualization_msgs::Marker::CUBE;
  // Lifetime
  block_marker_.lifetime = marker_lifetime_;

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = base_frame_;
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;

  // Load Mesh ----------------------------------------------------
  mesh_marker_.header.frame_id = base_frame_;

  // Set the marker action.  Options are ADD and DELETE
  mesh_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  mesh_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Lifetime
  mesh_marker_.lifetime = marker_lifetime_;

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  sphere_marker_.ns = "Sphere";
  // Set the marker type.
  sphere_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  sphere_marker_.action = visualization_msgs::Marker::ADD;
  // Marker group position and orientation
  sphere_marker_.pose.position.x = 0;
  sphere_marker_.pose.position.y = 0;
  sphere_marker_.pose.position.z = 0;
  sphere_marker_.pose.orientation.x = 0.0;
  sphere_marker_.pose.orientation.y = 0.0;
  sphere_marker_.pose.orientation.z = 0.0;
  sphere_marker_.pose.orientation.w = 1.0;
  // Create a sphere point
  geometry_msgs::Point point_a;
  // Add the point pair to the line message
  sphere_marker_.points.push_back( point_a );
  sphere_marker_.colors.push_back( getColor( BLUE ) );
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;

  // Load Text ----------------------------------------------------
  // Set the namespace and id for this marker.  This serves to create a unique ID
  text_marker_.ns = "Text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // Lifetime
  text_marker_.lifetime = marker_lifetime_;

  return true;
}

void RvizVisualTools::loadMarkerPub()
{
  if (pub_rviz_markers_)
    return;

  // Rviz marker publisher
  pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing Rviz markers on topic " << pub_rviz_markers_.getTopic());

  waitForSubscriber(pub_rviz_markers_);
}

bool RvizVisualTools::waitForSubscriber(const ros::Publisher &pub, const double &wait_time)
{
  // Benchmark runtime
  ros::Time start_time;
  start_time = ros::Time::now();

  // Will wait at most 1000 ms (1 sec)
  ros::Time maxTime(ros::Time::now() + ros::Duration(wait_time));

  // This is wrong. It returns only the number of subscribers that have already established their direct connections to this publisher
  int num_existing_subscribers = pub.getNumSubscribers();

  // How often to check for subscribers
  ros::Rate poll_rate(200);

  // Wait for subsriber
  while(num_existing_subscribers == 0)
  {
    // Check if timed out
    if (ros::Time::now() > maxTime)
    {
      ROS_WARN_STREAM_NAMED("visual_tools", "Topic '" << pub.getTopic() << "' unable to connect to any subscribers within "
                            << wait_time << " seconds. It is possible initially published visual messages will be lost.");
      return false;
    }
    ros::spinOnce();

    // Sleep
    poll_rate.sleep();

    // Check again
    num_existing_subscribers = pub.getNumSubscribers();
    //std::cout << "num_existing_subscribers " << num_existing_subscribers << std::endl;
  }

  // Benchmark runtime
  if (false)
  {
    double duration = (ros::Time::now() - start_time).toSec();
    ROS_DEBUG_STREAM_NAMED("visual_tools", "Topic '" << pub.getTopic() << "' took " << duration
                           << " seconds to connect to a subscriber. Connected to " << num_existing_subscribers
                           << " total subsribers");
  }
  return true;
}

void RvizVisualTools::setFloorToBaseHeight(double floor_to_base_height)
{
  floor_to_base_height_ = floor_to_base_height;
}

void RvizVisualTools::setLifetime(double lifetime)
{
  marker_lifetime_ = ros::Duration(lifetime);

  // Update cached markers
  arrow_marker_.lifetime = marker_lifetime_;
  cuboid_marker_.lifetime = marker_lifetime_;
  line_marker_.lifetime = marker_lifetime_;
  sphere_marker_.lifetime = marker_lifetime_;
  block_marker_.lifetime = marker_lifetime_;
  mesh_marker_.lifetime = marker_lifetime_;
  cylinder_marker_.lifetime = marker_lifetime_;
  text_marker_.lifetime = marker_lifetime_;
}

const rviz_visual_tools::colors RvizVisualTools::getRandColor()
{
  std::vector<rviz_visual_tools::colors> all_colors;

  all_colors.push_back(RED);
  all_colors.push_back(GREEN);
  all_colors.push_back(BLUE);
  all_colors.push_back(GREY);
  all_colors.push_back(DARK_GREY);
  all_colors.push_back(WHITE);
  all_colors.push_back(ORANGE);
  //all_colors.push_back(BLACK);
  all_colors.push_back(YELLOW);
  all_colors.push_back(BROWN);
  all_colors.push_back(PINK);
  all_colors.push_back(LIME_GREEN);
  all_colors.push_back(PURPLE);
  all_colors.push_back(CYAN);
  all_colors.push_back(MAGENTA);

  int rand_num = iRand(0, all_colors.size() - 1);
  return all_colors[ rand_num ];
}

std_msgs::ColorRGBA RvizVisualTools::getColor(const rviz_visual_tools::colors &color)
{
  std_msgs::ColorRGBA result;
  result.a = alpha_;
  switch(color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = 1.0;
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      result.a = 1.0;
    case DARK_GREY:
      result.r = 0.6;
      result.g = 0.6;
      result.b = 0.6;
      result.a = 1.0;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case TRANSLUCENT_LIGHT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.1;
      break;
    case TRANSLUCENT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.25;
      break;
    case TRANSLUCENT_DARK:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.5;
      break;
    case BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = 1.0;
    case BROWN:
      result.r = 0.597;
      result.g = 0.296;
      result.b = 0.0;
      result.a = 1.0;
      break;
    case PINK:
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = 1.0;
      break;
    case LIME_GREEN:
      result.r = 0.6;
      result.g = 1.0;
      result.b = 0.2;
      result.a = 1.0;
      break;
    case CLEAR:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 0.0;
      break;
    case PURPLE:
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = 1.0;
      break;
    case CYAN:
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case MAGENTA:
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = 1.0;
      break;
    case RAND:
      // Make sure color is not *too* light
      do
      {
        result.r = fRand(0.0,1.0);
        result.g = fRand(0.0,1.0);
        result.b = fRand(0.0,1.0);
        result.a = 1.0;
      } while (result.r + result.g + result.b < 1.5); // 3 would be white
      break;
    case DEFAULT:
      ROS_WARN_STREAM_NAMED("getColor","The 'DEFAULT' color should probably not be used with getColor(). Defaulting to blue.");
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = 1.0;
  }

  return result;
}

geometry_msgs::Vector3 RvizVisualTools::getScale(const rviz_visual_tools::scales &scale, bool arrow_scale, double marker_scale)
{
  geometry_msgs::Vector3 result;
  double val(0.0);
  switch(scale)
  {
    case XXSMALL:
      val = 0.005;
      break;
    case XSMALL:
      val = 0.01;
      break;
    case SMALL:
      val = 0.03;
      break;
    case REGULAR:
      val = 0.05;
      break;
    case LARGE:
      val = 0.1;
      break;
    case xLARGE:
      val = 0.2;
      break;
    case xxLARGE:
      val = 0.3;
      break;
    case xxxLARGE:
      val = 0.4;
      break;
    case XLARGE:
      val = 0.5;
      break;
    case XXLARGE:
      val = 1.0;
      break;
    default:
      ROS_ERROR_STREAM_NAMED("visualization_tools","Not implemented yet");
      break;
  }

  // Allows an individual marker size factor and a size factor for all markers
  result.x = val * marker_scale * global_scale_;
  result.y = val * marker_scale * global_scale_;
  result.z = val * marker_scale * global_scale_;

  // The y and z scaling is smaller for arrows
  if (arrow_scale)
  {
    result.y *= 0.1;
    result.z *= 0.1;
  }

  return result;
}

Eigen::Vector3d RvizVisualTools::getCenterPoint(Eigen::Vector3d a, Eigen::Vector3d b)
{
  Eigen::Vector3d center;
  center[0] = (a[0] + b[0]) / 2;
  center[1] = (a[1] + b[1]) / 2;
  center[2] = (a[2] + b[2]) / 2;
  return center;
}

Eigen::Affine3d RvizVisualTools::getVectorBetweenPoints(Eigen::Vector3d a, Eigen::Vector3d b)
{
  // from http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

  // Goal pose:
  Eigen::Quaterniond q;

  Eigen::Vector3d axis_vector = b - a;
  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0*acos(theta);

  //-------------------------------------------
  // Method 1 - TF - works
  //Convert to TF
  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  //-------------------------------------------
  // Method 2 - Eigen - broken TODO
  //q = Eigen::AngleAxis<double>(angle_rotation, right_axis_vector);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  q.normalize();
  pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
  pose.translation() = a;

  return pose;
}

bool RvizVisualTools::publishMarker(const visualization_msgs::Marker &marker)
{
  // Add single marker to array
  markers_.markers.push_back(marker);

  // Determine if we should publish now
  if (!batch_publishing_enabled_)
  {
    return triggerBatchPublish();
  }

  return true;
}

void RvizVisualTools::enableBatchPublishing(bool enable)
{
  batch_publishing_enabled_ = enable;
  //ROS_ERROR_STREAM_NAMED("temp","BATCH PUBLISHING ENABLED = " << enable);
}

bool RvizVisualTools::triggerBatchPublish()
{
  bool result = publishMarkers( markers_ );

  markers_.markers.clear(); // remove all cached markers
  return result;
}

bool RvizVisualTools::triggerBatchPublishAndDisable()
{
  triggerBatchPublish();
  batch_publishing_enabled_ = false;
  //ROS_ERROR_STREAM_NAMED("temp","BATCH PUBLISHING DISABLED");
}

bool RvizVisualTools::publishMarkers(const visualization_msgs::MarkerArray &markers)
{
  if (!pub_rviz_markers_) // always check this before publishing
    loadMarkerPub();

  pub_rviz_markers_.publish( markers );
  ros::spinOnce();
  return true;
}

bool RvizVisualTools::publishSphere(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  return publishSphere(convertPose(pose), color, scale, ns);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_visual_tools::colors &color, const double scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Point &point, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  return publishSphere(pose_msg, color, scale, ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  return publishSphere(pose, color, getScale(scale, false, 0.1), ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, double scale, const std::string& ns)
{
  geometry_msgs::Vector3 scale_msg;
  scale_msg.x = scale;
  scale_msg.y = scale;
  scale_msg.z = scale;
  return publishSphere(pose, color, scale_msg, ns);
}

bool RvizVisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, const geometry_msgs::Vector3 scale, const std::string& ns)
{
  // Set the frame ID and timestamp
  sphere_marker_.header.stamp = ros::Time::now();

  sphere_marker_.id++;
  sphere_marker_.color = getColor(color);
  sphere_marker_.scale = scale;
  sphere_marker_.ns = ns;

  // Update the single point with new pose
  sphere_marker_.points[0] = pose.position;
  sphere_marker_.colors[0] = getColor(color);

  // Helper for publishing rviz markers
  return publishMarker( sphere_marker_ );
}

bool RvizVisualTools::publishXArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(convertPose(pose), color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishYArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                   const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(convertPose(pose), color, scale, length);
}

bool RvizVisualTools::publishArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                   const rviz_visual_tools::scales &scale, double length)
{
  // Set the frame ID and timestamp.  
  arrow_marker_.header.stamp = ros::Time::now();

  arrow_marker_.id++;
  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);
  arrow_marker_.scale.x = length; // overrides previous x scale specified

  // Helper for publishing rviz markers
  return publishMarker( arrow_marker_ );
}

bool RvizVisualTools::publishArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                   const rviz_visual_tools::scales &scale, double length)
{
  // Set the frame ID and timestamp.  
  arrow_marker_.header = pose.header;

  arrow_marker_.id++;
  arrow_marker_.pose = pose.pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);
  arrow_marker_.scale.x = length; // overrides previous x scale specified

  // Helper for publishing rviz markers
  return publishMarker( arrow_marker_ );
}

bool RvizVisualTools::publishBlock(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, const double &block_size)
{
  // Set the timestamp
  block_marker_.header.stamp = ros::Time::now();

  block_marker_.id++;

  // Set the pose
  block_marker_.pose = pose;

  // Set marker size
  block_marker_.scale.x = block_size;
  block_marker_.scale.y = block_size;
  block_marker_.scale.z = block_size;

  // Set marker color
  block_marker_.color = getColor( color );

  // Helper for publishing rviz markers
  return publishMarker( block_marker_ );
}

bool RvizVisualTools::publishBlock(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color, const double &block_size)
{
  return publishBlock(convertPose(pose), color, block_size);
}

bool RvizVisualTools::publishAxisWithLabel(const Eigen::Affine3d &pose, const std::string& label, 
                                           const rviz_visual_tools::scales &scale)
{
  publishText(pose, label, rviz_visual_tools::BLACK, rviz_visual_tools::SMALL, false);  // TODO: change size based on passed in scale
  publishAxis(pose, 0.1, 0.01, label);
  return true;
}

bool RvizVisualTools::publishAxis(const geometry_msgs::Pose &pose, double length, double radius, const std::string& ns)
{
  return publishAxis(convertPose(pose), length, radius, ns);
}

bool RvizVisualTools::publishAxis(const Eigen::Affine3d &pose, double length, double radius, const std::string& ns)
{
  // Batch publish, unless it is already enabled by user
  bool manual_batch_publish = false;
  if (!batch_publishing_enabled_)
  {
    enableBatchPublishing(true);
    manual_batch_publish = true;
  }
    
  // Publish x axis
  Eigen::Affine3d x_pose = Eigen::Translation3d(length / 2.0, 0, 0) *
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  x_pose = pose * x_pose;
  publishCylinder(x_pose, rviz_visual_tools::RED, length, radius, ns);

  // Publish y axis
  Eigen::Affine3d y_pose = Eigen::Translation3d(0, length / 2.0, 0) *
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
  y_pose = pose * y_pose;
  publishCylinder(y_pose, rviz_visual_tools::GREEN, length, radius, ns);

  // Publish z axis
  Eigen::Affine3d z_pose = Eigen::Translation3d(0, 0, length / 2.0) *
    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  z_pose = pose * z_pose;
  publishCylinder(z_pose, rviz_visual_tools::BLUE, length, radius, ns);

  // Batch publish
  if (manual_batch_publish)
    triggerBatchPublishAndDisable();

  return true;
}

bool RvizVisualTools::publishCylinder(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color, double height, 
                                      double radius, const std::string& ns)
{
  return publishCylinder(convertPose(pose), color, height, radius, ns);
}

bool RvizVisualTools::publishCylinder(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, double height, 
                                      double radius, const std::string& ns)
{
  // Set the timestamp
  cylinder_marker_.header.stamp = ros::Time::now();
  cylinder_marker_.ns = ns;
  cylinder_marker_.id++;

  // Set the pose
  cylinder_marker_.pose = pose;

  // Set marker size
  cylinder_marker_.scale.x = radius;
  cylinder_marker_.scale.y = radius;
  cylinder_marker_.scale.z = height;

  // Set marker color
  cylinder_marker_.color = getColor( color );

  // Helper for publishing rviz markers
  return publishMarker( cylinder_marker_ );
}

bool RvizVisualTools::publishMesh(const Eigen::Affine3d &pose, const std::string& file_name, const rviz_visual_tools::colors &color,
                                  double scale, const std::string &ns, const std::size_t &id)
{
  return publishMesh(convertPose(pose), file_name, color, scale, ns, id);
}

bool RvizVisualTools::publishMesh(const geometry_msgs::Pose &pose, const std::string& file_name, const rviz_visual_tools::colors &color,
                                  double scale, const std::string &ns, const std::size_t &id)
{
  // Set the timestamp
  mesh_marker_.header.stamp = ros::Time::now();

  if (id == 0)
    mesh_marker_.id++;
  else
    mesh_marker_.id = id;

  // Set the mesh
  mesh_marker_.mesh_resource = file_name;
  mesh_marker_.mesh_use_embedded_materials = true;

  // Set the pose
  mesh_marker_.pose = pose;

  // Set marker size
  mesh_marker_.scale.x = scale;
  mesh_marker_.scale.y = scale;
  mesh_marker_.scale.z = scale;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  mesh_marker_.ns = ns;

  // Set marker color
  mesh_marker_.color = getColor( color );

  // Helper for publishing rviz markers
  return publishMarker( mesh_marker_ );
}

bool RvizVisualTools::publishGraph(const graph_msgs::GeometryGraph &graph, const rviz_visual_tools::colors &color, double radius)
{
  // Track which pairs of nodes we've already connected since graph is bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator,bool> return_value;

  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert( node_ids(i,j) );
      if (return_value.second == false)
      {
        // Element already existed in set, so don't add a new collision object
      }
      else
      {
        // Create a cylinder from two points
        a = convertPoint(graph.nodes[i]);
        b = convertPoint(graph.nodes[graph.edges[i].node_ids[j]]);

        // add other direction of edge
        added_edges.insert( node_ids(j,i) );

        // Distance between two points
        double height = (a - b).lpNorm<2>();

        // Find center point
        Eigen::Vector3d pt_center = getCenterPoint(a, b);

        // Create vector
        Eigen::Affine3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Affine3d rotation;
        rotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Publish individually
        publishCylinder(convertPose(pose), color, height, radius);
      }
    }
  }

  return true;
}

bool RvizVisualTools::publishCuboid(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                    const rviz_visual_tools::colors &color)
{
  return publishCuboid(convertPoint(point1), convertPoint(point2), color);
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                    const rviz_visual_tools::colors &color)
{
  // Set the timestamp
  cuboid_marker_.header.stamp = ros::Time::now();

  cuboid_marker_.id++;
  cuboid_marker_.color = getColor(color);

  // Calculate center pose
  geometry_msgs::Pose pose;
  pose.position.x = (point1.x - point2.x) / 2.0 + point2.x;
  pose.position.y = (point1.y - point2.y) / 2.0 + point2.y;
  pose.position.z = (point1.z - point2.z) / 2.0 + point2.z;
  cuboid_marker_.pose = pose;

  // Calculate scale
  cuboid_marker_.scale.x = fabs(point1.x - point2.x);
  cuboid_marker_.scale.y = fabs(point1.y - point2.y);
  cuboid_marker_.scale.z = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!cuboid_marker_.scale.x) cuboid_marker_.scale.x = SMALL_SCALE;
  if (!cuboid_marker_.scale.y) cuboid_marker_.scale.y = SMALL_SCALE;
  if (!cuboid_marker_.scale.z) cuboid_marker_.scale.z = SMALL_SCALE;

  // Helper for publishing rviz markers
  return publishMarker( cuboid_marker_ );
}

bool RvizVisualTools::publishCuboid(const geometry_msgs::Pose &pose, const double depth, const double width,
                                    const double height, const rviz_visual_tools::colors &color)
{
  cuboid_marker_.header.stamp = ros::Time::now();

  cuboid_marker_.id++;
  cuboid_marker_.color = getColor(color);

  cuboid_marker_.pose = pose;

  // Prevent scale from being zero
  if (depth <= 0)
    cuboid_marker_.scale.x = SMALL_SCALE;
  else
    cuboid_marker_.scale.x = depth;

  if (width <= 0)
    cuboid_marker_.scale.y = SMALL_SCALE;
  else
    cuboid_marker_.scale.y = width;

  if (height <= 0)
    cuboid_marker_.scale.z = SMALL_SCALE;
  else
    cuboid_marker_.scale.z = height;

  return publishMarker( cuboid_marker_ );
}

bool RvizVisualTools::publishLine(const Eigen::Affine3d &point1, const Eigen::Affine3d &point2,
                                  const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale)
{
  return publishLine( convertPoseToPoint(point1), convertPoseToPoint(point2), color, scale );
}

bool RvizVisualTools::publishLine(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2,
                                  const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale)
{
  return publishLine (RvizVisualTools::convertPoint(point1), RvizVisualTools::convertPoint(point2), color, scale);
}

bool RvizVisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                  const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale)
{
  // Set the timestamp
  line_marker_.header.stamp = ros::Time::now();

  line_marker_.id++;
  line_marker_.color = getColor(color);
  line_marker_.scale = getScale( scale, false, 0.1 );

  line_marker_.points.clear();
  line_marker_.points.push_back(point1);
  line_marker_.points.push_back(point2);

  // Helper for publishing rviz markers
  return publishMarker( line_marker_ );
}

bool RvizVisualTools::publishPath(const std::vector<geometry_msgs::Point> &path, const rviz_visual_tools::colors &color,
                                  const rviz_visual_tools::scales &scale, const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM_NAMED("publishPath","Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = ns;

  // Provide a new id every call to this function
  line_list_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor( color );
  line_list_marker_.scale = getScale(scale, false, 0.25);
  line_list_marker_.color = this_color;
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();

  // Convert path coordinates
  for( std::size_t i = 1; i < path.size(); ++i )
  {
    // Add the point pair to the line message
    line_list_marker_.points.push_back( path[i-1] );
    line_list_marker_.points.push_back( path[i] );
    line_list_marker_.colors.push_back( this_color );
    line_list_marker_.colors.push_back( this_color );
  }

  // Helper for publishing rviz markers
  return publishMarker( line_list_marker_ );
}

bool RvizVisualTools::publishPolygon(const geometry_msgs::Polygon &polygon, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point temp;
  geometry_msgs::Point first; // remember first point because we will connect first and last points for last line
  for (std::size_t i = 0; i < polygon.points.size(); ++i)
  {
    temp.x = polygon.points[i].x;
    temp.y = polygon.points[i].y;
    temp.z = polygon.points[i].z;
    if (i == 0)
      first = temp;
    points.push_back(temp);
  }
  points.push_back(first); // connect first and last points for last line

  publishPath(points, color, scale, ns);
}

  bool RvizVisualTools::publishWireframeCuboid(const Eigen::Affine3d &pose,
                                               double depth,
                                               double width,
                                               double height,
                                               const rviz_visual_tools::colors &color)
  {
    Eigen::Vector3d min_point, max_point;
    min_point << -depth/2, -width/2, -height/2;
    max_point << depth/2, width/2, height/2;
    return publishWireframeCuboid(pose, min_point, max_point, color);
  }

bool RvizVisualTools::publishWireframeCuboid(const Eigen::Affine3d &pose,
                                             const Eigen::Vector3d &min_point,
                                             const Eigen::Vector3d &max_point,
                                             const rviz_visual_tools::colors &color)
{
  // Extract 8 cuboid vertices
  Eigen::Vector3d p1 (min_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p2 (min_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p3 (max_point[0], min_point[1], max_point[2]);
  Eigen::Vector3d p4 (max_point[0], min_point[1], min_point[2]);
  Eigen::Vector3d p5 (min_point[0], max_point[1], min_point[2]);
  Eigen::Vector3d p6 (min_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p7 (max_point[0], max_point[1], max_point[2]);
  Eigen::Vector3d p8 (max_point[0], max_point[1], min_point[2]);

  p1 = pose * p1;
  p2 = pose * p2;
  p3 = pose * p3;
  p4 = pose * p4;
  p5 = pose * p5;
  p6 = pose * p6;
  p7 = pose * p7;
  p8 = pose * p8;

  RvizVisualTools::publishLine(p1, p2, color);
  RvizVisualTools::publishLine(p1, p4, color);
  RvizVisualTools::publishLine(p1, p5, color);
  RvizVisualTools::publishLine(p5, p6, color);
  RvizVisualTools::publishLine(p5, p8, color);
  RvizVisualTools::publishLine(p2, p6, color);
  RvizVisualTools::publishLine(p6, p7, color);
  RvizVisualTools::publishLine(p7, p8, color);
  RvizVisualTools::publishLine(p2, p3, color);
  RvizVisualTools::publishLine(p4, p8, color);
  RvizVisualTools::publishLine(p3, p4, color);
  RvizVisualTools::publishLine(p3, p7, color);

  return true;
}

bool RvizVisualTools::publishWireframeRectangle(const Eigen::Affine3d &pose, const double& height, const double& width,
                                                const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale)
{
  // Extract 8 cuboid vertices
  Eigen::Vector3d p1 (-width/2.0, -height/2.0, 0.0);
  Eigen::Vector3d p2 (-width/2.0,  height/2.0, 0.0);
  Eigen::Vector3d p3 (width/2.0,   height/2.0, 0.0);
  Eigen::Vector3d p4 (width/2.0,  -height/2.0, 0.0);

  p1 = pose * p1;
  p2 = pose * p2;
  p3 = pose * p3;
  p4 = pose * p4;

  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = "Wireframe Rectangle";

  // Provide a new id every call to this function
  line_list_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor( color );
  line_list_marker_.scale = getScale(scale, false, 0.25);
  line_list_marker_.color = this_color;
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();

  // Add each point pair to the line message
  line_list_marker_.points.push_back( convertPoint(p1) );
  line_list_marker_.points.push_back( convertPoint(p2) );
  line_list_marker_.colors.push_back( this_color );
  line_list_marker_.colors.push_back( this_color );

  line_list_marker_.points.push_back( convertPoint(p2) );
  line_list_marker_.points.push_back( convertPoint(p3) );
  line_list_marker_.colors.push_back( this_color );
  line_list_marker_.colors.push_back( this_color );

  line_list_marker_.points.push_back( convertPoint(p3) );
  line_list_marker_.points.push_back( convertPoint(p4) );
  line_list_marker_.colors.push_back( this_color );
  line_list_marker_.colors.push_back( this_color );

  line_list_marker_.points.push_back( convertPoint(p4) );
  line_list_marker_.points.push_back( convertPoint(p1) );
  line_list_marker_.colors.push_back( this_color );
  line_list_marker_.colors.push_back( this_color );

  // Helper for publishing rviz markers
  return publishMarker( line_list_marker_ );
}

bool RvizVisualTools::publishSpheres(const std::vector<Eigen::Vector3d> &points, const rviz_visual_tools::colors &color, const double scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points_msg;
  //geometry_msgs::Point temp;

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    //tf::pointEigenToMsg(points[i], temp);
    points_msg.push_back(convertPoint(points[i]));
  }


  return publishSpheres(points_msg, color, scale, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors &color, const double scale, const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  publishSpheres( points, color, scale_vector, ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, const std::string& ns)
{
  publishSpheres( points, color, getScale(scale, false, 0.25), ns);
}

bool RvizVisualTools::publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_visual_tools::colors &color, const geometry_msgs::Vector3 &scale, const std::string& ns)
{
  spheres_marker_.header.stamp = ros::Time();
  spheres_marker_.ns = ns;

  // Provide a new id every call to this function
  spheres_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor( color );
  spheres_marker_.scale = scale;
  spheres_marker_.color = this_color;
  //spheres_marker_.points.clear();
  spheres_marker_.colors.clear();

  spheres_marker_.points = points; // straight copy

  // Convert path coordinates
  for( std::size_t i = 0; i < points.size(); ++i )
  {
    spheres_marker_.colors.push_back( this_color );
  }

  // Helper for publishing rviz markers
  return publishMarker( spheres_marker_ );
}

bool RvizVisualTools::publishText(const Eigen::Affine3d &pose, const std::string &text,
                                  const rviz_visual_tools::colors &color,
                                  const rviz_visual_tools::scales &scale, bool static_id)
{
  return publishText(convertPose(pose), text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_visual_tools::colors &color, const rviz_visual_tools::scales &scale, bool static_id)
{
  return publishText(pose, text, color, getScale(scale), static_id);
}

bool RvizVisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_visual_tools::colors &color, const geometry_msgs::Vector3 scale, bool static_id)
{
  // Save the ID if this is a static ID or keep incrementing ID if not static
  double temp_id = text_marker_.id;
  if (static_id)
  {
    text_marker_.id = 0;
  }
  else
  {
    text_marker_.id++;
  }

  text_marker_.header.stamp = ros::Time::now();
  text_marker_.header.frame_id = base_frame_;
  text_marker_.text = text;
  text_marker_.pose = pose;
  text_marker_.color = getColor( color );
  text_marker_.scale = scale;

  // Helper for publishing rviz markers
  publishMarker( text_marker_ );

  // Restore the ID count if needed
  if (static_id)
    text_marker_.id = temp_id;

  return true;
}

bool RvizVisualTools::publishTests()
{
  // Create pose
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;

  // Test all shapes ----------

  ROS_INFO_STREAM_NAMED("test","Publishing Axis");
  generateRandomPose(pose1);
  publishAxis(pose1);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
  generateRandomPose(pose1);
  publishArrow(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Sphere");
  generateRandomPose(pose1);
  publishSphere(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Rectangular Cuboid");
  // TODO: use generateRandomCuboid()
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishCuboid(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Line");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishLine(pose1.position, pose2.position, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Block");
  generateRandomPose(pose1);
  publishBlock(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Cylinder");
  generateRandomPose(pose1);
  publishCylinder(pose1, rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Text");
  generateRandomPose(pose1);
  publishText(pose1, "Test", rviz_visual_tools::RAND);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Wireframe Cuboid");
  Eigen::Vector3d min_point, max_point;
  // TODO: use generateRandomCuboid()
  min_point << -0.1, -.25, -.3;
  max_point << .3, .2, .1;
  generateRandomPose(pose1);
  publishWireframeCuboid(convertPose(pose1), min_point, max_point);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing depth/width/height Wireframe Cuboid");
  double depth = 0.5, width = 0.25, height = 0.125;
  generateRandomPose(pose1);
  publishWireframeCuboid(convertPose(pose1), depth, width, height);
  ros::Duration(1.0).sleep();

  return true;
}

geometry_msgs::Pose RvizVisualTools::convertPose(const Eigen::Affine3d &pose)
{
  tf::poseEigenToMsg(pose, shared_pose_msg_);
  return shared_pose_msg_;
}

Eigen::Affine3d RvizVisualTools::convertPose(const geometry_msgs::Pose &pose)
{
  tf::poseMsgToEigen(pose, shared_pose_eigen_);
  return shared_pose_eigen_;
}

Eigen::Affine3d RvizVisualTools::convertPoint32ToPose(const geometry_msgs::Point32 &point)
{
  shared_pose_eigen_ = Eigen::Affine3d::Identity();
  shared_pose_eigen_.translation().x() = point.x;
  shared_pose_eigen_.translation().y() = point.y;
  shared_pose_eigen_.translation().z() = point.z;
  return shared_pose_eigen_;
}

geometry_msgs::Pose RvizVisualTools::convertPointToPose(const geometry_msgs::Point &point)
{
  geometry_msgs::Pose pose_msg; // TODO, how to use shared_pose_msg_ but reset the orientation?
  pose_msg.position = point;
  return pose_msg;
}

geometry_msgs::Point RvizVisualTools::convertPoseToPoint(const Eigen::Affine3d &pose)
{
  tf::poseEigenToMsg(pose, shared_pose_msg_);
  return shared_pose_msg_.position;
}

Eigen::Vector3d RvizVisualTools::convertPoint(const geometry_msgs::Point &point)
{
  shared_point_eigen_[0] = point.x;
  shared_point_eigen_[1] = point.y;
  shared_point_eigen_[2] = point.z;
  return shared_point_eigen_;
}

Eigen::Vector3d RvizVisualTools::convertPoint32(const geometry_msgs::Point32 &point)
{
  shared_point_eigen_[0] = point.x;
  shared_point_eigen_[1] = point.y;
  shared_point_eigen_[2] = point.z;
  return shared_point_eigen_;
}

geometry_msgs::Point32 RvizVisualTools::convertPoint32(const Eigen::Vector3d &point)
{
  shared_point32_msg_.x = point[0];
  shared_point32_msg_.y = point[1];
  shared_point32_msg_.z = point[2];
  return shared_point32_msg_;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const geometry_msgs::Vector3 &point)
{
  shared_point_msg_.x = point.x;
  shared_point_msg_.y = point.y;
  shared_point_msg_.z = point.z;
  return shared_point_msg_;
}

geometry_msgs::Point RvizVisualTools::convertPoint(const Eigen::Vector3d &point)
{
  shared_point_msg_.x = point.x();
  shared_point_msg_.y = point.y();
  shared_point_msg_.z = point.z();
  return shared_point_msg_;
}

void RvizVisualTools::generateRandomPose(geometry_msgs::Pose& pose, RandomPoseBounds pose_bounds)
{
  generateRandomPose(shared_pose_eigen_, pose_bounds);
  pose = convertPose(shared_pose_eigen_);
}

void RvizVisualTools::generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& depth, double& width, double& height,
                                           RandomPoseBounds pose_bounds, RandomCuboidBounds cuboid_bounds)
{
  // Size
  depth = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  width = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);
  height = fRand(cuboid_bounds.cuboid_size_min_, cuboid_bounds.cuboid_size_max_);

  // Orientation
  generateRandomPose(cuboid_pose, pose_bounds);
}

void RvizVisualTools::generateRandomPose(Eigen::Affine3d& pose, RandomPoseBounds pose_bounds)
{
  // Error check elevation & azimuth angles
  // 0 <= elevation <= pi
  // 0 <= azimuth   <= 2 * pi
  if (pose_bounds.elevation_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED("gen_random_pose", "min elevation bound < 0, setting equal to 0");
    pose_bounds.elevation_min_ = 0;
  }

  if (pose_bounds.elevation_max_ > M_PI)
  {
    ROS_WARN_STREAM_NAMED("gen_random_pose", "max elevation bound > pi, setting equal to pi ");
    pose_bounds.elevation_max_ = M_PI;
  }

  if (pose_bounds.azimuth_min_ < 0)
  {
    ROS_WARN_STREAM_NAMED("gen_random_pose", "min azimuth bound < 0, setting equal to 0");
    pose_bounds.azimuth_min_ = 0;
  }

  if (pose_bounds.azimuth_max_ > 2 * M_PI)
  {
    ROS_WARN_STREAM_NAMED("gen_random_pose", "max azimuth bound > 2 pi, setting equal to 2 pi ");
    pose_bounds.azimuth_max_ = 2 * M_PI;
  }


  // Position
  pose.translation().x() = dRand(pose_bounds.x_min_, pose_bounds.x_max_);
  pose.translation().y() = dRand(pose_bounds.y_min_, pose_bounds.y_max_);
  pose.translation().z() = dRand(pose_bounds.z_min_, pose_bounds.z_max_);

  // Random orientation (random rotation axis from unit sphere and random angle)
  double angle = dRand(pose_bounds.angle_min_, pose_bounds.angle_max_);
  double elevation = dRand(pose_bounds.elevation_min_, pose_bounds.elevation_max_);
  double azimuth = dRand(pose_bounds.azimuth_min_, pose_bounds.azimuth_max_);

  Eigen::Vector3d axis;
  axis[0] = sin(elevation) * cos(azimuth);
  axis[1] = sin(elevation) * sin(azimuth);
  axis[2] = cos(elevation);

  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), axis));
  pose = Eigen::Translation3d(pose.translation().x(), pose.translation().y(), pose.translation().z()) * quat;
}

void RvizVisualTools::generateEmptyPose(geometry_msgs::Pose& pose)
{
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
}

double RvizVisualTools::dRand(double dMin, double dMax)
{
  double d = (double)rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

float RvizVisualTools::fRand(float dMin, float dMax)
{
  float d = (float)rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

int RvizVisualTools::iRand(int min, int max)
{
  int n = max - min + 1;
  int remainder = RAND_MAX % n;
  int x;
  do
  {
    x = rand();
  }
  while (x >= RAND_MAX - remainder);
  return min + x % n;
}

void RvizVisualTools::print()
{
  ROS_WARN_STREAM_NAMED("visual_tools","Debug Visual Tools variable values:");
  std::cout << "marker_topic_: " << marker_topic_ << std::endl;
  std::cout << "base_frame_: " << base_frame_ << std::endl;
  std::cout << "floor_to_base_height_: " << floor_to_base_height_ << std::endl;
  std::cout << "marker_lifetime_: " << marker_lifetime_.toSec() << std::endl;
  std::cout << "alpha_: " << alpha_ << std::endl;
}

<<<<<<< Updated upstream
=======
bool RvizVisualTools::publishXArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(convertPose(pose), color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishXArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  return publishArrow(pose, color, scale, length);
}

bool RvizVisualTools::publishYArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishYArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}

bool RvizVisualTools::publishZArrow(const Eigen::Affine3d &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = pose * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  return publishArrow(convertPose(arrow_pose), color, scale, length);
}

bool RvizVisualTools::publishZArrow(const geometry_msgs::PoseStamped &pose, const rviz_visual_tools::colors &color,
                                    const rviz_visual_tools::scales &scale, double length)
{
  Eigen::Affine3d arrow_pose = convertPose(pose.pose) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
  geometry_msgs::PoseStamped new_pose = pose;
  new_pose.pose = convertPose(arrow_pose);
  return publishArrow(new_pose, color, scale, length);
}


>>>>>>> Stashed changes

} // namespace
