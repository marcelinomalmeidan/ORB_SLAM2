/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <string>
#include <vector>
#include <set>
#include "visualization_functions.h"

namespace visualization_functions {

// Overwrites the given properties of the marker array.
void SetMarkerProperties(const std_msgs::Header &header,
                         const double &life_time,
                         visualization_msgs::MarkerArray* markers) {
  int count = 0;
  for (visualization_msgs::Marker& marker : markers->markers) {
    marker.header = header;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}


void DeleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray *marker_array) {
  visualization_msgs::Marker deleteMarker;
  deleteMarker.action = deleteMarker.DELETEALL;
  deleteMarker.scale.x = 0.1;
  deleteMarker.scale.y = 0.1;
  deleteMarker.scale.z = 0.1;
  deleteMarker.header.frame_id = frame_id;
  deleteMarker.ns = "";
  marker_array->markers.push_back(deleteMarker);
}

void SetMarkersForDeletion(visualization_msgs::MarkerArray* marker_array) {
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
  }
}

void DrawNodes(const std::vector<Eigen::Vector3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array) {
  // marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  // Get the number of requested waypoints
  uint n_w = points.size();

  if (n_w == 0) {
    marker.action = visualization_msgs::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::Marker::ADD;
  }

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point NewPoint;
    NewPoint.x = points[i](0);
    NewPoint.y = points[i](1);
    NewPoint.z = points[i](2);
    marker.points.push_back(NewPoint);
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  SetMarkerProperties(header, 0.0, marker_array);
}

void MarkerNode(const Eigen::Vector3d &point,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const double &resolution,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;
  marker->color.a = transparency;
  marker->scale.x = resolution;
  marker->scale.y = resolution;
  marker->scale.z = resolution;
  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->pose.orientation.w = 1.0;

  geometry_msgs::Point position_msg;
  position_msg.x = point(0);
  position_msg.y = point(1);
  position_msg.z = point(2);
  marker->pose.position = position_msg;
  marker->header.seq = seqNumber;
}

void MarkerNodeTemplate(const std::string &frame_id,
                        const std::string &ns,  // namespace
                        const double &size,
                        const std_msgs::ColorRGBA &color,
                        const double &transparency,  // 0 -> transparent, 1 -> opaque
                        const int &seqNumber,
                        visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::POINTS;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;
  marker->color.a = transparency;
  marker->scale.x = size;
  marker->scale.y = size;
  marker->scale.z = size;
  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->pose.orientation.w = 1.0;

  marker->header.seq = seqNumber;
}


void DrawArrowPoints(const Eigen::Vector3d& p1,
                     const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA &color,
                     const double &diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  marker->points[0] = msg_conversions::eigen_to_ros_point(p1);
  marker->points[1] = msg_conversions::eigen_to_ros_point(p2);
  // EigenPoint2RosPoint(p1, &marker->points[0]);
  // EigenPoint2RosPoint(p2, &marker->points[1]);

  marker->scale.x = diameter * 0.1;
  marker->scale.y = diameter * 2 * 0.1;
  marker->scale.z = 0.1;
  marker->pose.orientation.w = 1.0;
}

// Return visualization markers frustum visualization
void GetFrustumMarker(const double fov, const double aspectRatio,
                      const std::string &frame_id, geometry_msgs::Pose pose,
                      visualization_msgs::Marker *line_list) {
    Eigen::Affine3d eig_pose = msg_conversions::ros_pose_to_eigen_transform(pose);

    // Set frustum corners
    double znear = 0.02;
    double hh = znear*tan(fov/2.0);
    double hw = hh*aspectRatio;
    Eigen::Vector3d UL = eig_pose*Eigen::Vector3d(znear,  hw,  hh);
    Eigen::Vector3d UR = eig_pose*Eigen::Vector3d(znear, -hw,  hh);
    Eigen::Vector3d DL = eig_pose*Eigen::Vector3d(znear,  hw, -hh);
    Eigen::Vector3d DR = eig_pose*Eigen::Vector3d(znear, -hw, -hh);
    Eigen::Vector3d origin = eig_pose*Eigen::Vector3d(0.0, 0.0, 0.0);

    // Initialize array
    line_list->header.frame_id = frame_id;
    line_list->header.stamp = ros::Time::now();
    line_list->ns = "keyframes";
    line_list->action = visualization_msgs::Marker::ADD;
    line_list->pose.orientation.w = 1.0;
    line_list->type = visualization_msgs::Marker::LINE_LIST;
    line_list->id = 0;
    line_list->scale.x = 0.001;  // Line width
    line_list->color = visualization_functions::Color::Yellow();
    // line_list->lifetime = ros::Duration(1);  // Disappears in 1 second

    std::vector<Eigen::Vector3d> points;
    points.push_back(origin);
    points.push_back(UL);
    points.push_back(UR);
    points.push_back(DL);
    points.push_back(DR);

    geometry_msgs::Point node1, node2;
    for (uint i = 0; i < points.size(); i++) {
        for (uint j = 0; j < points.size(); j++) {
            if (i == j) {
                continue;
            }
            node1 = msg_conversions::eigen_to_ros_point(points[i]);
            node2 = msg_conversions::eigen_to_ros_point(points[j]);
            line_list->points.push_back(node1);
            line_list->points.push_back(node2);
        }
    }
}

// Return visualization marker for graph visualization
void GraphMarker(const std::string &frame_id, visualization_msgs::Marker *line_list) {

    // Initialize array
    line_list->header.frame_id = frame_id;
    line_list->header.stamp = ros::Time::now();
    line_list->ns = "graph";
    line_list->action = visualization_msgs::Marker::ADD;
    line_list->pose.orientation.w = 1.0;
    line_list->type = visualization_msgs::Marker::LINE_LIST;
    line_list->id = 0;
    line_list->scale.x = 0.001;  // Line width
    line_list->color = visualization_functions::Color::Green();
}

}  // namespace visualization_functions
