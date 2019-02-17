/**
* This file is part of ORB-SLAM2.
*
* This portion has been written by Marcelino Almeida <marcelino dot malmeidan at utexas dot edu>
* For more information see <https://github.com/marcelinomalmeidan/ORB_SLAM2.git>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RVIZDRAWER_H
#define RVIZDRAWER_H

#include <thread>

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "FrameDrawer.h"

// ROS-related libraries
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "visualization_functions.h"
#include "msg_conversions.h"

namespace ORB_SLAM2
{

class FrameDrawer;

class RvizDrawer
{
public:

    RvizDrawer(Map* pMap, FrameDrawer* pFrameDrawer, ros::NodeHandle *nh);

    Map* mpMap;
    FrameDrawer* mpFrameDrawer;

    void DrawTask();
    void DrawMap();
    void DrawKeyFrames();

private:

    ros::NodeHandle nh_;
    ros::Publisher map_points_pub_, keyframes_pub_, graph_pub_; 
    std::string frame_id_;
    float point_size_;
    std_msgs::ColorRGBA white_color_, red_color_;
    float transparency_;
    bool draw_keyframes_, draw_graph_;

    std::thread h_draw_thread_;

};

} //namespace ORB_SLAM

#endif // RVIZDRAWER_H
