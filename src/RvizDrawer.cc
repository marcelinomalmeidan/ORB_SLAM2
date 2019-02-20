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

#include "RvizDrawer.h"

namespace ORB_SLAM2
{


RvizDrawer::RvizDrawer(Map* pMap, FrameDrawer* pFrameDrawer, ros::NodeHandle *nh):
                        mpMap(pMap), mpFrameDrawer(pFrameDrawer), it_(nh_) {
    nh_ = *nh;
    map_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("MapPoints", 2);
    keyframes_pub_ = nh_.advertise<visualization_msgs::Marker>("Keyframes", 2, true);
    graph_pub_ = nh_.advertise<visualization_msgs::Marker>("Graph", 2, true);
    features_pub_ = it_.advertise("Image/Features", 1);
    frame_id_ = "slam";
    point_size_ = 0.005;
    white_color_ = visualization_functions::Color::White();
    red_color_ = visualization_functions::Color::Red();
    transparency_ = 1.0;
    draw_keyframes_ = true;
    draw_graph_ = true;

    h_draw_thread_ = std::thread(&RvizDrawer::DrawTask, this);
}

void RvizDrawer::DrawTask() {

    float fps = 30;
    ros::Rate loop_rate(fps);
    while(ros::ok()) {
        loop_rate.sleep();

        // Draw map points
        this->DrawMap();

        // Draw keyframes
        this->DrawKeyFrames();

        // Draw image with tracked features
        this->PublishImg();
    }
}

void RvizDrawer::DrawMap() {

    // Publish map points if there is at least one subscriber to it
    bool pub_map_points = (map_points_pub_.getNumSubscribers() > 0);
    if (pub_map_points) {
        const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
        const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty())
            return;

        // Get visualization markers
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker white_markers, red_markers;
        std::string namespace1 = "old_points";
        std::string namespace2 = "new_points";
        visualization_functions::MarkerNodeTemplate(frame_id_, namespace1, point_size_, white_color_,
                            transparency_, 0, &white_markers);
        visualization_functions::MarkerNodeTemplate(frame_id_, namespace2, point_size_, red_color_,
                            transparency_, 0, &red_markers);

        // White points (black in original ORB-SLAM2)
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            geometry_msgs::Point NewPoint;
            NewPoint.x =  pos.at<float>(2);
            NewPoint.y = -pos.at<float>(0);
            NewPoint.z = -pos.at<float>(1);
            white_markers.points.push_back(NewPoint);
        }

        // Red points
        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            geometry_msgs::Point NewPoint;
            NewPoint.x =  pos.at<float>(2);
            NewPoint.y = -pos.at<float>(0);
            NewPoint.z = -pos.at<float>(1);
            red_markers.points.push_back(NewPoint);
        }


        markers.markers.push_back(white_markers);
        markers.markers.push_back(red_markers);
        map_points_pub_.publish(markers);
    }
}

void RvizDrawer::DrawKeyFrames() {

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    // Publish keyframes if there is at least one subscriber to it
    bool pub_kf = (keyframes_pub_.getNumSubscribers() > 0);
    if(pub_kf && draw_keyframes_) {
        double fov = M_PI/2.0;
        double aspect_ratio = 1.33;
        visualization_msgs::Marker camera_views;

        for(size_t i=0; i<vpKFs.size(); i++) {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Tcw = pKF->GetPose();
            geometry_msgs::Pose pose;

            msg_conversions::orbslam_transform_to_ros_pose(Tcw, &pose);
            visualization_functions::GetFrustumMarker(fov, aspect_ratio, frame_id_, pose, &camera_views);
        }
        keyframes_pub_.publish(camera_views);
        // draw_keyframes_ = false;
    }

    // Publish graph if there is at least one subscriber to it
    bool pub_graph = (graph_pub_.getNumSubscribers() > 0);
    if(pub_graph && draw_graph_) {
        // Draw graph
        visualization_msgs::Marker graph_marker;
        visualization_functions::GraphMarker(frame_id_, &graph_marker);
        geometry_msgs::Point node1, node2;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty()) {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    node1 = msg_conversions::set_ros_point(Ow.at<float>(2),-Ow.at<float>(0),-Ow.at<float>(1));
                    node2 = msg_conversions::set_ros_point(Ow2.at<float>(2),-Ow2.at<float>(0),-Ow2.at<float>(1));
                    graph_marker.points.push_back(node1);
                    graph_marker.points.push_back(node2);
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent) {
                cv::Mat Owp = pParent->GetCameraCenter();
                node1 = msg_conversions::set_ros_point(Ow.at<float>(2),-Ow.at<float>(0),-Ow.at<float>(1));
                node2 = msg_conversions::set_ros_point(Owp.at<float>(2),-Owp.at<float>(0),-Owp.at<float>(1));
                graph_marker.points.push_back(node1);
                graph_marker.points.push_back(node2);
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++) {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                node1 = msg_conversions::set_ros_point(Ow.at<float>(2),-Ow.at<float>(0),-Ow.at<float>(1));
                node2 = msg_conversions::set_ros_point(Owl.at<float>(2),-Owl.at<float>(0),-Owl.at<float>(1));
                graph_marker.points.push_back(node1);
                graph_marker.points.push_back(node2);
            }
        }
        graph_pub_.publish(graph_marker);
        // draw_graph_ = false;
    }
}

void RvizDrawer::PublishImg() {
    bool pub_img = (keyframes_pub_.getNumSubscribers() > 0);
    if (pub_img) {
        sensor_msgs::Image::Ptr out_img;
        std_msgs::Header header;
        cv::Mat im = mpFrameDrawer->DrawFrame(&header);
        out_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, im).toImageMsg();
        features_pub_.publish(out_img);
    }
}


} //namespace ORB_SLAM
