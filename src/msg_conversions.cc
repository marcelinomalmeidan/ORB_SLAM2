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

#include <msg_conversions.h>

namespace msg_conversions {

geometry_msgs::Point set_ros_point(const double &x, const double &y, const double &z) {
  geometry_msgs::Point n;
  n.x = x;
  n.y = y;
  n.z = z;
  return n;
}

geometry_msgs::Quaternion set_ros_quaternion(const double &w, const double &x, 
                                             const double &y, const double &z) {
  geometry_msgs::Quaternion q;
  q.w = w;
  q.x = x;
  q.y = y;
  q.z = z;
  return q;
}

Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point & p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}


Eigen::Vector3d ros_to_eigen_vector(const geometry_msgs::Vector3 & v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d & v) {
  geometry_msgs::Vector3 n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d & v) {
  geometry_msgs::Point n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

Eigen::Quaterniond ros_to_eigen_quat(const geometry_msgs::Quaternion & q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond & q) {
  geometry_msgs::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Vector4d & v) {
  geometry_msgs::Quaternion out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  out.w = v.w();
  return out;
}

geometry_msgs::Vector3 array_to_ros_vector(float* array) {
  geometry_msgs::Vector3 v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

geometry_msgs::Point array_to_ros_point(float* array) {
  geometry_msgs::Point v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

geometry_msgs::Quaternion array_to_ros_quat(float* array) {
  geometry_msgs::Quaternion q;
  q.x = array[0];
  q.y = array[1];
  q.z = array[2];
  q.w = array[3];
  return q;
}

void ros_to_array_vector(const geometry_msgs::Vector3 & v, float* array) {
  array[0] = v.x;
  array[1] = v.y;
  array[2] = v.z;
}

void ros_to_array_point(const geometry_msgs::Point & p, float* array) {
  array[0] = p.x;
  array[1] = p.y;
  array[2] = p.z;
}

void ros_to_array_quat(const geometry_msgs::Quaternion & q, float* array) {
  array[0] = q.x;
  array[1] = q.y;
  array[2] = q.z;
  array[3] = q.w;
}

void eigen_to_array_vector(const Eigen::Vector3d & v, float* array) {
  array[0] = v.x();
  array[1] = v.y();
  array[2] = v.z();
}

void eigen_to_array_quat(const Eigen::Quaterniond & q, float* array) {
  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();
}

Eigen::Affine3d ros_pose_to_eigen_transform(const geometry_msgs::Pose & p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  transform.linear() = Eigen::Quaterniond(
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
  return transform;
}

Eigen::Affine3d ros_to_eigen_transform(const geometry_msgs::Transform & p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.translation.x, p.translation.y, p.translation.z);
  transform.linear() = Eigen::Quaterniond(
    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  return transform;
}

void orbslam_transform_to_ros_pose(const cv::Mat &TransfMat, geometry_msgs::Pose *p) {
  // cv::Mat TransfMatInv = TransfMat.inv();
  // Eigen::Vector3f tInv(TransfMatInv.at<float>(0,3), TransfMatInv.at<float>(1,3), TransfMatInv.at<float>(2,3));
  // Get rotation
  Eigen::Matrix3f R, Rinv;
  R << TransfMat.at<float>(0,0), TransfMat.at<float>(0,1), TransfMat.at<float>(0,2),
       TransfMat.at<float>(1,0), TransfMat.at<float>(1,1), TransfMat.at<float>(1,2),
       TransfMat.at<float>(2,0), TransfMat.at<float>(2,1), TransfMat.at<float>(2,2);
  Rinv = R.transpose();
  Eigen::Quaternionf q(Rinv);

  // Get translation
  Eigen::Vector3f t(TransfMat.at<float>(0,3), TransfMat.at<float>(1,3), TransfMat.at<float>(2,3));
  Eigen::Vector3f pos = -Rinv*t;

  // Set output structure
  p->position = set_ros_point(pos(2), -pos(0), -pos(1));
  p->orientation = set_ros_quaternion(q.w(), q.z(), -q.x(), -q.y());
}

void orbslam_transform_to_ros_pose(const cv::Mat &TransfMat, geometry_msgs::Pose *pose_base,
                                   geometry_msgs::Pose *pose_camera) {
  // cv::Mat TransfMatInv = TransfMat.inv();
  // Eigen::Vector3f tInv(TransfMatInv.at<float>(0,3), TransfMatInv.at<float>(1,3), TransfMatInv.at<float>(2,3));
  // Get rotation
  Eigen::Matrix3f R, Rinv;
  R << TransfMat.at<float>(0,0), TransfMat.at<float>(0,1), TransfMat.at<float>(0,2),
       TransfMat.at<float>(1,0), TransfMat.at<float>(1,1), TransfMat.at<float>(1,2),
       TransfMat.at<float>(2,0), TransfMat.at<float>(2,1), TransfMat.at<float>(2,2);
  Rinv = R.transpose();
  Eigen::Quaternionf q(Rinv);
  Eigen::Quaternionf q_enu(q.w(), q.z(), -q.x(), -q.y());

  // Get translation
  Eigen::Vector3f t(TransfMat.at<float>(0,3), TransfMat.at<float>(1,3), TransfMat.at<float>(2,3));
  Eigen::Vector3f pos = -Rinv*t;

  // Set pose of base link
  pose_base->position = set_ros_point(pos(2), -pos(0), -pos(1));
  pose_base->orientation = set_ros_quaternion(q.w(), q.z(), -q.x(), -q.y());

  // Set camera pose (z pointing outwards from the camera)
  Eigen::Quaternionf q_enu2cam(0.5, -0.5, 0.5, -0.5);
  Eigen::Quaternionf q_cam = q_enu*q_enu2cam;
  pose_camera->position = set_ros_point(pos(2), -pos(0), -pos(1));
  pose_camera->orientation = set_ros_quaternion(q_cam.w(), q_cam.x(), q_cam.y(), q_cam.z());
}

}  // end namespace msg_conversions
