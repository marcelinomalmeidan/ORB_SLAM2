echo "Building ROS nodes"

export ROS_DISTRO=kinetic
export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:/opt/ros/${ROS_DISTRO}/share

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
