
echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16

cd ..

echo "Building ROS nodes"

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:./Examples/ROS
export ROS_DISTRO=kinetic
export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:/opt/ros/${ROS_DISTRO}/share

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j16
