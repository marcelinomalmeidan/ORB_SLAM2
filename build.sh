echo "Configuring and building Thirdparty/DBoW2 ..."

export ROS_DISTRO=kinetic
export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:/opt/ros/${ROS_DISTRO}/share
# export CPATH=/opt/ros/${ROS_DISTRO}/include

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
