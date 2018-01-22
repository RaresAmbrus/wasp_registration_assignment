#!/bin/sh

# INSTALL DEPENDENCIES
sudo apt-get update
sudo apt-get install -y qtcreator terminator git

# INSTALL PCL 1.8
sudo apt-get install -y git build-essential linux-libc-dev cmake cmake-gui libusb-1.0-0-dev libusb-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev libeigen3-dev libboost-all-dev libvtk5.10-qt4 libvtk5.10 libvtk5-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev  mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre

cd ~
mkdir software
cd software
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xvf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_GPU=OFF -DBUILD_apps=OFF -DBUILD_examples=OFF
make -j3
sudo make install
cd ~/software

# INSTALL ROS Kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# INSTALL Ceres
sudo apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev
sudo apt-get install -y libsuitesparse-dev

cd ~/software
wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz
tar -zxf ceres-solver-1.13.0.tar.gz
cd ceres-solver-1.13.0
mkdir ceres-bin
cd ceres-bin
cmake ..
make -j3
make test
sudo make install

