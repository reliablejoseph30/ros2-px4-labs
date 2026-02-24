#!/bin/bash

set -e

echo "=== ROS2 + PX4 Lab Setup Starting ==="

# Check Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "This script only works on Ubuntu (including WSL Ubuntu)."
    exit 1
fi

echo "Updating system..."
sudo apt update && sudo apt upgrade -y

echo "Installing base tools..."
sudo apt install -y git curl wget build-essential software-properties-common lsb-release gnupg

echo "Installing ROS 2 Humble..."
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "Installing Gazebo Classic..."
sudo apt install -y gazebo11 libgazebo11-dev

echo "Installing GeographicLib datasets..."
sudo apt install -y geographiclib-tools
sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-magnetic emm2015

echo "Creating ROS 2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

if [ ! -d "ros2-px4-labs" ]; then
    git clone https://github.com/reliablejoseph30/ros2-px4-labs.git
fi

cd ~/ros2_ws
colcon build

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "=== SETUP COMPLETE ==="
echo "Run: source ~/.bashrc"
