#!/usr/bin/env pyhton3
import os
from os.path import expanduser
import random

## Common initialization
os.system("sudo apt update")
os.system("sudo apt -y install curl gnupg2 lsb-release ")

## ROS 1
# Install ROS 1 base
os.system("sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'")
os.system("sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654")
os.system("sudo apt update")
os.system("sudo apt -y install ros-melodic-desktop")
os.system("source ~/.bashrc")

os.system("sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential")
os.system("sudo apt -y install python-rosdep catkin")
os.system("sudo rosdep init")
os.system("rosdep update")
os.system("sudo apt -y autoremove") # Clean all remnants

# Setup catkin_ws
os.system("mkdir -p ~/catkin_ws/src")
os.chdir(expanduser("~") + "/catkin_ws")
os.system("catkin_make")

# Install vcstool
os.system("sudo apt -y install python3-vcstool")

# Install Nao things
os.chdir(expanduser("~") + "/catkin_ws/src")
# Base nao
os.system("sudo apt -y ros-melodic-nao-meshes ros-melodic-naoqi-bridge-msgs ros-melodic-camera-info-manager ros-melodic-octomap-msgs ros-melodic-move-base-msgs liboctomap-dev")
# NaoGazebo
os.system("sudo apt -y install ros-melodic-gazebo-ros ros-melodic-controller-manager")
os.system("git clone https://github.com/ros-naoqi/nao_moveit_config")
os.system("git clone https://github.com/ros-naoqi/nao_extras")
os.system("git clone https://github.com/ros-naoqi/naoqi_bridge")
os.system("git clone https://github.com/ros-naoqi/nao_robot")
os.system("git clone https://github.com/ros-naoqi/nao_virtual")
os.system("git clone https://github.com/ahornung/humanoid_msgs/")
os.chdir(expanduser("~") + "/catkin_ws/")
os.system("catkin_make")
# Much easier just to call rosdep, actually
os.system("rosdep install -i --from-path src --rosdistro melodic -y")

## ROS 2
# Make sure we have an UTF-8 compatible system
os.system("sudo apt update")
os.system("sudo apt -y install locales")
os.system("sudo locale-gen es_ES es_ES.UTF-8")
os.system("sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8")
os.system("export LANG=es_ES.UTF-8")

# Install ROS 2 base
os.system("curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -") # Add key
os.system("sudo sh -c 'echo \"deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'")
os.system("sudo apt update")
os.system("sudo apt -y install python3-colcon-common-extensions ros-eloquent-desktop")

# Install argcomplete (to have autocompletion)
os.system("sudo apt -y install python3-pip")
os.system("pip3 install -U argcomplete")

# Make ROS 2 development workspace
os.system("mkdir -p ~/dev_ws/src")
os.chdir(expanduser("~") + "/dev_ws")
os.system("colcon build")

domain_id = random.randint(0,256)
os.system("echo \"\n# ROS-related settings and funtions\" >> ~/.bashrc")
os.system(f"echo \"export ROS_DOMAIN_ID={domain_id}\" >> ~/.bashrc")
os.system(r'echo "echo \"ROS_DOMAIN_ID: \$ROS_DOMAIN_ID\"" >> ~/.bashrc')

# Add enable_ros1 function to set-up ros1 env
os.system("echo \"\nenable_ros1() {\" >> ~/.bashrc")
os.system("echo \"  source /opt/ros/melodic/setup.bash\" >> ~/.bashrc")
os.system("echo \"  source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc")
os.system("echo \"}\" >> ~/.bashrc")

# Add enable_ros1 function to set-up ros1 env
os.system("echo \"\nenable_ros2() {\" >> ~/.bashrc")
os.system("echo \"  source /opt/ros/eloquent/setup.bash\" >> ~/.bashrc")
os.system("echo \"  source /usr/share/colcon_cd/function/colcon_cd.sh\" >> ~/.bashrc")
os.system("echo \"  source ~/dev_ws/install/local_setup.bash\" >> ~/.bashrc")
os.system("echo \"  export _colcon_cd_root=~/ros2_install\" >> ~/.bashrc")
os.system("echo \"}\" >> ~/.bashrc")

# Finally, for VM guests make them use OpenGL 2.1 instead of 3.3 (it works much better)
os.system(r'grep -q ^flags.*\ hypervisor\  /proc/cpuinfo && echo "export SVGA_VGPU10=0" >> ~/.profile')

print(f"\n\nYour domain id is {domain_id}, make sure no one has the same")
print("Please do 'source ~/.bashrc' or close this shell and open a new one")