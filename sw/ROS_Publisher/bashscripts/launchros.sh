#!/bin/bash

source /opt/ros/foxy/setup.bash
source /root/install/setup.bash
source ~/.bashrc
export ROS_DOMAIN_ID=1
ros2 run image_pub pub_img
