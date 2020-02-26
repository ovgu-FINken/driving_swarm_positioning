#!/bin/bash
echo "This script configures the Turtlebot to work with vi serial connected DWM1000 Modules"
if [[ $(id -u) -ne 0 ]] ; then echo "Please run as root" ; exit 1 ; fi
echo "continue (yes/no)?"
read enter
if [[ $enter = "yes" ]]; then
echo "working ... "
#install packages
sudo apt install ros-melodic-tf
sudo apt install ros-melodic-tf2
sudo apt install python-serial
#run config commands
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
else
echo "exit"
exit 1
fi
