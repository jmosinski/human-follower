# Human_follower
Lego robot following humans

## ROS Kinetic
http://wiki.ros.org/kinetic/Installation

## Raspberry Pi System 
Installation
https://downloads.ubiquityrobotics.com/
Updating system, connecting to local network
https://learn.ubiquityrobotics.com/connect_network
ROS Master and multiple machines connection
https://learn.ubiquityrobotics.com/workstation_setup
Mobile app
https://learn.ubiquityrobotics.com/robot_commander

## RPLIDAR and ROS
http://wiki.ros.org/rplidar

## Husarion Core 2
https://husarion.com/manuals/core2/

## How does it work?
Density-based spatial clustering of applications with noise (DBSCAN)
Leg detection using proportions, distance between clusters and robot
Human detection using proportions between potencial legs (sorted cluster list)
Kalman filter to reduce noise
Pure pursuit control
steering the robot - Communication between Rspperry Pi nad Core 2 using UART and rosmsgs
