Sentry Nav 26
A modular navigation system built with ROS 2 Jazzy and Nav2 for autonomous robot navigation.
Overview
This workspace contains packages for simulation, localization, mapping, navigation, and system integration, designed for modular development and testing with Gazebo Harmonic and Nav2.
Packages

sentry_common: Common utilities and data structures.
sentry_sim: Gazebo Harmonic simulation environment.
sentry_nav: Navigation core (planners, controllers, behavior trees).
sentry_localization: Localization using AMCL or robot_localization.
sentry_mapping: Map generation using SLAM Toolbox.
sentry_bringup: System integration and launch files.
sentry_msgs: Custom messages, services, and actions.

Prerequisites

ROS 2 Jazzy
Gazebo Harmonic
Ubuntu 24.04 (Noble Numbat)

Installation
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-gz-tools-vendor ros-jazzy-gz-sim-vendor ros-jazzy-ros-gz
cd ~/sentry_nav_26
colcon build --symlink-install

安装 NLopt 开发包
sudo apt update
sudo apt install libnlopt-dev libnlopt-cxx-dev

验证安装
pkg-config --modversion nlopt

Usage
source ~/sentry_nav_26/install/setup.bash
ros2 launch sentry_bringup bringup.launch.py

License
BSD-3-Clause