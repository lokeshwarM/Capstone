#!/bin/bash

# Source ROS 2 (change if using foxy/galactic/etc)
source /opt/ros/humble/setup.bash

echo "Starting Gazebo..."
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so city.sdf &
GAZEBO_PID=$!

echo "Waiting for Gazebo to start..."
sleep 5

echo "Spawning Drone 1..."
ros2 run gazebo_ros spawn_entity.py -entity drone1 -file drone.sdf -robot_namespace drone1 -x -7.5 -y -7.5 -z 0.5
echo "Spawning Drone 2..."
ros2 run gazebo_ros spawn_entity.py -entity drone2 -file drone.sdf -robot_namespace drone2 -x 7.5 -y -7.5 -z 0.5
echo "Spawning Drone 3..."
ros2 run gazebo_ros spawn_entity.py -entity drone3 -file drone.sdf -robot_namespace drone3 -x -7.5 -y 7.5 -z 0.5
echo "Spawning Drone 4..."
ros2 run gazebo_ros spawn_entity.py -entity drone4 -file drone.sdf -robot_namespace drone4 -x 7.5 -y 7.5 -z 0.5

echo "All drones spawned!"
echo "Now you can run: python3 manual_control.py"

wait $GAZEBO_PID
