#!/bin/bash

echo "Waiting for controller manager to be ready..."
sleep 5

echo "Loading joint_state_broadcaster..."
ros2 control load_controller --set-state active joint_state_broadcaster

echo "Loading forward_velocity_controller..."
ros2 control load_controller --set-state active forward_velocity_controller

echo "Controllers loaded. Check status with: ros2 control list_controllers"