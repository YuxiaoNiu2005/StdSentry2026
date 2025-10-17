#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/zc/StdSentry2025/install/setup.bash

sleep 5

ros2 launch sentry_control sentry_control.launch.py
ros2 run sentry_control sentry_nav_goal