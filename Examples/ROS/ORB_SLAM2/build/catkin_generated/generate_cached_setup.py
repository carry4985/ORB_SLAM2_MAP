# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/indigo/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/indigo/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/robot/catkin_slam/devel_isolated/scanmove;/home/robot/catkin_slam/devel_isolated/particle_filter_localization;/home/robot/catkin_slam/devel_isolated/ytlidar_ros;/home/robot/catkin_slam/devel_isolated/ydlidar;/home/robot/catkin_slam/devel_isolated/rviz_imu_plugin;/home/robot/catkin_slam/devel_isolated/mpu6050_imu;/home/robot/catkin_slam/devel_isolated/imu_tools;/home/robot/catkin_slam/devel_isolated/imu_filter_madgwick;/home/robot/catkin_slam/devel_isolated/imu_complementary_filter;/home/robot/catkin_slam/devel_isolated/hector_trajectory_server;/home/robot/catkin_slam/devel_isolated/hector_slam_launch;/home/robot/catkin_slam/devel_isolated/hector_slam;/home/robot/catkin_slam/devel_isolated/hector_map_server;/home/robot/catkin_slam/devel_isolated/hector_geotiff_plugins;/home/robot/catkin_slam/devel_isolated/hector_geotiff;/home/robot/catkin_slam/devel_isolated/hector_nav_msgs;/home/robot/catkin_slam/devel_isolated/hector_marker_drawing;/home/robot/catkin_slam/devel_isolated/hector_mapping;/home/robot/catkin_slam/devel_isolated/hector_compressed_map_transport;/home/robot/catkin_slam/devel_isolated/hector_map_tools;/home/robot/catkin_slam/devel_isolated/hector_imu_tools;/home/robot/catkin_slam/devel_isolated/hector_imu_attitude_to_tf;/home/robot/catkin_slam/devel_isolated/cartographer_toru;/home/robot/catkin_slam/devel_isolated/cartographer_rviz;/home/robot/catkin_slam/devel_isolated/cartographer_mir;/home/robot/catkin_slam/devel_isolated/cartographer_ros;/home/robot/catkin_slam/devel_isolated/cartographer_ros_msgs;/opt/ros/indigo".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/robot/Desktop/Project_CPP/ORB_SLAM2_Map/Examples/ROS/ORB_SLAM2/build/devel/env.sh')

output_filename = '/home/robot/Desktop/Project_CPP/ORB_SLAM2_Map/Examples/ROS/ORB_SLAM2/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
