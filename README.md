# octomap-slice

This package takes a pointcloud topic and produces an OctoMap (using the built-in octomap server ROS package), as well as a 2D slice at a user-defined altitude in the form of an occupancy grid map.

To run:

`roslaunch octomap_slice octomap_full_handling.launch cloud_topic:=<your_pointcloud> slice_height:=<your_altitude_meters> octomap_res:=<your_resolution_meters>`