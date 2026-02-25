#!/usr/bin/env python3

import os
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackage
 
def generate_launch_description():
  args = '-d' + os.path.join(get_package_share_directory('mbmadcar'), 'data', 'track.rviz')
  return LaunchDescription([
    launch_ros.actions.Node(package='rviz2', node_executable='rviz2', output='screen', arguments=[args])
    ,launch_ros.actions.Node(package='mbmadtrack', node_executable='tracknode', output='screen')
    ,launch_ros.actions.Node(package='mbmadcar', node_executable='carsimnode', node_namespace='/mad/car0', output='screen' 
     ,parameters = [ { 'x0': [ 1.0000, 0.1500, 0.8983, -1.5708 ] }
     ])
    ,launch_ros.actions.Node(package='mbmadlocate', node_executable='locatenode', output='screen'
      ,remappings = [ ( '/diagnostics', 'diagnostics' ) ])
      ,launch_ros.actions.Node(package='mbmadcar', node_executable='visionsimnode', output='screen'
        ,remappings = [ ( '/diagnostics', 'diagnostics' ) ])
    ,launch_ros.actions.Node(package='mbmadcar', node_executable='cardisplaynode', node_namespace='/mad/car0', output='screen'
      ,parameters = [ { 'colorRGB': [ 0.9, 0.9, 0.9 ] }
      ])
,launch_ros.actions.Node(package='mbmadctrl', node_executable='carctrlnode', node_namespace='/mad/car0', output='screen'
    ,remappings = [ ( '/diagnostics', 'diagnostics' ) ])
      ])
