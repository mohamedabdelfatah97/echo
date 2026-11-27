#!/usr/bin/env python3
"""ECHO NAV-0: small Jazzy Nav2 subset, adapted from verified Carter baseline.

This launch does not start Isaac Sim, AMCL, map_server, RTAB-Map, or the old navigation scene.
The sole publisher to /cmd_vel should be collision_monitor. Check this before sending any goal.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    nodes = []
    def add(package, executable, name, remaps=()):
        nodes.append(Node(
            package=package, executable=executable, name=name,
            output='screen', parameters=[params],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), *remaps],
        ))

    nodes.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='echo_nav0_pointcloud_to_laserscan', output='screen',
        remappings=[('cloud_in','/front_3d_lidar/lidar_points'), ('scan','/scan')],
        parameters=[{
            'use_sim_time':True,
            # Restore baseline LiDAR-frame conversion; front +/-90 deg scan only.
            # FRONT +/-90 degrees only; never assume rear coverage from this scan.
            'target_frame':'front_3d_lidar', 'transform_tolerance':0.3,
            'min_height':-0.4, 'max_height':1.5,
            'angle_min':-1.5708,'angle_max':1.5708,'angle_increment':0.0087,
            'scan_time':0.3333, 'range_min':0.05,'range_max':100.0,
            'use_inf':True,'inf_epsilon':1.0,
        }],
    ))
    add('nav2_controller','controller_server','controller_server', [('cmd_vel','cmd_vel_nav')])
    add('nav2_smoother','smoother_server','smoother_server')
    add('nav2_planner','planner_server','planner_server')
    add('nav2_behaviors','behavior_server','behavior_server', [('cmd_vel','cmd_vel_nav')])
    add('nav2_bt_navigator','bt_navigator','bt_navigator')
    add('nav2_velocity_smoother','velocity_smoother','velocity_smoother',[('cmd_vel','cmd_vel_nav')])
    add('nav2_collision_monitor','collision_monitor','collision_monitor')
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True,'autostart': True,'bond_timeout':20.0,
                     'node_names':['controller_server','smoother_server',
                      'planner_server','behavior_server','velocity_smoother',
                      'collision_monitor','bt_navigator']}],
    ))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(Path(__file__).resolve().parents[1] / 'config' / 'nav2' / 'nav0' / 'echo_nav0_params.yaml')),
        *nodes,
    ])
