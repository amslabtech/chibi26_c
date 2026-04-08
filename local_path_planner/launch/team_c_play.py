from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import LifecycleNode 
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='c_obstacle_detector',
            executable='c_obstacle_detector_node',
            # parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_c_local_map_creator',
            executable='team_c_local_map_creator_node',
            # parameters=[{'use_sim_time': True}],
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_c_localizer',
            executable='team_c_localizer_node',
            # parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_c_global_path_planner',
            executable='team_c_global_path_planner_node',
            # parameters=['/home/user/ws/src/chibi26_c/global_path_planner/config/param/global_path_planner.yaml', {'use_sim_time': True}],
             parameters=['/home/user/ws/src/chibi26_c/global_path_planner/config/param/global_path_planner.yaml', {'use_sim_time': False}],
        ),
        Node(
            package='team_c_local_goal_creator',
            executable='team_c_local_goal_creator_node',
            # parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='team_c_local_path_planner',
            executable='team_c_local_path_planner_node',
            # parameters=[{'use_sim_time': True}],
             parameters=['/home/user/ws/src/chibi26_c/local_path_planner/config/param/local_path_planner.yaml', {'use_sim_time': False}],
        ),
        
        # Node(
        #   package='rviz2',
        #   executable='rviz2',
        #   arguments=['-d','./src/chibi26_c/bag/rviz_debag.rviz'],
        #   parameters=[{'use_sim_time': True}],
        # ),
        

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/user/ws/src/chibi26_c/bag/map/c_map.yaml',
                'use_sim_time': True
            }]
        ),

        TimerAction(
            period=1.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser'],
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', '/home/user/ws/src/chibi26_c/bag/rosbag2_2026_03_11-05_14_01', '--clock'],
        #   output='screen'
        # )
            ]
         ),
    
                
            
        TimerAction(
            period=2.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    name='map_server_lifecycle',
                    output='screen',
                    arguments=['map_server']
                )
            ]
        )

        

        
        
        ])