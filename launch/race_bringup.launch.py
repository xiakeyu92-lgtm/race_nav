#!/usr/bin/env python3
"""
race_bringup.launch.py —— 比赛全流程启动文件。

启动顺序:
  1. map_server          — 加载赛方提供的灰度地图
  2. AMCL                — 定位 (发布 map→odom TF)
  3. controller_server   — DWB 局部规划 (不倒车)
  4. planner_server      — A* 全局规划
  5. behavior_server     — 恢复行为 (Spin, Wait, ClearCostmap)
  6. bt_navigator        — Behavior Tree 导航引擎
  7. lifecycle_manager   — 自动管理上面节点的生命周期
  8. race_state_machine  — ★ 比赛状态机 (你的 Python 代码)

使用:
  ros2 launch race_nav race_bringup.launch.py
  或指定地图路径:
  ros2 launch race_nav race_bringup.launch.py map_path:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    # ==================== 路径 ====================
    pkg_share = FindPackageShare('race_nav')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])

    # 默认地图路径 — 改成你的实际路径
    default_map = os.path.expanduser('~/maps/hospital_map.yaml')

    # ==================== 参数声明 ====================
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map,
        description='灰度地图 yaml 文件路径'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间 (真实机器人设为 false)'
    )

    # ==================== 节点 ====================

    # 1. Map Server — 加载灰度地图
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map_path')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 2. AMCL — 定位
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'amcl_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 3. Controller Server — DWB（需要 costmap 参数）
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'controller_params.yaml']),
            PathJoinSubstitution([config_dir, 'costmap_common.yaml']),
            PathJoinSubstitution([config_dir, 'local_costmap.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 4. Planner Server — A*（需要 costmap 参数）
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'planner_params.yaml']),
            PathJoinSubstitution([config_dir, 'costmap_common.yaml']),
            PathJoinSubstitution([config_dir, 'global_costmap.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 5. Behavior Server — 恢复行为
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_dir, 'behavior_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 6. BT Navigator — 只加载 BT XML
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {
                'default_bt_xml_filename': PathJoinSubstitution([
                    config_dir, 'navigate_recovery_no_backup.xml'
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # 7. Lifecycle Manager — 自动管理 Nav2 节点的生命周期
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
            },
        ],
    )

    # 8. ★ 比赛状态机 — 延迟 3 秒等 Nav2 全部就绪后再启动
    state_machine_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='race_nav',
                executable='race_state_machine',
                name='race_state_machine',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            )
        ],
    )

    # ==================== Launch ====================
    return LaunchDescription([
        map_path_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        lifecycle_manager_node,
        state_machine_node,
    ])
