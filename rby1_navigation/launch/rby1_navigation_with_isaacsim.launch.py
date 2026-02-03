# RBy1 Navigation with Isaac Sim Launch File
# Executes isaac-sim.sh with --exec to load USD and play simulation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # USD Scene - 0926Final.usd (relative to workspace src directory)
    # install/rby1_navigation/share/rby1_navigation -> go up 4 levels to workspace root, then into src
    rby1_nav_dir = get_package_share_directory("rby1_navigation")
    # rby1_nav_dir = /home/.../rby1_ros2_ws/install/rby1_navigation/share/rby1_navigation
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(rby1_nav_dir))))
    usd_path = os.path.join(workspace_root, "src", "RBY1-robocup2024.usd")

    # Isaac Sim path
    isaac_sim_path = os.path.expanduser("~/isaacsim/_build/linux-x86_64/release/isaac-sim.sh")

    # Navigation parameters
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    
    # Use actual file names
    map_file = os.path.join(rby1_nav_dir, "maps", "athome_navigation.yaml")
    param_file = os.path.join(rby1_nav_dir, "params", "rby1_navigation_params.yaml")
    
    # Script to load USD and play
    load_script = os.path.join(rby1_nav_dir, "scripts", "load_usd_and_play.py")
    
    # Use nav2_bringup default rviz config
    rviz_config = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    # Nodes to launch after Isaac Sim is ready
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "rviz_launch.py")),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "rviz_config": rviz_config
        }.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "true",
            "params_file": param_file
        }.items(),
    )

    # Convert 3D PointCloud to 2D LaserScan for AMCL (Ouster is 3D LiDAR)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': -0.1,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,  # ~720 points
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/point_cloud'),
            ('scan', '/laser_scan'),
        ],
        output='screen',
    )

    # AMCL will publish map->odom TF (tf_broadcast: true in params)
    # No static transform needed

    # Signal file that Isaac Sim creates when ready
    signal_file = "/tmp/isaac_sim_ready"
    
    # Remove old signal file before starting
    if os.path.exists(signal_file):
        os.remove(signal_file)

    # Isaac Sim process with --exec to load USD and play
    # Pass USD path via environment variable
    isaac_sim_process = ExecuteProcess(
        cmd=[
            isaac_sim_path,
            '--exec', load_script,
        ],
        name='isaac_sim',
        output='screen',
        shell=False,
        additional_env={'ISAAC_USD_PATH': usd_path},
    )

    # Watcher script that monitors signal file and launches Nav2 when ready
    watcher_script = f'''
import os
import time
import subprocess
import sys

signal_file = "{signal_file}"
print("[watcher] Waiting for Isaac Sim to be ready...")

while not os.path.exists(signal_file):
    time.sleep(1)

print("[watcher] Isaac Sim is ready! Signal file detected.")
print("[watcher] Launching Nav2 and RViz...")
sys.exit(0)
'''

    # Watcher process
    watcher_process = ExecuteProcess(
        cmd=['python3', '-c', watcher_script],
        name='isaac_sim_watcher',
        output='screen',
        shell=False,
    )

    # Event handler: when watcher exits (signal detected), launch Nav2 and pointcloud_to_laserscan
    nav2_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=watcher_process,
            on_exit=[pointcloud_to_laserscan_node, rviz_launch, nav2_bringup_launch]
        )
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Isaac Sim) clock"
        ),
        DeclareLaunchArgument(
            "map",
            default_value=map_file,
            description="Full path to map yaml file"
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=param_file,
            description="Full path to nav2 param file"
        ),

        # Launch Isaac Sim with script to load USD and play
        isaac_sim_process,

        # Watcher that monitors for Isaac Sim ready signal
        watcher_process,

        # Event: Launch Nav2 when watcher detects signal
        nav2_event_handler,
    ])
