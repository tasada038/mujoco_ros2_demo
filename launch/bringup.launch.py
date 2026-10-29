"""Start the MuJoCo viewer and RViz.

  ros2 launch mujoco_ros2_demo bringup.launch.py                      # ant
  ros2 launch mujoco_ros2_demo bringup.launch.py model:=manipulator   # 3-DOF manipulator

The ant walker is started separately:  ros2 run mujoco_ros2_demo ant_walker
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'mujoco_ros2_demo'

# model name -> (MJCF file in <package>/models, RViz config in <package>/rviz)
MODELS = {
    'ant': ('ant_sensors.xml', 'ant.rviz'),
    'manipulator': ('manipulator_3dof.xml', 'manipulator.rviz'),
}


def setup(context):
    model = LaunchConfiguration('model').perform(context)
    if model not in MODELS:
        raise RuntimeError(f"model must be one of {list(MODELS)}, got '{model}'")
    xml, rviz_config = MODELS[model]

    return [
        Node(
            package=PKG,
            executable='mujoco_viewer',
            name='mujoco_viewer',
            parameters=[{'model_path': xml}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(PKG), 'rviz', rviz_config)],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='ant', description='ant | manipulator'),
        OpaqueFunction(function=setup),
    ])