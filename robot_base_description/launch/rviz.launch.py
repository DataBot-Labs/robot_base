import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_base_description').find('robot_base_description')
    default_model_path = os.path.join(pkg_share, 'urdf/robot_base.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_default.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters= [{'use_sim_time': use_sim_time}],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
    ])