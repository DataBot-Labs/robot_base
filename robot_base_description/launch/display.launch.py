import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
import os
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    package_name = 'robot_base_description'
    robot_model_path = 'urdf/robot_base.xacro'
    rviz_config_path = 'config/rviz_default.rviz'
    teleop_twist_joy_config_path = 'config/ps4.config.yaml'
    robot_controllers_path = "config/hoverboard_controllers.yaml"
    
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_robot_model_path = os.path.join(pkg_share, robot_model_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_path)
    default_teleop_twist_joy_config_path = os.path.join(pkg_share, teleop_twist_joy_config_path)
    default_robot_controllers_path = os.path.join(pkg_share, robot_controllers_path)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    teleop_twist_joy_config_file = LaunchConfiguration("teleop_twist_joy_config_file")
    robot_controllers_file = LaunchConfiguration("robot_controllers_file")
   
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_model_path = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_robot_model_path, 
        description='Absolute path to robot urdf file')

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    declare_teleop_twist_joy_config_file = DeclareLaunchArgument(
        name='teleop_twist_joy_config_file',
        default_value=default_teleop_twist_joy_config_path,
        description='Absolute path to teleop twist joy config file')
        
    declare_robot_controllers = DeclareLaunchArgument(
        name="robot_controllers_file",
        default_value=default_robot_controllers_path,
        description='Absolute path to controllers config file')
    
    start_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': Command(['xacro ', urdf_model])},
                    robot_controllers_file],
        output="both",
    )
    
    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)}]
    )
    
    start_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    start_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hoverboard_base_controller", "-c", "/controller_manager"],
    )
    
    start_joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 30.0}],
    )
        
    start_teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        remappings=[
            ('/cmd_vel', '/hoverboard_base_controller/cmd_vel_unstamped'),]
    )
    
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
        
    start_lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB2',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True,}]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_model_path)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(declare_robot_controllers)
    ld.add_action(declare_teleop_twist_joy_config_file)
    
    ld.add_action(start_control_node)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_joint_state_broadcaster_spawner)
    ld.add_action(start_robot_controller_spawner)
    ld.add_action(start_joy_node)
    ld.add_action(start_teleop_twist_joy_node)
    ld.add_action(start_rviz_node)
    
    # Uncomment this line if you want to start the lidar node
    # ld.add_action(start_lidar_node)
    
    return ld

