#'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the package name from the launch argument
    pkg_name = LaunchConfiguration('package_name').perform(context)
    
    # Get the URDF file path
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'pba.urdf.xacro'
    )
    
    # Check if the URDF file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # Process the xacro file to get the URDF content
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # Gazebo launch
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_file_path = os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'peg_board_assembly'],
        output='screen'
    )

    # Node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # RVIZ launch
    # We will declare a separate RVIZ config file to load
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'config.rviz'
    )
    
    # A robot state publisher is required to make RVIZ work with the model.
    # We will spawn a joint state publisher to work along with the robot state publisher.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return [
        gazebo_server,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_publisher_node,
        rviz_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'package_name',
            default_value='sirgas_description',
            description='Name of the package containing the URDF.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
'''

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the package name from the launch argument
    pkg_name = LaunchConfiguration('package_name').perform(context)
    
    # Define URDF paths using find package share for robustness
    pba_urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'pba.urdf.xacro')
    ur_urdf_path = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.urdf.xacro')
    
    # Get the explicit path to the xacro executable
    xacro_path = os.path.join(get_package_prefix('xacro'), 'bin', 'xacro')

    # Process URDF files using the Command substitution for reliability
    pba_description_content = Command([xacro_path, '--inorder', pba_urdf_path])
    ur20_left_description_content = Command([
        xacro_path,
        '--inorder',
        ur_urdf_path,
        'ur_type:=ur20',
        'name:=ur20_left',
        'tf_prefix:=ur20_left',
        'robot_ip:=1.2.3.4'
    ])
    ur20_right_description_content = Command([
        xacro_path,
        '--inorder',
        ur_urdf_path,
        'ur_type:=ur20',
        'name:=ur20_right',
        'tf_prefix:=ur20_right',
        'robot_ip:=1.2.3.4'
    ])

    # Gazebo launch
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_file_path = os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Peg board assembly nodes
    peg_board_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='peg_board_state_publisher',
        output='screen',
        parameters=[{'robot_description': pba_description_content}],
        remappings=[('/robot_description', '/peg_board_assembly/robot_description')]
    )

    spawn_peg_board = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/peg_board_assembly/robot_description', 
            '-entity', 'peg_board_assembly',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'
        ],
        output='screen'
    )

    # Left UR20 robot nodes
    ur20_left_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ur20_left_description_content, 'use_sim_time': True}],
        namespace='ur20_left'
    )

    spawn_ur20_left = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_ur20_left',
        arguments=[
            '-entity', 'ur20_left',
            '-topic', '/ur20_left/robot_description',
            '-x', '-0.5',
            '-y', '0.5',
            '-z', '0.0',
            '--ros-args', '--remap', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    ur20_left_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='ur20_left'
    )
    
    ur20_left_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('ur_bringup'), 'rviz', 'ur.rviz')],
        namespace='ur20_left'
    )

    # Right UR20 robot nodes
    ur20_right_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ur20_right_description_content, 'use_sim_time': True}],
        namespace='ur20_right'
    )

    spawn_ur20_right = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity_ur20_right',
        arguments=[
            '-entity', 'ur20_right',
            '-topic', '/ur20_right/robot_description',
            '-x', '0.5',
            '-y', '-0.5',
            '-z', '0.0',
            '--ros-args', '--remap', 'use_sim_time:=true'
        ],
        output='screen'
    )
    
    ur20_right_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='ur20_right'
    )
    
    ur20_right_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('ur_bringup'), 'rviz', 'ur.rviz')],
        namespace='ur20_right'
    )

    return [
        gazebo_server,
        peg_board_state_publisher,
        spawn_peg_board,
        ur20_left_state_publisher,
        ur20_left_joint_publisher,
        ur20_left_rviz,
        spawn_ur20_left,
        ur20_right_state_publisher,
        ur20_right_joint_publisher,
        ur20_right_rviz,
        spawn_ur20_right
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'package_name',
            default_value='sirgas_description',
            description='Name of the package containing the URDF file'
        ),
        OpaqueFunction(function=launch_setup)
    ])

'''