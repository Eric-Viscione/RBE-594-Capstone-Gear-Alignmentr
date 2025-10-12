import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# Define the package name
packageName = 'sirgas_description'

# Relative path of the xacro file with respect to the package path
xacroRelativePath = 'urdf/robotiq_arg85_description.urdf.xacro'

# RViz config file path respect to the package path
rvizRelativePath = 'rviz/gsm.rviz'

# Relative path of the ros2_control configuration file with respect to the package path
ros2controlRelativePath = 'config/gsm_controller.yaml'

def generate_launch_description():
    # Absolute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

    # Absolute xacro model path
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)

    # Absolute RViz config file path
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)

    # Absolute ros_control controller path
    ros2ControllerPath = os.path.join(pkgPath, ros2controlRelativePath)

    # Get the robot description from the xacro model file
    robot_desc = xacro.process_file(xacroModelPath).toxml()

    # Define a parameter with the robot xacro description
    robot_description = {'robot_description': robot_desc}

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(name='gui', default_value='true',
                                             description='Start the RViz2 GUI.')
    )

    # Initialize Arguments
    gui = LaunchConfiguration('gui')

    # For starting Gazebo
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments=[('gz_args', ' -r -v 3 empty.sdf')],
        condition=launch.conditions.IfCondition(gui))
    
    # Gazebo Bridge
    gazebo_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')
    
    gz_spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'robot_system_position',
            '-allow_renaming',
            'true'])
    
    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])
    
    # RViz Node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
        )
    
    # Joint State Broadcaster
    joint_state_broadcaster_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'])
    
    # Velocity Controller
    robot_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'])
    
    nodeList = [
        gazebo,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        # rviz_node,
        joint_state_broadcaster_node,
        # robot_controller_spawner,
        ]
    
    return launch.LaunchDescription(declared_arguments + nodeList)