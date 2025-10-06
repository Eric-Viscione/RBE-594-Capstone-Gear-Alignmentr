import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# Define the package name
packageName = 'sirgas_description'

# ----------------- UR20 -----------------
# Relative path of the xacro file with respect to the package path
ur20_xacroRelativePath = 'urdf/ur20_starter.urdf.xacro'

# RViz config file path respect to the package path
# rvizRelativePath = 'rviz/config.rviz'

# Relative path of the ros2_control configuration file with respect to the package path
ur20_ros2controlRelativePath = 'config/ur20_starter_controllers.yaml'

# ----------------- PEG BOARD ASSEMBLY (PBA) -----------------
# Relative path of the xacro file with respect to the package path
pba_xacroRelativePath = 'urdf/pba.urdf.xacro'

# Relative path of the ros2_control configuration file with respect to the package path
pba_ros2controlRelativePath = 'config/pba_controller.yaml'


def generate_launch_description():
    # Absolute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

    # ----------------- UR20 -----------------
    # Absolute xacro model path
    ur20_xacroModelPath = os.path.join(pkgPath, ur20_xacroRelativePath)
    
    # Absolute ros_control controller path (needed to be passed to spawner)
    ur20_ros2ControllerPath = os.path.join(pkgPath, ur20_ros2controlRelativePath)

    # Get the UR20 robot description from the xacro model file
    ur20_robot_desc = xacro.process_file(ur20_xacroModelPath).toxml()
    
    # Define a parameter with the UR20 robot xacro description for robot_state_publisher
    robot_description = {'robot_description': ur20_robot_desc}

    # ----------------- PEG BOARD ASSEMBLY (PBA) -----------------
    # Absolute xacro model path
    pba_xacroModelPath = os.path.join(pkgPath, pba_xacroRelativePath)
    
    # Absolute ros_control controller path (needed to be passed to spawner)
    pba_ros2ControllerPath = os.path.join(pkgPath, pba_ros2controlRelativePath)
    
    # NEW: Get the PBA robot description from the xacro model file
    pba_robot_desc = xacro.process_file(pba_xacroModelPath).toxml() 


    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'))
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Gazebo simulation
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_ros.substitutions.FindPackageShare(package='ros_gz_sim').find('ros_gz_sim'),
                'launch/gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 3 empty.sdf',
            'ros_args': '--ros-args --param use_sim_time:=true'}.items()
    )
    
    # Bridge to ROS topics (Clock is for use_sim_time)
    gazebo_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    # GSM Spawn: Spawns the GSM robot using the shared robot_description topic
    # The -x argument is added here to set the default initial position.
    gsm_gz_spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'ur20_starter',
            '-allow_renaming',
            'true',
            '-y',
            '1.0'] 
    )
    
    # PBA Spawn: Spawns the PBA robot using the description string directly
    pba_gz_spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            pba_robot_desc,
            '-name',
            'peg_board_assembly',
            '-allow_renaming',
            'true']
    )
    
    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])
    
    # Joint State Broadcaster (Handles all robots' joint state broadcasters)
    ur20_joint_state_broadcaster_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/ur20_starter_controller_manager'],
        output='screen',
        )
    
    pba_joint_state_broadcaster_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/pba_controller_manager'],
        output='screen',
        )
    
    #  UR20 Controller Spawner
    ur20_jt_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '-c', 
            '/ur20_starter_controller_manager',  # <-- Specify the custom controller manager
        ]
    )

    ur20_v_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_velocity_controller',
            '-c', 
            '/ur20_starter_controller_manager',  # <-- Specify the custom controller manager
        ]
    )

    ur20_p_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '-c', 
            '/ur20_starter_controller_manager',  # <-- Specify the custom controller manager
        ]
    )
        
    # PBA Velocity Controller Spawner
    pba_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_velocity_controller', 
            '-c', 
            '/pba_controller_manager', # <-- Specify the custom controller manager
        ]
    )
    
    nodeList = [
        gazebo,
        gazebo_bridge,
        gsm_gz_spawn_entity, 
        pba_gz_spawn_entity, 
        robot_state_publisher_node,
        pba_joint_state_broadcaster_node,
        ur20_joint_state_broadcaster_node,
        pba_joint_state_broadcaster_node,
        ur20_jt_controller_spawner,
        ur20_v_controller_spawner,
        ur20_p_controller_spawner,
        pba_controller_spawner,
        ]

    return launch.LaunchDescription(declared_arguments + nodeList)