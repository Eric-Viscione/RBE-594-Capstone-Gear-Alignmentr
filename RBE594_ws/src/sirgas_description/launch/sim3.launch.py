import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# Define the package name
packageName = 'sirgas_description'

# ----------------- GEAR STARTER MOTOR (GSM) -----------------
# Relative path of the xacro file with respect to the package path
gsm_xacroRelativePath = 'urdf/gear_starter_motor.urdf.xacro'

# RViz config file path respect to the package path
rvizRelativePath = 'rviz/gsm.rviz'

# Relative path of the ros2_control configuration file with respect to the package path
gsm_ros2controlRelativePath = 'config/gsm_controller.yaml'

# ----------------- PEG BOARD ASSEMBLY (PBA) -----------------
# Relative path of the xacro file with respect to the package path
pba_xacroRelativePath = 'urdf/pba.urdf.xacro'

# Relative path of the ros2_control configuration file with respect to the package path
pba_ros2controlRelativePath = 'config/pba_controller.yaml'


def generate_launch_description():
    # Absolute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

    # ----------------- GEAR STARTER MOTOR (GSM) -----------------
    # Absolute xacro model path
    gsm_xacroModelPath = os.path.join(pkgPath, gsm_xacroRelativePath)
    
    # Absolute ros_control controller path (needed to be passed to spawner)
    gsm_ros2ControllerPath = os.path.join(pkgPath, gsm_ros2controlRelativePath)

    # Get the GSM robot description from the xacro model file
    gsm_robot_desc = xacro.process_file(gsm_xacroModelPath).toxml()
    
    # Define a parameter with the GSM robot xacro description for robot_state_publisher
    robot_description = {'robot_description': gsm_robot_desc}

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
            'gear_starter_motor',
            '-allow_renaming',
            'true',
            '-x',
            '-0.6675'] 
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
    joint_state_broadcaster_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'])
    
    # GSM Velocity Controller Spawner
    gsm_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller', '--ros-args', '--params-file', gsm_ros2ControllerPath])
        
    # PBA Velocity Controller Spawner
    # pba_controller_spawner = launch_ros.actions.Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['forward_velocity_controller', '--ros-args', '--params-file', pba_ros2ControllerPath])
    
    nodeList = [
        gazebo,
        gazebo_bridge,
        gsm_gz_spawn_entity, 
        pba_gz_spawn_entity, 
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        gsm_controller_spawner,
        # pba_controller_spawner
        ]

    return launch.LaunchDescription(declared_arguments + nodeList)