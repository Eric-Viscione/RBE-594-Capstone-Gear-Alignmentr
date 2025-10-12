import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# Define the package name
packageName = 'sirgas_description'
 # Absolute package path
pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

# Relative path of the xacro file with respect to the package path
xacroRelativePath = 'urdf/ur20_pba.urdf.xacro'

# RViz config file path respect to the package path
rvizRelativePath = 'rviz/gsm.rviz'

# Relative path of the ros2_control configuration file with respect to the package path
ros2controlRelativePath = 'config/test.yaml'

# Define the new controller manager name
controller_manager_name = '/combined_controller_manager'


# Absolute camera SDF model path
cameraSdfPath = os.path.join(pkgPath, 'meshes/sim_cam/model.sdf')

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
     # --- Camera Launch Logic Starts Here ---

    # Camera Spawner Node
    spawn_sim_cam = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sim_cam',
        output='screen',
        arguments=[
            '-file',
            cameraSdfPath, # Use the absolute path defined above
            '-name',
            'sim_cam',
            '-x', '0', '-y', '0', '-z', '2',
            '-R', '0', '1.57', '-Y', '0' # Note: SDF's pose is roll/pitch/yaw
        ]
    )

    # Timer Action for delay before starting the bridge (2.0 seconds)
    timer_bridge_sim_cam = launch.actions.TimerAction(
        period=2.0,
        actions=[
            # Camera Bridge Node
            launch_ros.actions.Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_sim_cam',
                output='screen',
                arguments=[
                    '/camera_feed/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera_feed/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                ]
            )
        ]
    )
    
    # --- Camera Launch Logic Ends Here ---
    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}])
    
    # RViz Node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
        )
    
    # Joint State Broadcaster
    # Use the specific broadcaster names from the combined_controllers.yaml (test.yaml)
    ur20_jsb_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur20_joint_state_broadcaster', '-c', controller_manager_name])
    
    pba_jsb_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pba_joint_state_broadcaster', '-c', controller_manager_name])
    
    gripper_jsb_spawner = launch_ros.actions.Node(
    package='controller_manager',
    executable='spawner',
    arguments=['gripper_joint_state_broadcaster', '-c', controller_manager_name])
    
    # Velocity Controllers (Ensure you use the names defined in test.yaml: ur20_forward_velocity_controller, pba_velocity_controller)
    pba_v_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pba_velocity_controller', '-c', controller_manager_name])
    
    ur20_v_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur20_forward_velocity_controller', '-c', controller_manager_name])
    
    # Joint Trajectory Controller
    ur20_jt_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur20_joint_trajectory_controller', '-c', controller_manager_name])
    
    # Joint Position Controller
    ur20_p_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur20_forward_position_controller', '-c', controller_manager_name])

    # Gripper Position Controller
    gripper_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_position_controller', '-c', controller_manager_name])
        
    nodeList = [
        gazebo,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        # rviz_node,
        ur20_jsb_spawner, # Use specific ur20 joint state broadcaster
        pba_jsb_spawner, # Use specific pba joint state broadcaster
        gripper_jsb_spawner,
        spawn_sim_cam, # New: Camera Spawner
        timer_bridge_sim_cam, # New: Timer and Camera Bridge
        ur20_jt_controller_spawner,
        # ur20_v_controller_spawner,
        # ur20_p_controller_spawner,
        pba_v_controller_spawner,
        gripper_controller_spawner,
        # ... (include all other necessary controller spawners like scaled_joint_trajectory_controller)
    ]
    
    return launch.LaunchDescription(declared_arguments + nodeList)