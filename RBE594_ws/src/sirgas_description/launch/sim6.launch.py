import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

# Define the package name
packageName = 'test_ws'

 # Absolute package path
pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
#Relative path of the apriltag xacro 

# Relative path of the xacro file with respect to the package path
xacroRelativePath = os.path.join(pkgPath, 'config', 'panda_pba_robots.urdf.xacro')

# Absolute camera SDF model path
cameraSdfPath = os.path.join(pkgPath, 'meshes/sim_cam/model.sdf')

# RViz config file path respect to the package path
rvizConfigPath = os.path.join(pkgPath, 'config', 'moveit.rviz')

# Define the new controller manager name
controller_manager_name = '/combined_controller_manager'


def generate_launch_description():
    # Absolute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)

    # Absolute xacro model path
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)

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
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                             description='Use simulation (Gazebo) clock if true')
    )

    # Initialize Arguments
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Moveit Config
    moveit_config = (
        MoveItConfigsBuilder('panda_pba_robots', package_name='test_ws')
        .robot_description(file_path='config/panda_pba_robots.urdf.xacro')
        .robot_description_semantic(file_path='config/panda_pba_robots.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .sensors_3d('config/sensors_3d.yaml')
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True, 
            publish_planning_scene=True, publish_geometry_updates=True,
            publish_state_updates=True, publish_transforms_updates=True
        )
        .planning_pipelines(
            pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner']
        )
        .to_moveit_configs()
    )

    move_group_launch = generate_move_group_launch(moveit_config)

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
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        )
    
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
            'true'],
        parameters=[{'use_sim_time': use_sim_time}]
            )
            
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
            '-R', '0', 'P' ,'1.57', '-Y', '0' # Note: SDF's pose is roll/pitch/yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}]
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
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # --- Camera Launch Logic Ends Here ---
    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath],
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time}
        ],
    )

    # Joint State Broadcasters
    panda_jsb_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_joint_state_broadcaster', '-c', controller_manager_name],
        parameters=[{'use_sim_time': use_sim_time}])

    pba_jsb_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pba_joint_state_broadcaster', '-c', controller_manager_name],
        parameters=[{'use_sim_time': use_sim_time}])

    # Panda Controllers for Gazebo
    arm_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '-c', controller_manager_name],
        parameters=[{'use_sim_time': use_sim_time}])
    
    hand_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller', '-c', controller_manager_name],
        parameters=[{'use_sim_time': use_sim_time}])
    
    # PBA Controller for Gazebo
    pba_v_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pba_velocity_controller', '-c', controller_manager_name],
        parameters=[{'use_sim_time': use_sim_time}])
    
    nodeList = [
        gazebo,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        move_group_launch,
        rviz_node,
        panda_jsb_spawner,
        pba_jsb_spawner, # Use specific pba joint state broadcaster
        spawn_sim_cam, # New: Camera Spawner
        timer_bridge_sim_cam, # New: Timer and Camera Bridge
        pba_v_controller_spawner,
        arm_controller,
        hand_controller,
        # ... (include all other necessary controller spawners like scaled_joint_trajectory_controller)
    ]
    
    return launch.LaunchDescription(declared_arguments + nodeList)