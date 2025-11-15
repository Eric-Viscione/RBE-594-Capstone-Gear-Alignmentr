import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

# Define the package name
packageName = 'test_ws'
test_ws_path = FindPackageShare('test_ws').find('test_ws')
sirgas_path  = FindPackageShare('sirgas_description').find('sirgas_description')
 # Absolute package path
pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
#Relative path of the apriltag xacro 

# Relative path of the xacro file with respect to the package path
xacroRelativePath = os.path.join('config', 'panda_pba_robots.urdf.xacro')
# Absolute camera SDF model path

sirgas_share = get_package_share_directory('sirgas_description')
cameraSdfPath = os.path.join(sirgas_share, 'meshes', 'sim_cam', 'model.sdf')
# RViz config file path respect to the package path
rvizConfigPath = os.path.join(pkgPath, 'config', 'moveit.rviz')

# Define the new controller manager name
controller_manager_name = '/combined_controller_manager'

# Absolute xacro model path for the cube
cubeXacroPath = os.path.join(sirgas_path, 'urdf', 'cube.urdf.xacro')

# Get the cube description from the xacro model file
cube_desc = xacro.process_file(cubeXacroPath).toxml()

# Define a parameter with the cube xacro description
cube_description = {'cube_description': cube_desc}

'''-----------Gear Paths-----------'''
# Absolute xacro model path for the Starter Gear
starter_gearXacroPath = os.path.join(sirgas_path, 'urdf', 'starter_gear.urdf.xacro')

# Get the Starter Gear description from the xacro model file
starter_gear_desc = xacro.process_file(starter_gearXacroPath).toxml()

# Define a parameter with the Starter Gear xacro description
starter_gear_description = {'starter_gear_description': starter_gear_desc}

# Absolute xacro model path for the First Gear
first_gearXacroPath = os.path.join(sirgas_path, 'urdf', 'first_gear.urdf.xacro')

# Get the First Gear description from the xacro model file
first_gear_desc = xacro.process_file(first_gearXacroPath).toxml()

# Define a parameter with the First Gear xacro description
first_gear_description = {'first_gear_description': first_gear_desc}

# Absolute xacro model path for the Second Gear
second_gearXacroPath = os.path.join(sirgas_path, 'urdf', 'second_gear.urdf.xacro')

# Get the Second Gear description from the xacro model file
second_gear_desc = xacro.process_file(second_gearXacroPath).toxml()

# Define a parameter with the Second Gear xacro description
second_gear_description = {'second_gear_description': second_gear_desc}

# Absolute xacro model path for the Third Gear
third_gearXacroPath = os.path.join(sirgas_path, 'urdf', 'third_gear.urdf.xacro')

# Get the Third Gear description from the xacro model file
third_gear_desc = xacro.process_file(third_gearXacroPath).toxml()

# Define a parameter with the Third Gear xacro description
third_gear_description = {'second_gear_description': third_gear_desc}


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
    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{test_ws_path}/meshes:{sirgas_path}/meshes:' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
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
            '-file', cameraSdfPath,
            '-name', 'sim_cam',
            '-x','0','-y','0','-z','2',
            '-R','0','-P','0','-Y','0'
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
    # Spawn cube
    gz_spawn_cube = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            cube_desc,
            '-name',
            'pickup_cube',
            '-allow_renaming',
            'true',
            # Set the initial position (e.g., at x=0, y=-1.0, z=0.025 for a 0.05m cube)
            '-x','0','-y','-1.0','-z','0.025',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gz_spawn_starter_gear = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            starter_gear_desc,
            '-name',
            'starter_gear',
            '-allow_renaming',
            'true',
            '-x','-0.375','-y','0.0','-z','0.125' 
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gz_spawn_first_gear = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            first_gear_desc,
            '-name',
            'first_gear',
            '-allow_renaming',
            'true',
            '-x','0.0','-y','-1.0','-z','0.025'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gz_spawn_second_gear = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            second_gear_desc,
            '-name',
            'second_gear',
            '-allow_renaming',
            'true',
            '-x','0.25','-y','0.0','-z','0.125' 
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gz_spawn_third_gear = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            third_gear_desc,
            '-name',
            'third_gear',
            '-allow_renaming',
            'true',
            '-x','0.45','-y','0.0','-z','0.125' 
        ],
        parameters=[{'use_sim_time': use_sim_time}]
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
    green_detector_node = launch_ros.actions.Node(
        package='sirgas_apriltag_detector', # Replace with your actual package name if different
        executable='black_tag_detector',
        name='green_tag_detector', # Unique name
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # Configuration for Green (H=50-70)
            {'color_mode': "hsv"},
            {'hsv_low': [50, 100, 100]},
            {'hsv_high': [70, 255, 255]},
            {'image_topic': '/camera_feed/image'},
            # Set unique output topics
            {'pose_topic_cam': '/tag_pose_cam/green'},
            {'pose_topic_world': '/tag_pose_world/green'},
            {'long_axis_cam_topic': '/tag_long_axis_cam/green'},
            {'long_axis_world_topic': '/tag_long_axis_world/green'},
            # Optional debug settings
            {'save_debug_images': True},
            {'debug_images_dir': '~/RBE594_ws/debug_images/green'},
            {'debug_save_every_n': 5}
        ]
    )
    
    # 2. Black Tag Detector (Grayscale 'dark' mode)
    black_detector_node = launch_ros.actions.Node(
        package='sirgas_apriltag_detector', # Replace with your actual package name if different
        executable='black_tag_detector',
        name='black_tag_detector', # Unique name
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # Configuration for Black/Dark Contrast (default)
            {'color_mode': "dark"},
            {'t_dark': 20}, # Dark threshold in [0, 255]
            {'image_topic': '/camera_feed/image'},
            # Set unique output topics
            {'pose_topic_cam': '/tag_pose_cam/black'},
            {'pose_topic_world': '/tag_pose_world/black'},
            {'long_axis_cam_topic': '/tag_long_axis_cam/black'},
            {'long_axis_world_topic': '/tag_long_axis_world/black'},
            # Optional debug settings
            {'save_debug_images': True},
            {'debug_images_dir': '~/RBE594_ws/debug_images/black'},
            {'debug_save_every_n': 5}
        ]
    )
    
    nodeList = [
        set_gz_resources,
        gazebo,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        move_group_launch,
        rviz_node,
        panda_jsb_spawner,
        pba_jsb_spawner, # Use specific pba joint state broadcaster
        spawn_sim_cam, # 
        timer_bridge_sim_cam, # 
        pba_v_controller_spawner,
        arm_controller,
        hand_controller,
        # gz_spawn_cube,
        # gz_spawn_starter_gear,
        gz_spawn_first_gear,
        green_detector_node,
        black_detector_node
        # gz_spawn_second_gear,
        # gz_spawn_third_gear,
    ]
    
    return launch.LaunchDescription(declared_arguments + nodeList)