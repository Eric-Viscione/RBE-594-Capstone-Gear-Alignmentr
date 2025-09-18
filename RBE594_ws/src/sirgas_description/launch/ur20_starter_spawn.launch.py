import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    ros_distro_value = LaunchConfiguration("ros_distro").perform(context)

    # Determine Gazebo and spawn executables based on ROS distribution
    gazebo_package = ''
    gazebo_launch_executable = ''
    spawn_executable = ''
    world_arg = {}

    if ros_distro_value == 'humble':
        gazebo_package = 'gazebo_ros'
        gazebo_launch_executable = 'gazebo.launch.py'
        spawn_executable = 'spawn_entity.py'
        world_arg = {'world': world_file}
    else:  # Default to Jazzy/Rolling
        gazebo_package = 'ros_gz_sim'
        gazebo_launch_executable = 'gz_sim.launch.py'
        spawn_executable = 'create'
        world_arg = {'gz_args': [" -r -v 4 ", world_file]}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "ur20_view.rviz"]
    )

    xacro_file_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    ).perform(context)

    # Use subprocess to process the xacro file
    xacro_args = [
        f"safety_limits:={safety_limits.perform(context)}",
        f"safety_pos_margin:={safety_pos_margin.perform(context)}",
        f"safety_k_position:={safety_k_position.perform(context)}",
        f"name:=ur",
        f"ur_type:={ur_type.perform(context)}",
        f"prefix:={prefix.perform(context)}",
        f"sim_ignition:=false" if ros_distro_value == 'humble' else f"sim_ignition:=true",
        f"sim_gazebo:=true" if ros_distro_value == 'humble' else f"sim_gazebo:=false",
        f"simulation_controllers:={initial_joint_controllers.perform(context)}",
    ]

    try:
        robot_description_content = subprocess.check_output(
            ['xacro', xacro_file_path] + xacro_args,
            stderr=subprocess.STDOUT
        ).decode('utf-8')
    except subprocess.CalledProcessError as e:
        print(f"Failed to process xacro file: {e.output.decode('utf-8')}")
        raise

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )
    
    # Conditional Gazebo launch and robot spawning based on ROS distribution
    if ros_distro_value == 'humble':
        gz_spawn_entity = Node(
            package=gazebo_package,
            executable=spawn_executable,
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", "ur",
            ],
        )

        gz_launch_description_with_gui = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(gazebo_package), "launch", gazebo_launch_executable)
            ),
            launch_arguments={"gui": "true", "world": world_file}.items(),
            condition=IfCondition(gazebo_gui),
        )

        gz_launch_description_without_gui = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(gazebo_package), "launch", gazebo_launch_executable)
            ),
            launch_arguments={"gui": "false", "world": world_file}.items(),
            condition=UnlessCondition(gazebo_gui),
        )
        
        nodes_to_start = [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            initial_joint_controller_spawner_stopped,
            initial_joint_controller_spawner_started,
            gz_spawn_entity,
            gz_launch_description_with_gui,
            gz_launch_description_without_gui,
        ]
    else: # Jazzy/Ignition
        gz_spawn_entity = Node(
            package=gazebo_package,
            executable=spawn_executable,
            output="screen",
            arguments=[
                "-string",
                robot_description_content,
                "-name",
                "ur",
                "-allow_renaming",
                "true",
            ],
        )
        gz_launch_description_with_gui = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
            condition=IfCondition(gazebo_gui),
        )
        gz_launch_description_without_gui = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
            condition=UnlessCondition(gazebo_gui),
        )
        gz_sim_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            ],
            output="screen",
        )
        nodes_to_start = [
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            initial_joint_controller_spawner_stopped,
            initial_joint_controller_spawner_started,
            gz_spawn_entity,
            gz_launch_description_with_gui,
            gz_launch_description_without_gui,
            gz_sim_bridge,
        ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # ROS Distribution Argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros_distro",
            default_value="humble",
            description='ROS 2 distribution to use ("humble" or "jazzy").',
        )
    )
    
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur20",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="sirgas_description",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur20_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("sirgas_description"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="sirgas_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur20_starter.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.world", # Changed to empty.world for Humble
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])