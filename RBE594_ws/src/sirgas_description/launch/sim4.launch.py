import os
from ament_index_python.packages import get_package_share_directory   # ✅ add
import launch
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
import launch_ros
import xacro

packageName = "sirgas_description"
pkg_share = get_package_share_directory(packageName)
cameraSdfPath = os.path.join(pkg_share, "meshes/sim_cam/model.sdf")

xacroRelativePath = "urdf/ur20_pba.urdf.xacro"
rvizRelativePath  = "rviz/gsm.rviz"
ros2controlRelativePath = "config/test.yaml"
controller_manager_name = "/combined_controller_manager"

def generate_launch_description():
    xacroModelPath     = os.path.join(pkg_share, xacroRelativePath)
    rvizConfigPath     = os.path.join(pkg_share, rvizRelativePath)
    ros2ControllerPath = os.path.join(pkg_share, ros2controlRelativePath)

    meshes_dir = os.path.join(pkg_share, "meshes")
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{meshes_dir}:" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )
    set_ign_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=f"{meshes_dir}:" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    )

    robot_desc = xacro.process_file(xacroModelPath).toxml()
    robot_description = {"robot_description": robot_desc}

    declared_arguments = [
        launch.actions.DeclareLaunchArgument(
            name="gui", default_value="true", description="Start the RViz2 GUI."
        )
    ]
    gui = LaunchConfiguration("gui")

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=launch.conditions.IfCondition(gui),
    )

    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen",
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "robot_system_position",
            "-allow_renaming", "true",
        ],
    )

    spawn_sim_cam = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_sim_cam",
        output="screen",
        arguments=[
            "-file", cameraSdfPath,
            "-name", "sim_cam",
            "-x", "0", "-y", "0", "-z", "2",
            "-R", "0", "-P", "0", "-Y", "0"
        ],
    )

   # spawn_apriltag = launch_ros.actions.Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     name="spawn_apriltag_tag",
    #     output="screen",
    #     arguments=[
    #         "-file", "model://Apriltag36_11_00000",
    #         "-name", "apriltag_block",
    #         "-x", "0.5", "-y", "0.5", "-z", "0.05",
    #         "-R", "0", "-P", "0", "-Y", "0"
    #     ]
    # )

    timer_bridge_sim_cam_independent = launch.actions.TimerAction(
        period=2.0,
        actions=[launch_ros.actions.Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="bridge_sim_cam",
            output="screen",
            arguments=[
                "/camera_feed/image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera_feed/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            ],
        )],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="screen", arguments=["-d", rvizConfigPath],
    )

    ur20_jsb_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["ur20_joint_state_broadcaster", "-c", controller_manager_name],
    )
    pba_jsb_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["pba_joint_state_broadcaster", "-c", controller_manager_name],
    )
    pba_v_controller_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["pba_velocity_controller", "-c", controller_manager_name],
    )
    ur20_v_controller_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["ur20_forward_velocity_controller", "-c", controller_manager_name],
    )
    ur20_jt_controller_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["ur20_joint_trajectory_controller", "-c", controller_manager_name],
    )
    ur20_p_controller_spawner = launch_ros.actions.Node(
        package="controller_manager", executable="spawner",
        arguments=["ur20_forward_position_controller", "-c", controller_manager_name],
    )

    nodeList = [
        set_gz_path,            
        set_ign_path,           
        gazebo,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        ur20_jsb_spawner,
        pba_jsb_spawner,
        spawn_sim_cam,
        timer_bridge_sim_cam_independent,
        ur20_jt_controller_spawner,
        # ur20_v_controller_spawner,
        pba_v_controller_spawner,
        # ur20_p_controller_spawner,
        # spawn_apriltag,       
    ]

    return launch.LaunchDescription(declared_arguments + nodeList)
