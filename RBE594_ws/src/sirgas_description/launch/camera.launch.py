import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Define the package name
packageName = 'sirgas_description'

def generate_launch_description():
    # --- 1. Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # --- 2. File Paths ---
    pkg_share_dir = get_package_share_directory(packageName)
    
    # Absolute xacro model path
    xacro_model_path = os.path.join(pkg_share_dir, 'urdf', 'camera.urdf.xacro')
    
    # Absolute RViz config file path
    rviz_config_path = os.path.join(pkg_share_dir, 'rviz', 'camera.rviz')
    
    # Get Gazebo Sim launch file
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- 3. Process Xacro via Command ---
    robot_description = {'robot_description': Command(['xacro ', xacro_model_path])}

    # --- 4. Robot State Publisher (TF) ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # --- 5. Launch Gazebo Sim (Ignition) ---
    gazebo = IncludeLaunchDescription(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        launch_arguments=[('gz_args', '-r -v 3 empty.sdf')]
    )
    
    # --- 6. Spawn the Entity in Gazebo Sim ---
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'camera_model',
                   '-allow_renaming', 'true']
    )
    
    # --- 7. CORRECT Gazebo-ROS Bridge Configuration ---
    gazebo_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
               # FIX: Map GZ's default image topic (image) to ROS's desired topic (image_raw)
               '/camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image@/camera_sensor/image_raw',
               # Bridge the CameraInfo topic
               '/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
    output='screen'
)
    
    # --- 9. Image Viewer for Debugging ---
    image_view_node = Node(
        package='image_tools',
        executable='showimage',
        name='camera_image_view',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/image', '/camera/image_raw')],
        # Start after a delay to ensure camera is ready
        condition=launch.conditions.IfCondition(use_rviz)
    )
    
    # --- 10. RViz Node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )
    
    # --- 11. Debug: Topic Lister ---
    topic_lister = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5; ros2 topic list | grep -E "(camera|gz)"'],
        output='screen',
        shell=True
    )
    
    # --- 12. Return Launch Description ---
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Whether to launch RViz2.'),
            
        # Execution
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gazebo_bridge,
        rviz_node,
        topic_lister,
    ])