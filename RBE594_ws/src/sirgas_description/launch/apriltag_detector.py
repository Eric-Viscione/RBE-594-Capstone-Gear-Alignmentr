import launch
import launch_ros

def generate_launch_description():
    """
    Launches the AprilTag Detector node.
    Requires the image bridge to be running (from sim4.launch.py).
    """
    
    apriltag_detector_node = launch_ros.actions.Node(
        package='sirgas_description',
        executable='apriltag_detector', # This name comes from the setup.py entry_points
        name='apriltag_detector',
        output='screen',
        emulate_tty=True
    )

    return launch.LaunchDescription([
        apriltag_detector_node,
    ])
