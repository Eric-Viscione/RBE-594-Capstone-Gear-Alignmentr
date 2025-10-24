#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# The PBA robot's starter joint has a velocity command interface.
# We will publish to its command topic to make it move.

# Based on the launch file and xacro, the controller is 'pba_velocity_controller'
# and the controller manager is '/combined_controller_manager'.
CONTROLLER_COMMAND_TOPIC = '/pba_velocity_controller/commands'

class PBARobotVelocityController(Node):
    """
    A node to publish velocity commands to the PBA robot's velocity controller.
    """
    def __init__(self):
        super().__init__('pba_robot_velocity_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, CONTROLLER_COMMAND_TOPIC, 10)
        self.get_logger().info(f"PBA Velocity Publisher created on topic: {CONTROLLER_COMMAND_TOPIC}")

    def send_velocity_command(self, velocity):
        """
        Sends a Float64 message to the controller command topic.
        :param velocity: The target joint velocity (rad/s).
        """
        msg = Float64MultiArray()
        msg.data = [float(velocity)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity command: {velocity} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PBARobotVelocityController()
        
        # --- Movement Sequence ---
        
        # 1. Move forward at 1.0 rad/s for 3 seconds
        move_time_sec = 3
        velocity_forward = 1.0
        
        node.get_logger().info(f"Moving PBA starter joint forward at {velocity_forward} rad/s for {move_time_sec}s")
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < move_time_sec:
            node.send_velocity_command(velocity_forward)
            rclpy.spin_once(node, timeout_sec=0.1) # Publish at ~10Hz

        # 2. Stop for 1 second
        node.get_logger().info("Stopping PBA starter joint.")
        node.send_velocity_command(0.0)
        time.sleep(1)

        # 3. Move backward at -0.5 rad/s for 2 seconds
        move_time_sec = 2
        velocity_backward = -0.5
        
        node.get_logger().info(f"Moving PBA starter joint backward at {velocity_backward} rad/s for {move_time_sec}s")
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < move_time_sec:
            node.send_velocity_command(velocity_backward)
            rclpy.spin_once(node, timeout_sec=0.1) # Publish at ~10Hz
            
        # 4. Final Stop
        node.get_logger().info("Final stop for PBA starter joint.")
        node.send_velocity_command(0.0)
        
        node.get_logger().info("PBA robot movement demonstration complete.")

    except Exception as e:
        if node:
            node.get_logger().error(f"An error occurred: {e}")
        else:
            print(f"An error occurred during initialization: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
