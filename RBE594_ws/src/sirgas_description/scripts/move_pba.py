#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState # Import the JointState message type
import time
import matplotlib.pyplot as plt

# The PBA robot's starter joint has a velocity command interface.
# We will publish to its command topic to make it move.
CONTROLLER_COMMAND_TOPIC = '/pba_velocity_controller/commands'

# The standard topic for all joint state information, including effort/torque
JOINT_STATES_TOPIC = '/joint_states' 

# The specific joint name we are interested in
TARGET_JOINT_NAME = 'pb_starter_joint' 

class PBARobotVelocityController(Node):
    """
    A node to publish velocity commands to the PBA robot's velocity controller
    and subscribe to joint effort/torque feedback via /joint_states.
    """
    def __init__(self):
        super().__init__('pba_robot_velocity_publisher')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Float64MultiArray, CONTROLLER_COMMAND_TOPIC, 10)
        self.get_logger().info(f"PBA Velocity Publisher created on topic: {CONTROLLER_COMMAND_TOPIC}")

        # Subscriber for JointState feedback
        self.subscription = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self.joint_states_callback,
            10
        )
        self.get_logger().info(f"PBA JointState Subscriber created on topic: {JOINT_STATES_TOPIC}")
        
        # Data storage for plotting
        self.time_data = []
        self.torque_data = []
        self.start_time = time.time() # To track relative time
        self.is_recording = True # Flag to control recording

    def send_velocity_command(self, velocity):
        """
        Sends a Float64 message to the controller command topic.
        :param velocity: The target joint velocity (rad/s).
        """
        msg = Float64MultiArray()
        # Assuming the starter joint is the only joint commanded, or the first one.
        msg.data = [float(velocity)] 
        self.publisher_.publish(msg)

    def joint_states_callback(self, msg: JointState):
        """
        Callback function for the /joint_states subscription.
        Finds the torque for the TARGET_JOINT_NAME and records the data.
        """
        if self.is_recording:
            try:
                # 1. Find the index of the TARGET_JOINT_NAME in the 'name' array
                joint_index = msg.name.index(TARGET_JOINT_NAME)
                
                # 2. Extract the effort (torque) from the 'effort' array using that index
                # Ensure the 'effort' array is long enough
                if joint_index < len(msg.effort):
                    torque = msg.effort[joint_index] 
                    current_time = time.time() - self.start_time
                    
                    self.torque_data.append(torque)
                    self.time_data.append(current_time)
                else:
                    self.get_logger().warn(f"Joint '{TARGET_JOINT_NAME}' found, but effort data is missing or out of sync.")

            except ValueError:
                # This happens if TARGET_JOINT_NAME is not in msg.name
                self.get_logger().debug(f"Joint '{TARGET_JOINT_NAME}' not yet in JointState message.")
            except Exception as e:
                self.get_logger().error(f"Error in joint_states_callback: {e}")

    def plot_data(self):
        """
        Generates and displays a plot of the recorded torque data vs. time.
        """
        if not self.time_data:
            self.get_logger().warn("No torque data was recorded to plot.")
            return

        plt.figure()
        plt.plot(self.time_data, self.torque_data)
        plt.title(f'PBA {TARGET_JOINT_NAME} Torque vs. Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Torque (Nm)')
        plt.grid(True)
        self.get_logger().info("Displaying torque plot. Close the plot window to finish the program.")
        plt.show() # This is a blocking call.

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PBARobotVelocityController()
        
        # Reset start time for relative timing
        node.start_time = time.time()

        # --- Movement Sequence ---
        
        # 1. Move forward at 1.0 rad/s for 5 seconds
        move_time_sec = 5
        velocity_forward = 5.0
        
        node.get_logger().info(f"Moving PBA starter joint forward at {velocity_forward} rad/s for {move_time_sec}s")
        start_time = time.time()
        
        # Start recording before movement
        node.is_recording = True 

        # Keep a high publishing rate for a smoother movement and data acquisition
        while rclpy.ok() and (time.time() - start_time) < move_time_sec:
            node.send_velocity_command(velocity_forward)
            # Spin once processes callbacks (like the torque subscription) and timers
            # Polling at ~100Hz for smoother data recording
            rclpy.spin_once(node, timeout_sec=0.01) 

        # 2. Stop
        node.get_logger().info("Final stop for PBA starter joint.")
        # Publish stop command for a short duration to ensure it is received
        for _ in range(5): 
            node.send_velocity_command(0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
        
        # Wait a moment to capture the final stop torque (e.g., gravity, friction)
        stop_wait_time = 1.0
        stop_start_time = time.time()
        while rclpy.ok() and (time.time() - stop_start_time) < stop_wait_time:
            rclpy.spin_once(node, timeout_sec=0.01)

        # Stop recording after movement and stop
        node.is_recording = False 

        node.get_logger().info("PBA robot movement demonstration complete.")
        
        # --- Plotting ---
        node.plot_data()

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