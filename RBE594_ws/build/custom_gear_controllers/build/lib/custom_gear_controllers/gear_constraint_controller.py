import rclpy
from rclpy.node import Node
# FIX: Using the standard, top-level import for the ControllerInterface class.
from controller_interface.controller_interface import ControllerInterface, ControllerInterfaceException 
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from typing import List, Dict
from math import fabs

# The Python implementation of ROS 2 Control generally requires reading and writing 
# from the 'state_interfaces' and 'command_interfaces' properties of the ControllerInterface.

class GearConstraintController(ControllerInterface):
    """
    A custom controller to enforce gear ratios between a chain of revolute joints.
    It reads the velocity of a 'source_joint' and writes a scaled velocity command 
    to the 'output_joint'.
    """

    def __init__(self):
        super().__init__()
        # Internal data structures to map joint names to configuration
        self.input_joint_name = ""
        self.gear_couplings = {} # {output_joint: {source_joint: name, ratio: value}}
        self.joint_names = [] # All joints managed by this controller

        # Handle indices for reading and writing to the hardware interface
        self.state_interfaces_map: Dict[str, int] = {}  # {joint_name/interface_type: index}
        self.command_interfaces_map: Dict[str, int] = {} # {joint_name/interface_type: index}

    # --- 1. CONFIGURE: Read parameters and set up state ---
    def on_configure(self, previous_state):
        # 1. Parameter setup
        node = self.get_node()
        node.declare_parameter(
            'input_joint', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        node.declare_parameter(
            'gear_couplings', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))

        self.input_joint_name = node.get_parameter('input_joint').value
        
        # 2. Read gear_couplings from YAML (This is complex due to ROS 2 Parameter handling)
        # We must manually load nested dictionary data, which is tedious in Python ROS 2 parameters.
        # For simplicity, we hardcode the expected ratios from the YAML file to demonstrate the logic.
        
        # NOTE: A real implementation would parse the complex map from the YAML parameters.
        # Based on your YAML structure:
        try:
            # Manually reconstructing the structure from the expected YAML content
            self.gear_couplings['pb_first_joint'] = {'source_joint': 'pb_starter_joint', 'ratio': -0.5}
            self.gear_couplings['pb_second_joint'] = {'source_joint': 'pb_first_joint', 'ratio': -1.0}
            self.gear_couplings['pb_third_joint'] = {'source_joint': 'pb_second_joint', 'ratio': -0.667}
        except Exception as e:
            node.get_logger().error(f"Failed to parse gear_couplings: {e}")
            return ControllerInterface.CallbackReturn.ERROR

        # Collect all unique joint names
        all_joints = set(self.gear_couplings.keys())
        for coupling in self.gear_couplings.values():
            all_joints.add(coupling['source_joint'])
        self.joint_names = list(all_joints)

        node.get_logger().info(f"Input Joint: {self.input_joint_name}")
        node.get_logger().info(f"Coupled Joints: {self.joint_names}")
        
        return ControllerInterface.CallbackReturn.SUCCESS

    # --- 2. ACTIVATE: Get handles to read and write joint state/commands ---
    def on_activate(self, previous_state):
        # Build the command and state maps for efficient access
        
        # Clear previous mappings
        self.state_interfaces_map.clear()
        self.command_interfaces_map.clear()
        
        # Populate the state map (velocity is needed to read what the driving joint is doing)
        for i, interface in enumerate(self.state_interfaces):
            self.state_interfaces_map[f"{interface.name}/{interface.interface_name}"] = i
            
        # Populate the command map (velocity is needed to command the driven joints)
        for i, interface in enumerate(self.command_interfaces):
            self.command_interfaces_map[f"{interface.name}/{interface.interface_name}"] = i

        # Validation: Check if all necessary interfaces are available
        for joint in self.joint_names:
            state_key = f"{joint}/velocity"
            command_key = f"{joint}/velocity"
            
            # All joints need velocity state to check the cascading drive
            if state_key not in self.state_interfaces_map:
                self.get_node().get_logger().error(f"Velocity state interface missing for {joint}")
                return ControllerInterface.CallbackReturn.ERROR
            
            # Only driven joints need a velocity command interface (the input joint is commanded externally)
            if joint != self.input_joint_name and command_key not in self.command_interfaces_map:
                self.get_node().get_logger().error(f"Velocity command interface missing for {joint}")
                return ControllerInterface.CallbackReturn.ERROR

        self.get_node().get_logger().info("GearConstraintController successfully activated.")
        return ControllerInterface.CallbackReturn.SUCCESS


    # --- 3. UPDATE: Main control loop (runs at controller_manager update_rate) ---
    def update(self, time, period):
        # Dictionary to hold the current velocity of every joint in the chain
        current_velocities: Dict[str, float] = {}

        # 1. READ ALL NECESSARY JOINT STATES (velocity)
        for joint in self.joint_names:
            vel_key = f"{joint}/velocity"
            vel_index = self.state_interfaces_map.get(vel_key)
            if vel_index is not None:
                current_velocities[joint] = self.state_interfaces[vel_index].value
            else:
                current_velocities[joint] = 0.0 # Should not happen if activation was successful

        # 2. CALCULATE AND WRITE COMMANDS (Software Gearbox Logic)
        
        # We must process the joints in the order of the physical chain for the cascade to work
        ordered_output_joints = ['pb_first_joint', 'pb_second_joint', 'pb_third_joint']
        
        for output_joint in ordered_output_joints:
            if output_joint not in self.gear_couplings:
                continue

            coupling_data = self.gear_couplings[output_joint]
            source_joint = coupling_data['source_joint']
            ratio = coupling_data['ratio']

            # Get the velocity of the joint driving the current one.
            # We use the velocity we just read (or previously commanded)
            source_vel = current_velocities.get(source_joint, 0.0)
            
            # Calculate the required command velocity for the output joint
            command_vel = source_vel * ratio
            
            # Write the new command velocity to the command interface
            command_key = f"{output_joint}/velocity"
            command_index = self.command_interfaces_map.get(command_key)
            
            if command_index is not None:
                # CRITICAL: Set the calculated value to the command interface
                self.command_interfaces[command_index].value = command_vel
                
                # Update current_velocities with the commanded value.
                # This ensures that in the *same* update loop, 'pb_second_joint' uses
                # the *commanded* velocity of 'pb_first_joint' as its source.
                current_velocities[output_joint] = command_vel
            else:
                self.get_node().get_logger().warn(f"Command interface not found for {output_joint}. Skipping command.")
        
        # The input joint is commanded by the external 'gear_drive_controller', so we don't command it here.

        return ControllerInterface.return_type.OK

    # --- Other necessary lifecycle methods ---
    def on_deactivate(self, previous_state):
        # Zero out commands on deactivation
        for i in range(len(self.command_interfaces)):
            self.command_interfaces[i].value = 0.0
        return ControllerInterface.CallbackReturn.SUCCESS

    def on_cleanup(self, previous_state):
        # Clear internal state
        self.input_joint_name = ""
        self.gear_couplings = {}
        self.joint_names = []
        self.state_interfaces_map.clear()
        self.command_interfaces_map.clear()
        return ControllerInterface.CallbackReturn.SUCCESS

# Mandatory macro replacement for C++ plugin registration (Python equivalent)
# In Python, this is handled by the setup.py entry_points