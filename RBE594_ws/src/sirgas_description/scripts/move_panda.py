#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import time
from threading import Event

class MoveItPanda(Node):
    def __init__(self):
        super().__init__('moveit_panda')
        
        # Action client for MoveIt MoveGroup action
        self.moveit_action_client = ActionClient(
            self, 
            MoveGroup, 
            '/move_action'
        )
        
        # Action client for trajectory execution
        self.trajectory_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/panda_arm_controller/follow_joint_trajectory'
        )

         # Action client for gripper control
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/hand_controller/gripper_cmd'
        )
        
        # Subscribe to joint states for current robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.joint_state_event = Event()
        
        # Define poses from MoveIt assistant (from SRDF file)
        self.poses = {
            # Ready position from SRDF
            'ready': [1.5527, 0.0877, -0.08, -1.0748, -0.1121, 1.1697, 0.6243],
            # Home position (all zeros)
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6243],
            # Additional positions can be added here if defined in your MoveIt config
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.04
        }
        
        self.get_logger().info("MoveIt Panda node initialized")

    def joint_state_callback(self, msg):
        """Store current joint state for planning"""
        # Filter for Panda joints only
        panda_joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        panda_positions = []
        panda_velocities = []
        panda_names = []
        
        for i, name in enumerate(msg.name):
            if name in panda_joint_names:
                panda_names.append(name)
                panda_positions.append(msg.position[i])
                panda_velocities.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
        
        # Create filtered joint state
        filtered_state = JointState()
        filtered_state.header = msg.header
        filtered_state.name = panda_names
        filtered_state.position = panda_positions
        filtered_state.velocity = panda_velocities
        
        self.current_joint_state = filtered_state
        self.joint_state_event.set()
        # self.get_logger().info(f"Received joint state with {len(panda_names)} Panda joints")

    def wait_for_joint_state(self, timeout=10.0):
        """Wait for joint state message"""
        self.get_logger().info("Waiting for joint state...")
        
        # If we already have a joint state, use it
        if self.current_joint_state is not None:
            self.get_logger().info("Using cached joint state")
            return True
            
        # Otherwise wait for new message
        if not self.joint_state_event.wait(timeout):
            self.get_logger().error("Timeout waiting for joint state!")
            # Try to use a default joint state as fallback
            self.create_default_joint_state()
            return True  # Continue anyway with default state
        return True

    def create_default_joint_state(self):
        """Create a default joint state if none is available"""
        self.get_logger().info("Creating default joint state")
        self.current_joint_state = JointState()
        self.current_joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        self.current_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def plan_with_moveit(self, target_joints=None, target_pose=None):
        """Use MoveIt to plan a trajectory"""
        self.get_logger().info("Planning with MoveIt...")
        
        if not self.moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return None

        # Create MoveGroup goal
        goal_msg = MoveGroup.Goal()
        
        # Motion plan request
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # Set start state to current joint state
        if self.current_joint_state:
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            request.start_state = robot_state
            self.get_logger().info(f"Using start state with joints: {self.current_joint_state.position}")
        
        # Set goal - either joint target or pose target
        if target_joints:
            request.goal_constraints.append(self.create_joint_constraint(target_joints))
            self.get_logger().info(f"Planning to joint target: {target_joints}")
        elif target_pose:
            request.goal_constraints.append(self.create_pose_constraint(target_pose))
            self.get_logger().info(f"Planning to pose target: [{target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z}]")
        else:
            self.get_logger().error("No target specified!")
            return None
        
        # Planning options
        planning_options = PlanningOptions()
        planning_options.plan_only = True  # We just want the plan, not execution
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 5
        
        goal_msg.request = request
        goal_msg.planning_options = planning_options
        
        # Send goal
        self.get_logger().info("Sending planning request to MoveIt...")
        future = self.moveit_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected!")
            return None
            
        self.get_logger().info("MoveIt goal accepted, waiting for result...")
        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result and hasattr(result.result, 'error_code') and result.result.error_code.val == result.result.error_code.SUCCESS:
            self.get_logger().info("MoveIt planning successful!")
            return result.result.planned_trajectory.joint_trajectory
        else:
            error_code = result.result.error_code.val if result and hasattr(result.result, 'error_code') else "UNKNOWN"
            self.get_logger().error(f"MoveIt planning failed! Error code: {error_code}")
            return None

    def create_joint_constraint(self, target_joints):
        """Create joint constraints for planning"""
        from moveit_msgs.msg import Constraints, JointConstraint
        
        constraints = Constraints()
        joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", 
            "panda_joint7"
        ]
        
        for i, (name, position) in enumerate(zip(joint_names, target_joints)):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
            
        return constraints

    def create_pose_constraint(self, target_pose):
        """Create pose constraints for planning"""
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "panda_link0"
        pos_constraint.link_name = "panda_hand"
        
        # Create a small tolerance volume
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.02]  # 2cm tolerance
        
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint (loose)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "panda_link0"
        orient_constraint.link_name = "panda_hand"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 1.0
        orient_constraint.absolute_y_axis_tolerance = 1.0
        orient_constraint.absolute_z_axis_tolerance = 1.0
        orient_constraint.weight = 0.5
        constraints.orientation_constraints.append(orient_constraint)
        
        return constraints

    def execute_trajectory(self, joint_trajectory):
        """Execute trajectory using direct action client"""
        self.get_logger().info("Executing trajectory...")
        
        if not self.trajectory_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory action server not available!")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory
        
        future = self.trajectory_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected!")
            return False
            
        self.get_logger().info("Trajectory execution in progress...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result:
            self.get_logger().info("Trajectory execution completed!")
            return True
        else:
            self.get_logger().error("Trajectory execution failed!")
            return False

    def move_to_joints(self, target_joints):
        """Move to joint positions using MoveIt planning"""
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        # Plan with MoveIt
        trajectory = self.plan_with_moveit(target_joints=target_joints)
        if trajectory:
            # Execute the planned trajectory
            return self.execute_trajectory(trajectory)
        return False

    def move_gripper(self, position):
        """Move gripper to specified position using action client"""
        self.get_logger().info(f"Moving gripper to position: {position}")
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False

        # Create gripper command goal
        from control_msgs.msg import GripperCommand as GripperCommandMsg
        
        goal_msg = GripperCommand.Goal()
        command = GripperCommandMsg()
        command.position = position
        command.max_effort = 10.0  # Adjust as needed
        
        goal_msg.command = command
        
        # Send goal
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected!")
            return False
            
        self.get_logger().info("Gripper goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result:
            self.get_logger().info("Gripper movement completed!")
            return True
        else:
            self.get_logger().error("Gripper movement failed!")
            return False

    def execute_complete_sequence(self):
        """Execute the complete motion sequence"""
        self.get_logger().info("Starting complete motion sequence...")
        
        # 1. Move arm to ready position
        self.get_logger().info("Step 1: Moving arm to ready position...")
        if self.move_to_joints(self.poses['ready']):
            self.get_logger().info("SUCCESS: Ready position reached!")
        else:
            self.get_logger().error("FAILED: Could not reach ready position!")
            return False
        
        time.sleep(2.0)  # Pause at ready position
        
        # 2. Move gripper from close to close to open
        self.get_logger().info("Step 2: Operating gripper...")
        
        # Open
        self.get_logger().info("Opening gripper...")
        if self.move_gripper(self.gripper_positions['open']):
            self.get_logger().info("SUCCESS: Gripper opened!")
        else:
            self.get_logger().warn("Gripper movement may have failed")
        
        time.sleep(1.0)
        
        # Close
        self.get_logger().info("Closing gripper...")
        if self.move_gripper(self.gripper_positions['close']):
            self.get_logger().info("SUCCESS: Gripper closed!")
        else:
            self.get_logger().warn("Gripper movement may have failed")
        
        time.sleep(1.0)
        
        # 3. Move arm back to home position (all zeros)
        self.get_logger().info("Step 3: Moving arm back to home position...")
        if self.move_to_joints(self.poses['home']):
            self.get_logger().info("SUCCESS: Home position reached!")
        else:
            self.get_logger().error("FAILED: Could not reach home position!")
            return False
        
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

def main():
    rclpy.init()
    
    node = MoveItPanda()
    
    try:
        # Wait for everything to initialize
        node.get_logger().info("Waiting for initialization...")
        time.sleep(5.0)  # Increased wait time
        
        # Execute the complete sequence
        node.execute_complete_sequence()
            
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()