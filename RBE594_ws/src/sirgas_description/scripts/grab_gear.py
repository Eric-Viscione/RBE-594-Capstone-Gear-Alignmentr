#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from action_msgs.msg import GoalStatus
import time
from threading import Event
import math 
import numpy as np
from rclpy.executors import MultiThreadedExecutor 

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
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.04
        }
        
        self.get_logger().info("MoveIt Panda node initialized")

    def joint_state_callback(self, msg):
        """Store current joint state for planning"""
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
        
        filtered_state = JointState()
        filtered_state.header = msg.header
        filtered_state.name = panda_names
        filtered_state.position = panda_positions
        filtered_state.velocity = panda_velocities
        
        self.current_joint_state = filtered_state
        self.joint_state_event.set()

    def wait_for_joint_state(self, timeout=10.0):
        """Wait for joint state message"""
        self.get_logger().info("Waiting for joint state...")
        
        if self.current_joint_state is not None:
            self.get_logger().info("Using cached joint state")
            return True
            
        if not self.joint_state_event.wait(timeout):
            self.get_logger().error("Timeout waiting for joint state!")
            self.create_default_joint_state()
            return True
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

        goal_msg = MoveGroup.Goal()
        
        # Motion plan request
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        
        # --- ROBUST PLANNING PARAMETERS ---
        request.num_planning_attempts = 25  
        request.allowed_planning_time = 10.0 
        # ----------------------------------
        
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
        planning_options.plan_only = True
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 5
        
        goal_msg.request = request
        goal_msg.planning_options = planning_options
        
        self.get_logger().info("Sending planning request to MoveIt...")
        future = self.moveit_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected!")
            return None
            
        self.get_logger().info("MoveIt goal accepted, waiting for result...")
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
        """Create pose constraints for planning, now relative to the 'world' frame."""
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world" # Frame change
        pos_constraint.link_name = "panda_hand"
        
        # Create a small tolerance volume
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.02] 
        
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 0.7 
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint (loose)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world" # Frame change
        orient_constraint.link_name = "panda_hand"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 1e-1
        orient_constraint.absolute_y_axis_tolerance = 1e-1
        orient_constraint.absolute_z_axis_tolerance = 1e-1
        orient_constraint.weight = 1.0
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
            
        trajectory = self.plan_with_moveit(target_joints=target_joints)
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False

    def move_gripper(self, position):
        """Move gripper to specified position using action client, with a timeout."""
        self.get_logger().info(f"Moving gripper to position: {position}")
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False

        goal_msg = GripperCommand.Goal()
        command = GripperCommandMsg()
        command.position = position
        command.max_effort = 100.0
        
        goal_msg.command = command
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected!")
            return False
            
        self.get_logger().info("Gripper goal accepted, waiting for result...")
        
        # --- MODIFIED: Use the standard wait_for_result with a timeout ---
        timeout_sec = 10.0
        
        # Note: We must pass the future, not the GoalHandle, to the utility
        # that handles waiting for action results.
        result_future = goal_handle.get_result_async()
        
        # Spin until the result is complete or the timeout is reached
        # rclpy.spin_until_future_complete will block until completion or timeout
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        # Check the status after spinning
        if not result_future.done():
            # If the future is not done, it timed out.
            status = goal_handle.status
            if position == self.gripper_positions['close'] and status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().warn(f"Gripper close action timed out after {timeout_sec}s. Assuming successful grasp and continuing.")
                return True
            else:
                 self.get_logger().error(f"Gripper action timed out after {timeout_sec}s!")
                 # We can cancel the goal here if we want to clean up, but for a pick, continuing is often desired.
                 return False
        
        # If the future is done, we have a result.
        result = result_future.result()
        if result is None:
             self.get_logger().error("Gripper action failed to return a result!")
             return False

        # Get the final status of the goal
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Check the specific gripper result for a successful closure/grasp
            gripper_result = result.result
            if gripper_result.stalled:
                 self.get_logger().info("Gripper movement stalled (SUCCESS): Assumed gear grasped!")
                 return True
            elif gripper_result.reached_goal:
                self.get_logger().info("Gripper movement completed! (SUCCESS)")
                return True
            
        self.get_logger().error(f"Gripper movement failed with status: {status}")
        return False
        # --- END MODIFIED BLOCK ---

    def move_to_pose(self, target_pose: Pose):
        """
        Move the end-effector to a specified Pose (position and orientation) 
        using MoveIt planning.
        """
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_pose=target_pose)
        if trajectory:
            return self.execute_trajectory(trajectory)
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
        
        time.sleep(2.0)
        
        # 2. Operate gripper (Open)
        self.get_logger().info("Step 2: Opening gripper...")
        if self.move_gripper(self.gripper_positions['open']):
            self.get_logger().info("SUCCESS: Gripper opened!")
        else:
            self.get_logger().warn("Gripper movement may have failed")
        
        time.sleep(1.0)
        
        # 3. Move end-effector to (0.0, 0.0, 0.25) in WORLD frame and face down (Pick Position)
        self.get_logger().info("Step 3: Moving end-effector to (0.0, 0.0, 0.25) in WORLD frame and face down...")
        
        target_pose = Pose()
        target_pose.position.x = 0.0
        target_pose.position.y = 0.0
        target_pose.position.z = 0.25
        
        # Orientation: "face down" (180-degree rotation around Y-axis)
        target_pose.orientation.x = np.sqrt(2)/2
        target_pose.orientation.y = np.sqrt(2)/2
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0
        
        if self.move_to_pose(target_pose):
            self.get_logger().info("SUCCESS: Target pose reached!")
        else:
            self.get_logger().error("FAILED: Could not reach target pose!")
            return False

        time.sleep(2.0)
        
        # 4. Operate gripper (Close) - Ensures grasp with stall handling
        self.get_logger().info("Step 4: Closing gripper...")
        # Max effort is already set to 10.0 in move_gripper, ensuring maximum grip force.
        if self.move_gripper(self.gripper_positions['close']):
            self.get_logger().info("SUCCESS: Gripper closed (or gear grasped)!")
        else:
            self.get_logger().error("FAILED: Gripper failed to close!")
            return False
        
        time.sleep(1.0)
        
        # --- NEW STEP 5: LIFT STRAIGHT UP 0.3m ---
        
        # The new Z position is the old Z (0.25) + lift distance (0.30) = 0.55m
        lift_pose = Pose()
        lift_pose.position.x = 0.0
        lift_pose.position.y = 0.0
        lift_pose.position.z = 0.55
        lift_pose.orientation = target_pose.orientation # Maintain straight-down orientation
        
        self.get_logger().info("Step 5: Lifting gear straight up 0.3m to Z=0.55...")
        if self.move_to_pose(lift_pose):
            self.get_logger().info("SUCCESS: Lift complete!")
        else:
            self.get_logger().error("FAILED: Could not lift gear!")
            return False

        time.sleep(2.0)
        
        # 6. Move arm back to ready position (safe endpoint)
        self.get_logger().info("Step 6: Moving arm back to ready position...")
        if self.move_to_joints(self.poses['ready']):
            self.get_logger().info("SUCCESS: Ready position reached!")
        else:
            self.get_logger().error("FAILED: Could not reach ready position!")
            return False
        
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPanda()
    # Using MultiThreadedExecutor is good practice when dealing with multiple action clients/subscriptions
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Waiting for initialization...")
        time.sleep(5.0)
        
        # Execute the complete sequence
        node.execute_complete_sequence()
            
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
