#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState, CollisionObject, PlanningScene 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive # Import needed for the Cylinder
from shape_msgs.msg import Mesh
from move_pba import PBARobotVelocityController

import time
import subprocess
from threading import Event
import numpy as np
from rclpy.executors import MultiThreadedExecutor 
from moveit_msgs.msg import AttachedCollisionObject 

# NEW IMPORTS FOR CARTESIAN PATH CONSTRAINTS (ROS 2 Method)
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint


import math
from scipy.spatial.transform import Rotation as R
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

# --- CONSTANTS FOR GRASPING ---
GEAR_HEIGHT = 0.1   # Height 10 cm
GEAR_SIZE = 0.06  # Diameter 7 cm (User corrected value)
GEAR_BASE_Z = 0.0  # Base Z position
GEAR_CENTER_Z = GEAR_BASE_Z + (GEAR_HEIGHT / 2) 
# -----------------------------

class MoveItPanda(Node):
    def __init__(self):
        super().__init__('moveit_panda')
        
        # Action clients and publishers
        self.tag_processing_process = None
        self.moveit_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        # --- NEW: Forward Kinematics Service Client ---
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting again...')
        self.get_logger().info('FK service client created.')
        
        self.current_joint_state = None
        self.joint_state_event = Event()
        self.angle_correction_rad = None
        self.axis_diff_sub = self.create_subscription(
            PoseStamped, 
            '/tag_axis_difference', 
            self.axis_diff_callback, 
            1
        )
        # Define poses
        self.poses = {
            'ready': [1.5527, 0.0877, -0.08, -1.0748, -0.1121, 1.1697, 0.6243],
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6243],
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.06,
            'grasp': 0.02
        }
        
        # NOTE: End-effector link is needed for constraint definition. Assuming "panda_hand"
        self.end_effector_link = "panda_hand" 
        
        self.get_logger().info("MoveIt Panda node initialized")

    def joint_state_callback(self, msg):
        """Store current joint state for planning"""
        panda_joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        panda_positions = []
        panda_names = []
        
        for i, name in enumerate(msg.name):
            if name in panda_joint_names:
                panda_names.append(name)
                panda_positions.append(msg.position[i])
        
        filtered_state = JointState()
        filtered_state.header = msg.header
        filtered_state.name = panda_names
        filtered_state.position = panda_positions
        
        self.current_joint_state = filtered_state
        self.joint_state_event.set()

    def wait_for_joint_state(self, timeout=1.0):
        """Wait for joint state message"""
        self.get_logger().info("Waiting for joint state...")
        if self.current_joint_state is not None:
            self.get_logger().info("Using cached joint state")
            return True
        if not self.joint_state_event.wait(timeout):
            self.get_logger().error("Timeout waiting for joint state! Creating default state.")
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

    def clear_gear_references(self):
        """
        Forcefully removes the 'first_gear' from both the attached list 
        and the world list to ensure a clean state for the planning scene.
        """
        self.get_logger().warn("Executing FORCEFUL SCENE CLEANUP for 'first_gear'...")
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        
        # 1. Remove from the world list (CollisionObject.REMOVE)
        co_remove_world = CollisionObject()
        co_remove_world.header.frame_id = "world"
        co_remove_world.id = "first_gear"
        co_remove_world.operation = CollisionObject.REMOVE 
        ps_msg.world.collision_objects.append(co_remove_world)
        
        # 2. Remove from the attached list (AttachedCollisionObject REMOVE)
        aco_detach = AttachedCollisionObject()
        aco_detach.link_name = "panda_hand"
        aco_detach.object.id = "first_gear"
        aco_detach.object.operation = CollisionObject.REMOVE 
        ps_msg.robot_state.attached_collision_objects.append(aco_detach)

        # Must mark the robot_state section as a diff when modifying attached objects
        ps_msg.robot_state.is_diff = True 

        # Publish the combined cleanup message repeatedly for robustness
        for i in range(10):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        time.sleep(2.0) 
        self.get_logger().warn("Forceful scene cleanup complete. Scene should be clear for planning.")

    # def add_gear_to_scene(self):
    #     """Adds a collision object representing the gear using a SolidPrimitive (Cylinder)."""
    #     self.get_logger().info(f"Adding 'first_gear' (Rectangular Prism Length & Width={GEAR_SIZE}m, height={GEAR_HEIGHT}) to the planning scene...")
        
    #     gear_co = CollisionObject()
    #     gear_co.header.frame_id = "world" 
    #     gear_co.id = "first_gear"
        
    #     # box = SolidPrimitive()
    #     # box.type = SolidPrimitive.BOX
    #     # box.dimensions = [GEAR_SIZE, GEAR_SIZE, GEAR_HEIGHT] 

    #     # 1. Create a Mesh object
    #     gear_mesh = Mesh()

    #     # 2. Define the path to your STL file
    #     # NOTE: This path MUST be accessible by the MoveIt process.
    #     # You might need to use a package path resolver, similar to how it's done in the URDF:
    #     gear_mesh.filename = "package://sirgas_description/meshes/First_Gear.stl" 

    #     # 3. Define a scale factor (usually 1.0)
    #     gear_mesh.scale = [1.0, 1.0, 1.0]
    #     gear_pose = Pose()
    #     gear_pose.position.x = 0.0
    #     gear_pose.position.y = -1.0
    #     gear_pose.position.z = GEAR_CENTER_Z 
    #     gear_pose.orientation.w = 1.0 
    #     # 4. Assign the mesh to the Collision Object
    #     co.meshes.append(gear_mesh)
    #     co.mesh_poses.append(gear_pose) # Use the same pose as before
  
        
    #     # gear_co.primitives.append(box) 
    #     gear_co.primitive_poses.append(gear_pose) 
    #     gear_co.operation = CollisionObject.ADD 
        
    #     ps_msg = PlanningScene()
    #     ps_msg.world.collision_objects.append(gear_co)
    #     ps_msg.is_diff = True 
        
    #     self.get_logger().info("Publishing 'first_gear' (BOX) to planning scene...")
    #     for _ in range(5):
    #         self.planning_scene_pub.publish(ps_msg)
    #         time.sleep(0.1) 
            
    #     self.get_logger().info("'first_gear' (BOX) should now be in the planning scene.")

    def add_gear_to_scene(self):
        """Adds a collision object representing the gear using the accurate Mesh (.stl) geometry."""
        self.get_logger().info(f"Adding 'first_gear' (Mesh: First_Gear.stl) to the planning scene...")
        
        gear_co = CollisionObject()
        gear_co.header.frame_id = "world" 
        gear_co.id = "first_gear"
        
        # ... (header setup)
        
        # 1. Define the geometry as a SolidPrimitive (Cylinder)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        
        # Adjust dimensions: Use a cylinder that represents the outer, graspable part.
        # If the outer diameter is 0.06m, use slightly less for the cylinder radius.
        # R = 0.03m (GEAR_SIZE / 2.0)
        # Dimensions are [height, radius]
        # Set radius to a size that prevents the planner from passing through the graspable area.
        box.dimensions = [GEAR_SIZE, GEAR_SIZE, GEAR_HEIGHT] # Radius slightly larger than 0.03m

        # 2. Define the Pose
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = -1.0
        gear_pose.position.z = GEAR_CENTER_Z 
        gear_pose.orientation.w = 1.0 
        
        # 3. Assign the primitive and its pose
        gear_co.primitives.append(box) 
        gear_co.primitive_poses.append(gear_pose) 
        
        # 4. Set the operation
        gear_co.operation = CollisionObject.ADD

        # 5. Set the operation
        gear_co.operation = CollisionObject.ADD 
        
        # 6. Publish the Planning Scene update
        ps_msg = PlanningScene()
        ps_msg.world.collision_objects.append(gear_co)
        ps_msg.is_diff = True 
        
        self.get_logger().info("Publishing 'first_gear' (MESH) to planning scene...")
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1) 
            
        self.get_logger().info("'first_gear' (MESH) should now be in the planning scene.")

    def launch_tag_processing(self):
            """Launches the tag_processing.launch.py via subprocess."""
            self.get_logger().warn("Starting tag_processing.launch.py via subprocess ")
            
            # The command to execute
            command = [
                'ros2', 'launch', 
                'sirgas_apriltag_detector', 
                'tag_processing.launch.py'
            ]
            
            try:
                self.tag_processing_process = subprocess.Popen(command)
                self.get_logger().info(f"Tag processing launched with PID: {self.tag_processing_process.pid}")
                return True
            except FileNotFoundError:
                self.get_logger().error("ROS 2 command not found. Ensure your environment is sourced.")
                return False
            except Exception as e:
                self.get_logger().error(f"Error launching tag processing: {e}")
                return False
            
    def add_gear_to_scene2(self):
        """Adds a collision object representing the gear using a SolidPrimitive (Cylinder)."""
        self.get_logger().info(f"Adding 'first_gear' (Rectangular Prism Length & Width={GEAR_SIZE}m, height={GEAR_HEIGHT}) to the planning scene...")
        
        gear_co = CollisionObject()
        gear_co.header.frame_id = "world" 
        gear_co.id = "first_gear"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [GEAR_SIZE, GEAR_SIZE, GEAR_HEIGHT] 
        
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = 0.0
        gear_pose.position.z = 0.185
        gear_pose.orientation.w = 1.0 
        
        gear_co.primitives.append(box) 
        gear_co.primitive_poses.append(gear_pose) 
        gear_co.operation = CollisionObject.ADD 
        
        ps_msg = PlanningScene()
        ps_msg.world.collision_objects.append(gear_co)
        ps_msg.is_diff = True 
        
        self.get_logger().info("Publishing 'first_gear' (BOX) to planning scene...")
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1) 
            
        self.get_logger().info("'first_gear' (BOX) should now be in the planning scene.")

    def attach_gear_to_hand(self):
        """Attaches the gear to the robot hand, explicitly providing geometry for robustness."""
        self.get_logger().info("Attaching 'first_gear' to 'panda_hand'...")
        
        # Re-create geometry and pose 
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [GEAR_SIZE, GEAR_SIZE, GEAR_HEIGHT] 
        
        
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = -1.0
        gear_pose.position.z = GEAR_CENTER_Z
        gear_pose.orientation.w = 1.0 

        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand" 
        
        aco.object.header.frame_id = "world"
        aco.object.id = "first_gear"
        aco.object.operation = CollisionObject.ADD 
        
        # Explicitly include geometry when attaching
        aco.object.primitives.append(box) 
        aco.object.primitive_poses.append(gear_pose) 

        # Define the links the attached object is allowed to touch (CRITICAL FIX)
        aco.touch_links = ['panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        
        ps_msg = PlanningScene()
        ps_msg.robot_state.attached_collision_objects.append(aco) 
        ps_msg.robot_state.is_diff = True
        ps_msg.is_diff = True
        
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        self.get_logger().info("'first_gear' is now attached to the hand.")
    
    def attach_gear_to_hand2(self):
        """Attaches the gear to the robot hand, explicitly providing geometry for robustness."""
        self.get_logger().info("Attaching 'first_gear' to 'panda_hand'...")
        
        # Re-create geometry and pose 
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [GEAR_SIZE, GEAR_SIZE, GEAR_HEIGHT] 
        
        
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = 0.0
        gear_pose.position.z = 0.185
        gear_pose.orientation.w = 1.0 

        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand" 
        
        aco.object.header.frame_id = "world"
        aco.object.id = "first_gear"
        aco.object.operation = CollisionObject.ADD 
        
        # Explicitly include geometry when attaching
        aco.object.primitives.append(box) 
        aco.object.primitive_poses.append(gear_pose) 

        # Define the links the attached object is allowed to touch (CRITICAL FIX)
        aco.touch_links = ['panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        
        ps_msg = PlanningScene()
        ps_msg.robot_state.attached_collision_objects.append(aco) 
        ps_msg.robot_state.is_diff = True
        ps_msg.is_diff = True
        
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        self.get_logger().info("'first_gear' is now attached to the hand.")

    def remove_gear_from_world_after_attach(self):
        """Explicitly removes the gear from the world collision list after attachment to prevent CheckStartStateCollision errors."""
        self.get_logger().info("Explicitly removing 'first_gear' from world collision objects (leaving attached copy)...")
        
        co_remove_world = CollisionObject()
        co_remove_world.header.frame_id = "world"
        co_remove_world.id = "first_gear"
        co_remove_world.operation = CollisionObject.REMOVE 
        
        ps_msg = PlanningScene()
        ps_msg.world.collision_objects.append(co_remove_world)
        ps_msg.is_diff = True
        
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
        
        self.get_logger().info("'first_gear' explicitly removed from world collision objects.")
    
    # --- MODIFIED FUNCTION: allow_start_state_collision ARGUMENT REMOVED ---
    def plan_with_moveit(self, target_joints=None, target_pose=None, path_constraints=None): 
        """Use MoveIt to plan a trajectory"""
        self.get_logger().info("Planning with MoveIt...")
        
        if not self.moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 15000
        request.allowed_planning_time = 15.0
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0
        
        if self.current_joint_state:
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            request.start_state = robot_state
        
        if target_joints:
            request.goal_constraints.append(self.create_joint_constraint(target_joints))
        elif target_pose:
            request.goal_constraints.append(self.create_pose_constraint(target_pose))
        else:
            self.get_logger().error("No target specified!")
            return None
        if path_constraints:
            request.path_constraints = path_constraints
        planning_options = PlanningOptions()
        planning_options.plan_only = True
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 75000
        
        goal_msg.request = request
        goal_msg.planning_options = planning_options
        
        future = self.moveit_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected!")
            return None
            
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

        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "panda_hand"
        
        # Create a small tolerance volume
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.0025] 
        
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0 
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint (tight)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "panda_hand"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 3e-5
        orient_constraint.absolute_y_axis_tolerance = 3e-5
        orient_constraint.absolute_z_axis_tolerance = 3e-5
        orient_constraint.weight = 0.95
        constraints.orientation_constraints.append(orient_constraint)
        
        return constraints

    def execute_trajectory(self, joint_trajectory):
        """Execute trajectory using direct action client"""
        self.get_logger().info("Executing trajectory...")
        
        if not self.trajectory_action_client.wait_for_server(timeout_sec=10.0):
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

    # --- MODIFIED FUNCTION: allow_start_state_collision ARGUMENT REMOVED ---
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
        command.max_effort = 1750.0
        
        goal_msg.command = command
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected!")
            return False
            
        self.get_logger().info("Gripper goal accepted, waiting for result...")
        
        timeout_sec = 10.0
        
        result_future = goal_handle.get_result_async()
        
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        # Check the status after spinning
        if not result_future.done():
            status = goal_handle.status
            # Special case for grasping: if executing (stalled) or accepted, assume success
            if position <= self.gripper_positions['open'] and status in [GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED]:
                self.get_logger().warn(f"Gripper action timed out after {timeout_sec}s while closing/grasping. Assuming successful grasp and continuing.")
                return True
            else:
                 self.get_logger().error(f"Gripper action timed out after {timeout_sec}s!")
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

    # --- MODIFIED FUNCTION: allow_start_state_collision ARGUMENT REMOVED ---
    def move_to_pose(self, target_pose: Pose, path_constraints=None): 
        """Move the end-effector to a specified Pose (position and orientation) using MoveIt planning."""
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_pose=target_pose, path_constraints=path_constraints)
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False
        


    def move_cartesian_straight_line(self, final_pose: Pose):
        """
        Use MoveIt's compute_cartesian_path service for straight-line Cartesian motion.
        This is the most direct equivalent to the RViz checkbox.
        """

        
        self.get_logger().info("Computing Cartesian path...")
        
        # Create service client
        cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        if not cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartesian path service not available!")
            return False
        
        # Prepare request
        request = GetCartesianPath.Request()
        
        # Set start state
        if self.current_joint_state:
            request.start_state.joint_state = self.current_joint_state
        
        request.group_name = "panda_arm"
        request.link_name = self.end_effector_link
        
        # Create waypoints - for straight line, we just need the final pose
        waypoint_pose = PoseStamped()
        waypoint_pose.header.frame_id = "world"
        waypoint_pose.pose = final_pose
        request.waypoints = [waypoint_pose.pose]
        
        request.max_step = 0.01  # Resolution of Cartesian path
        request.jump_threshold = 0.0  # Disable jump prevention for straight line
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = False  # This is the key - don't avoid collisions
        
        # Send request
        future = cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.get_logger().info("Cartesian path computation successful!")
                return self.execute_trajectory(response.solution.joint_trajectory)
            else:
                self.get_logger().error(f"Cartesian path computation failed! Error code: {response.error_code.val}")
        else:
            self.get_logger().error("Service call failed!")
        
        return False
    
    def get_current_pose(self) -> Pose:
        """
        Retrieves the current end-effector pose using the MoveIt Forward Kinematics service.
        """
        if self.current_joint_state is None:
            self.get_logger().error("Cannot compute FK: Current joint state is not available.")
            return None

        # 1. Build the FK request
        fk_request = GetPositionFK.Request()
        fk_request.fk_link_names = [self.end_effector_link]  # self.end_effector_link should be 'panda_hand'
        
        # 2. Populate the RobotState message with current joint data
        fk_request.robot_state.joint_state = self.current_joint_state
        
        # 3. Call the FK service
        future = self.fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            
            # 4. Check for success and return the pose
            if response.error_code.val == response.error_code.SUCCESS:
                # The response contains a list of poses, we only requested one link
                return response.pose_stamped[0].pose 
            else:
                self.get_logger().error(f"FK service failed with error code: {response.error_code.val}")
                return None
        else:
            self.get_logger().error("FK service call failed (No response).")
            return None
    
    def multiply_quaternions(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        """
        Multiplies two ROS Quaternion messages (q1 * q2) using scipy's Rotation.
        This performs the composition of rotations (q1 followed by q2).
        """
        # Convert ROS Quaternions to scipy Rotation objects (xyzw format)
        r1 = R.from_quat([q1.x, q1.y, q1.z, q1.w])
        r2 = R.from_quat([q2.x, q2.y, q2.z, q2.w])
        
        # Perform multiplication (composition)
        r_new = r1 * r2
        
        # Convert back to ROS Quaternion message
        q_out_array = r_new.as_quat()
        
        q_out = Quaternion()
        q_out.x = q_out_array[0]
        q_out.y = q_out_array[1]
        q_out.z = q_out_array[2]
        q_out.w = q_out_array[3]
        return q_out
    def create_partial_orientation_constraint(self, link_name, target_pose, free_axis='z', tolerance_rpy=None):
        """Creates an OrientationConstraint message that locks specific axes."""
        oc = OrientationConstraint()
        oc.header.frame_id = target_pose.header.frame_id if hasattr(target_pose, 'header') else 'panda_link0'
        oc.link_name = link_name
        oc.orientation = target_pose.orientation # Constrain to the current orientation
        
        # Set tolerances (roll, pitch, yaw)
        if tolerance_rpy is None:
            tolerance_rpy = [0.01, 0.01, 0.01] # Default tight tolerance
        
        oc.absolute_x_axis_tolerance = tolerance_rpy[0]
        oc.absolute_y_axis_tolerance = tolerance_rpy[1]
        oc.absolute_z_axis_tolerance = tolerance_rpy[2]

        # Explicitly loosen the tolerance for the axis we want to rotate around
        if free_axis == 'z':
            oc.absolute_z_axis_tolerance = math.pi # Allow full rotation
        elif free_axis == 'x':
            oc.absolute_x_axis_tolerance = math.pi
        elif free_axis == 'y':
            oc.absolute_y_axis_tolerance = math.pi
            
        oc.weight = 1.0 # Set high weight for a strict constraint
        return oc
    def create_position_constraint(self, link_name, target_pose, tolerance_xyz):
        """Creates a PositionConstraint message."""
        pc = PositionConstraint()
        pc.header.frame_id = target_pose.header.frame_id if hasattr(target_pose, 'header') else 'panda_link0'
        pc.link_name = link_name
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        # Define the bounding region for the constraint (a box around the current point)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2 * tolerance_xyz, 2 * tolerance_xyz, 2 * tolerance_xyz]
        
        # Set the constraint frame to the target position
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target_pose.pose if hasattr(target_pose, 'pose') else target_pose)
        
        pc.weight = 1.0 # Set high weight for a strict constraint
        return pc
    
    def get_current_joint_positions(self):
        """Returns the current joint positions as a list, ordered by name."""
        if self.current_joint_state is None:
            self.get_logger().error("Cannot retrieve joint positions: State not available.")
            return None

        # Order of joints must match the controller's expectation (panda_joint1 to panda_joint7)
        ordered_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # Create a dictionary for quick lookup by name
        name_to_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

        # Return positions in the canonical order
        return [name_to_pos.get(name, 0.0) for name in ordered_names]
    
    def rotate_joint7_directly(self, angle_radians: float) -> bool:
        """
        Directly commands panda_joint7 to rotate by a relative angle 
        using the FollowJointTrajectory action client, bypassing MoveIt planning.
        """
        self.get_logger().info(f"Attempting direct rotation of panda_joint7 by {np.degrees(angle_radians):.2f} degrees...")

        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False

        # Calculate the new target position for joint 7
        target_positions = list(current_positions)
        # Joint 7 is at index 6 in the list (0-indexed)
        target_positions[6] += angle_radians 

        # --- Create JointTrajectory Message ---
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # 1. Start point (current position) - Optional, but good practice
        point_start = JointTrajectoryPoint()
        point_start.positions = current_positions
        point_start.time_from_start.sec = 0  # Start immediately

        # 2. End point (target position)
        point_end = JointTrajectoryPoint()
        point_end.positions = target_positions
        # Set a duration for the movement (e.g., 2 seconds)
        point_end.time_from_start.sec = 2 

        trajectory.points.append(point_start)
        trajectory.points.append(point_end)

        # Execute the trajectory using the existing executor
        if self.execute_trajectory(trajectory):
            self.get_logger().info("SUCCESS: Direct rotation of panda_joint7 complete.")
            return True
        else:
            self.get_logger().error("FAILED: Direct joint trajectory execution failed.")
            return False
            
    def axis_diff_callback(self, msg: PoseStamped):

        self.angle_correction_rad = msg.pose.position.x
        self.get_logger().info(
            f"Received Z-axis correction angle: {np.degrees(self.angle_correction_rad):.2f} degrees"
        )
    def cleanup_subprocesses(self):
        """Terminates the tag processing subprocess if it is running."""
        if self.tag_processing_process:
            self.get_logger().warn("Executing graceful subprocess termination...")
            
            # 1. Terminate (sends SIGTERM)
            self.tag_processing_process.terminate()
            
            # 2. Wait for it to terminate, with a timeout
            try:
                self.tag_processing_process.wait(timeout=2)
                self.get_logger().info("Subprocess terminated gracefully.")
            except subprocess.TimeoutExpired:
                # 3. If it times out, force-kill (sends SIGKILL)
                self.get_logger().error("Subprocess termination timed out. Forcing kill.")
                self.tag_processing_process.kill() 
                self.tag_processing_process.wait()
            
            self.tag_processing_process = None

    def execute_complete_sequence(self):
        """
        Execute the complete motion sequence.
        """
        
        self.get_logger().info("Starting complete motion sequence...")
        test_rotate = False
        test_pba = False
        PICK_Z = 0.0775
        PRE_PICK_Z = 0.2
        base_correction_angle = 0 #typically 2.35
        side_orientation = Quaternion(x=np.sqrt(2)/2, y=0.0, z=np.sqrt(2)/2, w=0.0)
        face_down_orientation = Quaternion(x=np.sqrt(2)/2, y=np.sqrt(2)/2, z=0.0, w=0.0)
        pre_pick_pose = Pose(position=Point(x=-0.105, y=-1.0, z=PRE_PICK_Z), orientation=side_orientation)
        target_pose = Pose(position=Point(x=-0.103, y=-1.0, z=PICK_Z), orientation=side_orientation)
        pre_drop_pose = Pose(position=Point(x=-0.103, y=0.0, z=0.45), orientation=target_pose.orientation)
        place_pose = Pose(position=Point(x=-0.105, y=0.0, z=0.315), orientation=target_pose.orientation)
        pre_pick_pose2 = Pose(position=Point(x=0.0, y=0.0, z=0.4), orientation=face_down_orientation)
        pick_pose2 = Pose(position=Point(x=0.0, y=0.0, z=0.3), orientation=face_down_orientation)
        pre_rotate_pose =  Pose(position=Point(x=0.0, y=0.0, z=0.35), orientation=face_down_orientation)
        post_rotate_pose = Pose(position=Point(x=0.0, y=0.0, z = 0.31), orientation=face_down_orientation)
        push_pose = Pose(position=Point(x=0.0, y=0.0, z = 0.285), orientation=face_down_orientation)
        LIFT_DISTANCE = 0.4
        LIFT_Z = PICK_Z + LIFT_DISTANCE 
        # Using -0.1, -1.0 for X/Y position from 4B/5/6
        lift_pose = Pose(position=Point(x=-0.1, y=-1.0, z=LIFT_Z), orientation=target_pose.orientation) 


        # --- STEP 0: FORCEFUL CLEANUP ---
        self.clear_gear_references()
        if not test_pba:
            if test_rotate:
                self.get_logger().info("Test Mode Active: Gear on PBA at start")
            if not test_rotate:
                # 1. Move arm to ready position (Fixes StartStateCollision before adding object)
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
                
                # 3. ADD GEAR CYLINDER TO SCENE 
                self.get_logger().info("Step 3: Adding gear to the planning scene now that robot is in a clear position...")
                self.add_gear_to_scene()
                time.sleep(1.0)

                # 4A. Move to Pre-Pick Waypoint (High Z)
            
                self.get_logger().info(f"Step 4A: Moving to PRE-PICK pose (Z={PRE_PICK_Z}m)...")
                if not self.move_to_pose(pre_pick_pose):
                    self.get_logger().error("FAILED: Could not reach PRE-PICK pose!")
                    return False
                time.sleep(5.0)

                # 4B. Move down to Final Pick Position (Low Z)
                
                self.get_logger().info(f"Step 4B: Moving to FINAL PICK pose (Z={PICK_Z}m)...")
                if self.move_to_pose(target_pose):
                    self.get_logger().info("SUCCESS: Final pick pose reached!")
                else:
                    self.get_logger().error("FAILED: Could not reach FINAL PICK pose!")
                    return False
                time.sleep(5.0)
                
                # 5. Operate gripper (Close), ATTACH GEAR, and REMOVE WORLD COPY
                self.get_logger().info(f"Step 5: Closing gripper to GRASP position ({self.gripper_positions['grasp']}m)...")
                if self.move_gripper(self.gripper_positions['grasp']):
                    self.get_logger().info("SUCCESS: Gripper closed (or gear grasped)! Attaching gear to hand.")
                    
                    # 5A: Attach gear to the hand
                    self.attach_gear_to_hand()
                    time.sleep(3.0)

                    # 5B: Explicitly remove the original world copy to avoid CheckStartStateCollision
                    self.remove_gear_from_world_after_attach()
                    time.sleep(3.0)
                else:
                    self.get_logger().error("FAILED: Gripper failed to close!")
                    return False
                
                time.sleep(3.0)
                
                # 6. LIFT STRAIGHT UP 0.4m

                
                self.get_logger().info(f"Step 6: Lifting gear straight up {LIFT_DISTANCE}m to Z={LIFT_Z}...")
                if self.move_to_pose(lift_pose):
                    self.get_logger().info("SUCCESS: Lift complete!")
                else:
                    self.get_logger().error("FAILED: Could not lift gear!")
                    return False

                time.sleep(2.0)
                
                # 7. Move arm back to ready position
                self.get_logger().info("Step 7: Moving arm back to ready position...")
                move_success = self.move_to_joints(self.poses['ready'])
                
                if move_success:
                    self.get_logger().info("SUCCESS: Ready position reached!")
                else:
                    self.get_logger().error("FAILED: Could not reach ready position!")
                    return False # Fail if this move fails

                time.sleep(2.0)
                
                # Define the final drop pose (Place Pose) and the approach pose

                
                # 8A. Move to Pre-Drop Location (PTP Move)
                self.get_logger().info("Step 8A: Moving Gear to Pre-Drop Location (PTP) at Z=0.45m...")
                if self.move_to_pose(pre_drop_pose):
                    self.get_logger().info("SUCCESS: Pre-Drop Location reached!")
                else:
                    self.get_logger().error("FAILED: Could not reach Pre-Drop Location!")
                    return False
                    
                time.sleep(5.0)

                # 8B. Drop Gear via Cartesian Path (NEW STEP)
                self.get_logger().info("Step 8B: Dropping Gear via Cartesian Path (Linear Down) to Z=0.325m...")
                # This uses path constraints (ROS 2 method) to ensure a straight vertical drop 
                # while maintaining the X and Y coordinates.
                if self.move_cartesian_straight_line(place_pose):
                    self.get_logger().info("SUCCESS: Gear is placed on Peg Board!")
                else:
                    self.get_logger().error("FAILED: Could not execute Cartesian Drop!")
                    return False
                
            time.sleep(3.0)
            if test_rotate:
                self.get_logger().info("Step 1: Moving arm to ready position...")
                if self.move_to_joints(self.poses['ready']):
                    self.get_logger().info("SUCCESS: Ready position reached!")
                else:
                    self.get_logger().error("FAILED: Could not reach ready position!")
                    return False
                
                time.sleep(2.0)
            # 9. Operate gripper (Open)
            self.get_logger().info("Step 9: Opening gripper...")
            if self.move_gripper(self.gripper_positions['open']):
                self.get_logger().info("SUCCESS: Gripper opened!")
            else:
                self.get_logger().warn("Gripper movement may have failed")
            
            time.sleep(1.0)

            self.get_logger().info("--- SCENE CLEANUP: Clearing all gear references ---\n")
            self.clear_gear_references() 

            # 10. Move arm back to Home position
            self.get_logger().info("Step 10: Moving arm back to Home position...")
            move_success = self.move_to_joints(self.poses['home'])
            
            if move_success:
                self.get_logger().info("SUCCESS: Home position reached!")
            else:
                self.get_logger().error("FAILED: Could not reach Home position!")
                return False # Fail if this move fails
            
            ##Start tag identification
            self.launch_tag_processing()

            time.sleep(2.0)
            
            timeout = 15.0
            # 11. Take axis measurment, with arm out of way
            self.get_logger().info(f"Step 11: Waiting for FIRST axis difference measurement (max {timeout})...")
            start_time = time.time()
            while (self.angle_correction_rad is None) and (time.time() - start_time < timeout):
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.angle_correction_rad is None:
                self.get_logger().warn("Axis measurement TIMEOUT. Proceeding with NO rotation (correction=0.0).")
                correction_angle = 0.0
            else:
                correction_angle = self.angle_correction_rad
                self.get_logger().info(f"Step 1 SUCCESS: Received correction angle of {np.degrees(correction_angle):.2f} degrees.")
            if self.tag_processing_process:
                self.get_logger().info("Stopping tag_processing.launch.py subprocess...")
                self.tag_processing_process.terminate()
                # Wait briefly for the process to terminate gracefully
                try:
                    self.tag_processing_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    # If it doesn't terminate, try to kill it
                    self.tag_processing_process.kill() 
                    self.tag_processing_process.wait()
                self.tag_processing_process = None
                self.get_logger().info("Subprocess stopped and cleaned up.")
            self.destroy_subscription(self.axis_diff_sub)
            self.get_logger().info("Destroyed axis difference subscription to lock in alignment.")
            if correction_angle-base_correction_angle > 0.0:
                # 12. Move to Pre-Pick Waypoint (High Z)
                self.get_logger().info("Step 11: Adding gear to the planning scene now that robot is in a clear position...")
                self.add_gear_to_scene2()
                time.sleep(1.0)

                # 12A. Move to Pre-Pick Waypoint (High Z)
                
                self.get_logger().info(f"Step 12A: Moving to PRE-PICK pose ({pre_pick_pose2}m)...")
                if not self.move_to_pose(pre_pick_pose2):
                    self.get_logger().error("FAILED: Could not reach PRE-PICK pose!")
                    return False
                time.sleep(5.0)

                # 12B. Move down to Final Pick Position (Low Z)
                
                self.get_logger().info(f"Step 12B: Moving to FINAL PICK pose (Z={pick_pose2.position.z:.4f}m)...")
                if self.move_cartesian_straight_line(pick_pose2):
                    self.get_logger().info("SUCCESS: Final pick pose reached!")
                else:
                    self.get_logger().error("FAILED: Could not reach FINAL PICK pose!")
                    return False
                time.sleep(5.0)

                # 13. Operate gripper (Close), ATTACH GEAR, and REMOVE WORLD COPY
                self.get_logger().info(f"Step 13: Closing gripper to GRASP position ({self.gripper_positions['grasp']}m)...")
                if self.move_gripper(self.gripper_positions['grasp']):
                    self.get_logger().info("SUCCESS: Gripper closed (or gear grasped)! Attaching gear to hand.")
                    
                    # 13A: Attach gear to the hand
                    self.attach_gear_to_hand2()
                    time.sleep(3.0)

                    # 13B: Explicitly remove the original world copy to avoid CheckStartStateCollision
                    self.remove_gear_from_world_after_attach()
                    time.sleep(3.0)
                else:
                    self.get_logger().error("FAILED: Gripper failed to close!")
                    return False
                
                time.sleep(3.0)


                #step 14: move gear straight up to avoid other gears
                self.get_logger().info(f"Step 14: Moving to PRE-rotate pose ({pre_rotate_pose.position.z:.4f}m)...")
                # if not self.move_to_pose(pre_rotate_pose):
                if not self.move_cartesian_straight_line(pre_rotate_pose):
                    self.get_logger().error("FAILED: Could not reach PRE-rotate pose!")
                    return False
                time.sleep(5.0)
                
                # Step 15: Wait for the TagAxisComparator to publish the angle.
                # self.get_logger().info("--- STARTING ALIGNMENT CHECK ---")
                
                if abs(correction_angle) < base_correction_angle: # Only rotate if the angle is significant
                    self.get_logger().info(f"Step 15: Rotating hand by {np.degrees(correction_angle):.2f} degrees around Z...")
                    if not self.rotate_joint7_directly(angle_radians=correction_angle):
                        self.get_logger().error("FAILED: Initial hand rotation for alignment failed.")
                        return False
                    time.sleep(2.0)
                else:
                    self.get_logger().info("Step 15: Correction angle near zero. Skipping rotation.")



                self.get_logger().info(f"Step 16: Moving to Post-Rotate pose ({post_rotate_pose.position.z:.4f}m)...")
                if not self.move_to_pose(post_rotate_pose):
                    self.get_logger().error("FAILED: Could not reach post-rotate pose!")
                    return False
                time.sleep(5.0)

                self.get_logger().info("Step 17: Opening gripper...")
                if self.move_gripper(self.gripper_positions['open']):
                    self.get_logger().info("SUCCESS: Gripper opened!")
                else:
                    self.get_logger().warn("Gripper movement may have failed")
                
                time.sleep(1.0)

                self.get_logger().info("Step 18: Pushing gear down")
                self.move_cartesian_straight_line(push_pose)

                self.get_logger().info("--- SCENE CLEANUP: Clearing all gear references ---\n")
                self.clear_gear_references() 
                move_success = self.move_to_joints(self.poses['home']) 
                if not move_success:
                    self.get_logger().error("SEQUENCE FAILED: Final arm move failed.")
                    return False
          
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

        

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPanda()
    pba_node = PBARobotVelocityController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(pba_node)
    
    try:
        node.get_logger().info("Waiting for initialization...")
        time.sleep(5.0)
        
        node.execute_complete_sequence()
        pba_node.get_logger().info("\\n--- Starting PBA Velocity Movement Sequence ---")
        pba_node.start_time = time.time()
        
        # 1. Move forward at 1.0 rad/s for 5 seconds (The logic from the previous step)
        move_time_sec = 5
        velocity_forward = 1.0
        
        pba_node.is_recording = True 
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < move_time_sec:
            pba_node.send_velocity_command(velocity_forward)
            executor.spin_once(timeout_sec=0.01)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        
    finally:
        node.cleanup_subprocesses() 
        node.destroy_node()
        pba_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()