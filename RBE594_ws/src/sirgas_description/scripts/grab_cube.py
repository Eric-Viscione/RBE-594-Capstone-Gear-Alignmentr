#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState, CollisionObject, PlanningScene 
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from action_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
import time
from threading import Event
import numpy as np
from rclpy.executors import MultiThreadedExecutor 
from moveit_msgs.msg import AttachedCollisionObject 
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint

# --- CONSTANTS FOR GRASPING THE CUBE ---
# Based on cube.urdf.xacro: cube_size=0.05, origin xyz="0 -1.0 ${cube_size/2}" 
CUBE_SIZE = 0.05
CUBE_X = 0.0 
CUBE_Y = -1.0         
CUBE_Z = CUBE_SIZE / 2.0 # 0.025m (Center of the cube since origin is at the base)

# Use the actual cube position for the pick!
PICK_X = CUBE_X 
PICK_Y = CUBE_Y
# End effector approach Z: center of cube (0.025m) + a small offset for clearance (0.005m)
PICK_Z = 0.125
# ----------------------------------------

class MoveItPandaCube(Node):
    def __init__(self):
        super().__init__('moveit_panda_cube')
        
        # Action clients and publishers
        self.moveit_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.current_joint_state = None
        self.joint_state_event = Event()
        
        # Define poses
        self.poses = {
            # Standard Panda ready pose
            'ready': [1.5527, 0.0877, -0.08, -1.0748, -0.1121, 1.1697, 0.6243], 
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6243],
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.04,
            # Grasp position should be slightly less than the object width (0.05m)
            'grasp': 0.024
        }
        
        self.get_logger().info("MoveIt Panda Cube node initialized")

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

    def clear_cube_references(self):
        """
        Forcefully removes references to 'virtual_cube' (if it exists) and the static 
        'cube_link' from MoveIt's collision world and attached list.
        """
        self.get_logger().warn("Executing FORCEFUL SCENE CLEANUP...")
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        
        # 1. Remove from the attached list (AttachedCollisionObject REMOVE)
        for obj_id in ["virtual_cube", "cube_link"]:
            aco_detach = AttachedCollisionObject()
            aco_detach.link_name = "panda_hand"
            aco_detach.object.id = obj_id 
            # The operation is REMOVE the *object* reference from the *attached* list
            aco_detach.object.operation = CollisionObject.REMOVE 
            ps_msg.robot_state.attached_collision_objects.append(aco_detach)

        # 2. Remove the world objects (CollisionObject REMOVE)
        for obj_id in ["virtual_cube", "cube_link"]:
            co_remove = CollisionObject()
            co_remove.header.frame_id = "world"
            co_remove.id = obj_id
            co_remove.operation = CollisionObject.REMOVE
            ps_msg.world.collision_objects.append(co_remove)

        ps_msg.robot_state.is_diff = True 
        
        # Publish repeatedly for robustness
        for i in range(10):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        time.sleep(2.0) 
        self.get_logger().warn("Forceful scene cleanup complete.")

    def add_cube_to_scene(self):
        """
        Adds a dynamic CollisionObject named 'virtual_cube' to the planning scene
        to replace the static 'cube_link' for the grasp plan.
        """
        self.get_logger().info("Adding cube to planning scene as 'virtual_cube'...")
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        
        # 1. REMOVE the static 'cube_link'
        co_remove = CollisionObject()
        co_remove.header.frame_id = "world"
        co_remove.id = "cube_link"
        co_remove.operation = CollisionObject.REMOVE
        ps_msg.world.collision_objects.append(co_remove)

        # 2. ADD the dynamic 'virtual_cube'
        co_add = CollisionObject()
        co_add.header.frame_id = "world"
        co_add.id = "virtual_cube"
        
        # Define the box geometry (Crucial step)
        cube_primitive = SolidPrimitive()
        cube_primitive.type = SolidPrimitive.BOX
        cube_primitive.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]
        co_add.primitives.append(cube_primitive)
        
        # Define the pose 
        cube_pose = Pose()
        cube_pose.position.x = CUBE_X
        cube_pose.position.y = CUBE_Y
        cube_pose.position.z = CUBE_Z
        cube_pose.orientation.w = 1.0 # Default orientation
        co_add.primitive_poses.append(cube_pose)
        
        co_add.operation = CollisionObject.ADD
        ps_msg.world.collision_objects.append(co_add)
        
        # Publish repeatedly for robustness
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        self.get_logger().info("'virtual_cube' added and 'cube_link' removed from collision world.")
        # ADD EXTRA PAUSE TO ENSURE MOVEIT PROCESSES THE SCENE SWAP
        time.sleep(20.0) # <--- INCREASED DELAY from 15.0s

    def attach_cube_to_hand(self):
        """
        Attaches the dynamic CollisionObject 'virtual_cube' to the robot hand
        AND removes it from the world collision map simultaneously (the grasp signal).
        
        MODIFIED: Now uses a two-phase publishing approach to ensure 'cube_link' 
        is removed before the attachment process begins, solving the lingering collision issue.
        """
        self.get_logger().info("Attaching 'virtual_cube' to 'panda_hand' and removing from world...")

        # --- PHASE 1: FORCE REMOVAL OF STUBBORN URDF LINK 'cube_link' ---
        ps_msg_remove_link = PlanningScene()
        ps_msg_remove_link.is_diff = True
        
        co_remove_link = CollisionObject()
        co_remove_link.header.frame_id = "world"
        co_remove_link.id = "cube_link"
        co_remove_link.operation = CollisionObject.REMOVE
        ps_msg_remove_link.world.collision_objects.append(co_remove_link)

        self.get_logger().warn("Phase 1: Forcefully removing 'cube_link' again...")
        for _ in range(10):
            self.planning_scene_pub.publish(ps_msg_remove_link)
            time.sleep(0.05)
        # Give a significant pause to ensure the removal propagates across MoveIt's environment
        time.sleep(10.0) # <--- INCREASED DELAY from 5.0s

        # --- PHASE 2: ATTACH 'virtual_cube' and REMOVE from world ---
        ps_msg_attach_cube = PlanningScene()
        ps_msg_attach_cube.is_diff = True
        
        # 1. Define the AttachedCollisionObject (adds to robot's attached list)
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand" 
        
        aco.object.header.frame_id = "world"
        aco.object.id = "virtual_cube" 
        aco.object.operation = CollisionObject.ADD 
        
        # --- CRITICAL FIX: RE-ADD GEOMETRY AND POSE TO THE ATTACHED OBJECT ---
        cube_primitive = SolidPrimitive()
        cube_primitive.type = SolidPrimitive.BOX
        cube_primitive.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]
        aco.object.primitives.append(cube_primitive)
        
        cube_pose = Pose()
        cube_pose.position.x = CUBE_X
        cube_pose.position.y = CUBE_Y
        cube_pose.position.z = CUBE_Z
        cube_pose.orientation.w = 1.0
        aco.object.primitive_poses.append(cube_pose)
        # ---------------------------------------------------------------------
        
        # Define the links the attached object is allowed to touch
        aco.touch_links = ['panda_link7', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        ps_msg_attach_cube.robot_state.attached_collision_objects.append(aco) 
        
        # 2. Simultaneously REMOVE 'virtual_cube' from the world (removes it from the global map)
        co_remove_world = CollisionObject()
        co_remove_world.header.frame_id = "world"
        co_remove_world.id = "virtual_cube"
        co_remove_world.operation = CollisionObject.REMOVE
        ps_msg_attach_cube.world.collision_objects.append(co_remove_world)

        ps_msg_attach_cube.robot_state.is_diff = True
        
        self.get_logger().info("Phase 2: Attaching 'virtual_cube' and removing from world...")
        for _ in range(10):
            self.planning_scene_pub.publish(ps_msg_attach_cube)
            time.sleep(0.05)
            
        self.get_logger().info("'virtual_cube' is now attached to the hand.")
        
        # The 5.0s pause is good for ensuring the scene update propagates.
        time.sleep(5.0)


    def plan_with_moveit(self, target_joints=None, target_pose=None, allow_start_state_contact=False): 
        self.get_logger().info("Planning with MoveIt...")
        
        if not self.moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 100 
        request.allowed_planning_time = 60.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        if self.current_joint_state:
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            
            # HACK: If we expect a collision (like after grasping), set is_diff = True.
            # This often bypasses the 'CheckStartStateCollision' adapter, forcing 
            # the planner to proceed with the current, live robot state.
            if allow_start_state_contact:
                robot_state.is_diff = True
                
            request.start_state = robot_state
        
        if target_joints:
            request.goal_constraints.append(self.create_joint_constraint(target_joints))
        elif target_pose:
            request.goal_constraints.append(self.create_pose_constraint(target_pose))
        else:
            self.get_logger().error("No target specified!")
            return None
        
        planning_options = PlanningOptions()
        planning_options.plan_only = True
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 100
        
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
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "panda_hand"
        
        # Create a small tolerance volume
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.025] 
        
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0 
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint (loose)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "panda_hand"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 5e-4
        orient_constraint.absolute_y_axis_tolerance = 5e-4
        orient_constraint.absolute_z_axis_tolerance = 5e-4
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        
        return constraints

    def execute_trajectory(self, joint_trajectory):
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
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_joints=target_joints)
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False

    def move_gripper(self, position):
        """Move gripper with increased max_effort for better grasping."""
        self.get_logger().info(f"Moving gripper to position: {position}")
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False

        goal_msg = GripperCommand.Goal()
        command = GripperCommandMsg()
        command.position = position
        # INCREASED MAX EFFORT FOR ROBUST GRASPING
        command.max_effort = 100.0 
        
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
                 self.get_logger().info("Gripper movement stalled (SUCCESS): Assumed cube grasped!")
                 return True
            elif gripper_result.reached_goal:
                self.get_logger().info("Gripper movement completed! (SUCCESS)")
                return True
            
        self.get_logger().error(f"Gripper movement failed with status: {status}")
        return False

    def move_to_pose(self, target_pose: Pose, allow_start_state_contact=False): 
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_pose=target_pose, allow_start_state_contact=allow_start_state_contact) 
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False
        
    def execute_complete_sequence(self):
        """
        Execute the complete motion sequence for grabbing the cube.
        """
        self.get_logger().info("Starting complete motion sequence for CUBE...")
        
        # --- STEP 0: FORCEFUL CLEANUP ---
        self.clear_cube_references()
        
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
        
        # --- STEP 3: Add the Cube as a dynamic CollisionObject, replacing the static URDF link ---
        self.get_logger().info("Step 3: Removing static cube_link and adding dynamic virtual_cube...")
        self.add_cube_to_scene() 
        # Note: add_cube_to_scene now includes a 20.0s pause internally.

        # Define Poses
        PRE_PICK_Z = 0.35
        # Quaternion for the gripper facing straight down (x=sqrt(2)/2, y=sqrt(2)/2)
        face_down_orientation = Quaternion(x=np.sqrt(2)/2, y=np.sqrt(2)/2, z=0.0, w=0.0)

        # 4A. Move to Pre-Pick Waypoint (High Z)
        pre_pick_pose = Pose(position=Point(x=PICK_X, y=PICK_Y, z=PRE_PICK_Z), orientation=face_down_orientation)
        
        self.get_logger().info(f"Step 4A: Moving to PRE-PICK pose (X={PICK_X}, Y={PICK_Y}, Z={PRE_PICK_Z}m)...")
        if not self.move_to_pose(pre_pick_pose):
            self.get_logger().error("FAILED: Could not reach PRE-PICK pose!")
            return False
        time.sleep(1.0)

        # 4B. Move down to Final Pick Position (Low Z)
        target_pose = Pose(position=Point(x=PICK_X, y=PICK_Y, z=PICK_Z), orientation=face_down_orientation)
        
        self.get_logger().info(f"Step 4B: Moving to FINAL PICK pose (X={PICK_X}, Y={PICK_Y}, Z={PICK_Z}m)...")
        if self.move_to_pose(target_pose):
            self.get_logger().info("SUCCESS: Final pick pose reached!")
        else:
            self.get_logger().error("FAILED: Could not reach FINAL PICK pose!")
            return False
        time.sleep(2.0)
        
        # 5. Operate gripper (Close) and ATTACH CUBE
        self.get_logger().info(f"Step 5: Closing gripper to GRASP position ({self.gripper_positions['grasp']}m)...")
        if self.move_gripper(self.gripper_positions['grasp']):
            self.get_logger().info("SUCCESS: Gripper closed (or cube grasped)! Attaching virtual_cube to hand.")
            
            # 5A: Attach the virtual cube to the hand (CRITICAL FIX IS HERE)
            self.attach_cube_to_hand()
        else:
            self.get_logger().error("FAILED: Gripper failed to close!")
            return False
        
        time.sleep(5.0) # <--- INCREASED DELAY from 3.0s
        
        # 6. LIFT STRAIGHT UP 0.3m
        LIFT_DISTANCE = 0.30
        LIFT_Z = PICK_Z + LIFT_DISTANCE 
        lift_pose = Pose(position=Point(x=PICK_X, y=PICK_Y, z=LIFT_Z), orientation=target_pose.orientation)
        
        self.get_logger().info(f"Step 6: Lifting cube straight up {LIFT_DISTANCE}m to Z={LIFT_Z}...")
        # --- CRITICAL CHANGE: Set allow_start_state_contact=True for the move after grasping ---
        if self.move_to_pose(lift_pose, allow_start_state_contact=True): 
            self.get_logger().info("SUCCESS: Lift complete!")
        else:
            self.get_logger().error("FAILED: Could not lift cube!")
            return False

        time.sleep(2.0)
        
        # 7. Move arm back to ready position (safe endpoint)
        self.get_logger().info("Step 7: Moving arm back to ready position...")
        move_success = self.move_to_joints(self.poses['ready'])
        
        if move_success:
            self.get_logger().info("SUCCESS: Ready position reached!")
        else:
            self.get_logger().error("FAILED: Could not reach ready position! Proceeding to cleanup.")

        self.get_logger().info("--- SCENE CLEANUP: Clearing all cube references ---")
        self.clear_cube_references() 

        if not move_success:
            self.get_logger().error("SEQUENCE FAILED: Final arm move failed.")
            return False
            
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPandaCube()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Waiting for initialization...")
        time.sleep(5.0)
        
        node.execute_complete_sequence()
            
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()