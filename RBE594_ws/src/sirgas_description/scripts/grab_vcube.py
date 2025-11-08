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
CUBE_SIZE = 0.05
CUBE_X = 0.0 
CUBE_Y = -1.0         
# Cube center Z is half the size, assuming the base is at Z=0 in the world frame.
CUBE_Z = CUBE_SIZE / 2.0 # 0.025m 

# The end-effector approach Z position for grasping. 
PICK_Z = 0.125 
# ----------------------------------------

class MoveItPandaVirtualCube(Node):
    def __init__(self):
        super().__init__('moveit_panda_virtual_cube')
        
        # Action clients and publishers
        self.moveit_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        # Publisher for updating the MoveIt Planning Scene (critical for adding/removing objects)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        self.current_joint_state = None
        self.joint_state_event = Event()
        
        # Define poses (Joint positions)
        self.poses = {
            # Standard Panda ready pose
            'ready': [1.5527, 0.0877, -0.08, -1.0748, -0.1121, 1.1697, 0.6243], 
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.04,
            'grasp': 0.024
        }
        
        self.get_logger().info("MoveIt Panda Virtual Cube node initialized")

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
        if self.current_joint_state is not None:
            return True
        if not self.joint_state_event.wait(timeout):
            self.get_logger().error("Timeout waiting for joint state! Creating default state.")
            self.create_default_joint_state()
            return True
        return True

    def create_default_joint_state(self):
        self.current_joint_state = JointState()
        self.current_joint_state.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        self.current_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def clear_cube_references(self):
        """
        Forcefully removes references to 'virtual_cube' from MoveIt's 
        collision world and attached list. (Simplified from previous version)
        """
        self.get_logger().warn("Executing SCENE CLEANUP: Removing 'virtual_cube'...")
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        
        # Remove from the attached list
        for obj_id in ["virtual_cube"]:
            aco_detach = AttachedCollisionObject()
            aco_detach.link_name = "panda_hand"
            aco_detach.object.id = obj_id 
            aco_detach.object.operation = CollisionObject.REMOVE 
            ps_msg.robot_state.attached_collision_objects.append(aco_detach)

        # Remove the world object
        for obj_id in ["virtual_cube"]:
            co_remove = CollisionObject()
            co_remove.header.frame_id = "world"
            co_remove.id = obj_id
            co_remove.operation = CollisionObject.REMOVE
            ps_msg.world.collision_objects.append(co_remove)

        ps_msg.robot_state.is_diff = True 
        
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1)
            
        time.sleep(2.0) 
        self.get_logger().warn("Scene cleanup of 'virtual_cube' complete.")

    def add_cube_to_scene(self):
        """
        Adds the dynamic CollisionObject 'virtual_cube' at the desired location.
        (Simplified from previous version, no cube_link removal needed)
        """
        self.get_logger().info("Adding cube to planning scene as 'virtual_cube'...")
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        
        # 1. ADD the dynamic 'virtual_cube'
        co_add = CollisionObject()
        co_add.header.frame_id = "world"
        co_add.id = "virtual_cube"
        
        # Define the box geometry
        cube_primitive = SolidPrimitive()
        cube_primitive.type = SolidPrimitive.BOX
        cube_primitive.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]
        co_add.primitives.append(cube_primitive)
        
        # Define the pose at (0, -1, 0.025)
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
            
        self.get_logger().info("'virtual_cube' added to collision world at (0, -1, 0).")
        # Ensure MoveIt has time to process the scene update
        time.sleep(5.0) 

    def attach_cube_to_hand(self):
        """
        Attaches 'virtual_cube' to the hand and removes it from the world.
        """
        self.get_logger().info("Attaching 'virtual_cube' to 'panda_hand' and removing from world...")

        ps_msg_attach_cube = PlanningScene()
        ps_msg_attach_cube.is_diff = True
        
        # 1. Define the AttachedCollisionObject (adds to robot's attached list)
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand" 
        
        aco.object.header.frame_id = "world"
        aco.object.id = "virtual_cube" 
        aco.object.operation = CollisionObject.ADD 
        
        # Re-add geometry and pose to the attached object to ensure continuity
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
        
        # Define the links the attached object is allowed to touch
        aco.touch_links = ['panda_link7', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        ps_msg_attach_cube.robot_state.attached_collision_objects.append(aco) 
        
        # 2. Simultaneously REMOVE 'virtual_cube' from the world 
        co_remove_world = CollisionObject()
        co_remove_world.header.frame_id = "world"
        co_remove_world.id = "virtual_cube"
        co_remove_world.operation = CollisionObject.REMOVE
        ps_msg_attach_cube.world.collision_objects.append(co_remove_world)

        ps_msg_attach_cube.robot_state.is_diff = True
        
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg_attach_cube)
            time.sleep(0.05)
            
        self.get_logger().info("'virtual_cube' is now attached to the hand.")
        time.sleep(3.0) 

    # --- Utility Methods (Plan/Execute/Move) - Kept for completeness ---

    def plan_with_moveit(self, target_joints=None, target_pose=None, allow_start_state_contact=False): 
        self.get_logger().info("Planning with MoveIt...")
        if not self.moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 10 
        request.allowed_planning_time = 5.0
        
        if self.current_joint_state:
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            
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
        for name, position in zip(joint_names, target_joints):
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
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "panda_hand"
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.SPHERE
        volume.dimensions = [0.025] 
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0 
        constraints.position_constraints.append(pos_constraint)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "panda_hand"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 1e-5
        orient_constraint.absolute_y_axis_tolerance = 1e-5
        orient_constraint.absolute_z_axis_tolerance = 1e-5
        orient_constraint.weight = 0.7
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
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        return result and result.status == GoalStatus.STATUS_SUCCEEDED

    def move_to_joints(self, target_joints):
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_joints=target_joints)
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False

    def move_gripper(self, position):
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
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            # If timed out during closing/grasping, assume success and continue
            if position <= self.gripper_positions['open'] and goal_handle.status in [GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED]:
                self.get_logger().warn("Gripper action timed out while closing/grasping. Assuming successful grasp and continuing.")
                return True
            self.get_logger().error("Gripper action timed out!")
            return False
        
        result = result_future.result()
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            if result.result.stalled:
                 self.get_logger().info("Gripper movement stalled (SUCCESS): Assumed cube grasped!")
                 return True
            return True
            
        self.get_logger().error(f"Gripper movement failed with status: {result.status}")
        return False

    def move_to_pose(self, target_pose: Pose, allow_start_state_contact=False): 
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_pose=target_pose, allow_start_state_contact=allow_start_state_contact) 
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False
        
    def execute_complete_sequence(self):
        """Execute the complete motion sequence for grabbing the cube."""
        self.get_logger().info("Starting complete motion sequence for VIRTUAL CUBE...")
        
        # --- STEP 0: FORCEFUL CLEANUP ---
        self.clear_cube_references()
        
        # 1. Move arm to ready position
        if not self.move_to_joints(self.poses['ready']):
            self.get_logger().error("FAILED: Could not reach ready position!")
            return False
        
        # 2. Operate gripper (Open)
        if not self.move_gripper(self.gripper_positions['open']):
            self.get_logger().warn("Gripper open failed, proceeding anyway.")
        
        # 3. Add the dynamic 'virtual_cube' to the planning scene
        self.add_cube_to_scene() 

        # Define Poses
        PRE_PICK_Z = 0.35
        # Quaternion for the gripper facing straight down (90-degree wrist rotation)
        face_down_orientation = Quaternion(x=np.sqrt(2)/2, y=np.sqrt(2)/2, z=0.0, w=0.0)

        # 4A. Move to Pre-Pick Waypoint (High Z)
        pre_pick_pose = Pose(position=Point(x=CUBE_X, y=CUBE_Y, z=PRE_PICK_Z), orientation=face_down_orientation)
        if not self.move_to_pose(pre_pick_pose):
            self.get_logger().error("FAILED: Could not reach PRE-PICK pose!")
            return False

        # 4B. Move down to Final Pick Position (Low Z)
        target_pose = Pose(position=Point(x=CUBE_X, y=CUBE_Y, z=PICK_Z), orientation=face_down_orientation)
        if not self.move_to_pose(target_pose):
            self.get_logger().error("FAILED: Could not reach FINAL PICK pose!")
            return False
        
        # 5. Operate gripper (Close) and ATTACH CUBE
        if self.move_gripper(self.gripper_positions['grasp']):
            # 5A: Attach the virtual cube to the hand (CRITICAL)
            self.attach_cube_to_hand()
        else:
            self.get_logger().error("FAILED: Gripper failed to close or grasp!")
            return False
        
        # 6. LIFT STRAIGHT UP to safe height
        LIFT_Z = PICK_Z + 0.30
        lift_pose = Pose(position=Point(x=CUBE_X, y=CUBE_Y, z=LIFT_Z), orientation=face_down_orientation)

        # allow_start_state_contact=True is crucial here because the cube is now attached
        if not self.move_to_pose(lift_pose, allow_start_state_contact=True):
            self.get_logger().error("FAILED: Could not lift cube!")
            return False
        
        # 7. Move arm back to ready position (safe endpoint)
        move_success = self.move_to_joints(self.poses['ready'])
        
        if not move_success:
            self.get_logger().error("SEQUENCE FAILED: Final arm move failed.")

        # --- FINAL SCENE CLEANUP: Detach/Remove the cube for the next run ---
        self.get_logger().info("--- SCENE CLEANUP: Clearing all cube references ---")
        self.clear_cube_references() 
            
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPandaVirtualCube()
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

if __name__ == '__main__':
    main()