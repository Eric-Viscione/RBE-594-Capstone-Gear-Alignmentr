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
from shape_msgs.msg import SolidPrimitive # Import needed for the Cylinder
import time
from threading import Event
import numpy as np
from rclpy.executors import MultiThreadedExecutor 
from moveit_msgs.msg import AttachedCollisionObject 

# --- CONSTANTS FOR GRASPING ---
GEAR_HEIGHT = 0.025   # Height 2.5 cm
GEAR_DIAMETER = 0.07  # Diameter 7 cm (User corrected value)
GEAR_RADIUS = GEAR_DIAMETER / 2.0 # 0.035m
GEAR_BASE_Z = 0.15   # Base Z position
GEAR_CENTER_Z = GEAR_BASE_Z + (GEAR_HEIGHT / 2) # 0.1375m
# -----------------------------

class MoveItPanda(Node):
    def __init__(self):
        super().__init__('moveit_panda')
        
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
            'ready': [1.5527, 0.0877, -0.08, -1.0748, -0.1121, 1.1697, 0.6243],
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6243],
        }
        
        # Gripper positions
        self.gripper_positions = {
            'close': 0.0,
            'open': 0.04,
            'grasp': 0.0
        }
        
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

    def add_gear_to_scene(self):
        """Adds a collision object representing the gear using a SolidPrimitive (Cylinder)."""
        self.get_logger().info(f"Adding 'first_gear' (Cylinder D={GEAR_DIAMETER}m, R={GEAR_RADIUS}m) to the planning scene...")
        
        gear_co = CollisionObject()
        gear_co.header.frame_id = "world" 
        gear_co.id = "first_gear"
        
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [GEAR_HEIGHT, GEAR_RADIUS] 
        
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = 0.0
        gear_pose.position.z = GEAR_CENTER_Z 
        gear_pose.orientation.w = 1.0 
        
        gear_co.primitives.append(cylinder) 
        gear_co.primitive_poses.append(gear_pose) 
        gear_co.operation = CollisionObject.ADD 
        
        ps_msg = PlanningScene()
        ps_msg.world.collision_objects.append(gear_co)
        ps_msg.is_diff = True 
        
        self.get_logger().info("Publishing 'first_gear' (CYLINDER) to planning scene...")
        for _ in range(5):
            self.planning_scene_pub.publish(ps_msg)
            time.sleep(0.1) 
            
        self.get_logger().info("'first_gear' (CYLINDER) should now be in the planning scene.")

    def attach_gear_to_hand(self):
        """Attaches the gear to the robot hand, explicitly providing geometry for robustness."""
        self.get_logger().info("Attaching 'first_gear' to 'panda_hand'...")
        
        # Re-create geometry and pose 
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [GEAR_HEIGHT, GEAR_RADIUS] 
        
        gear_pose = Pose()
        gear_pose.position.x = 0.0
        gear_pose.position.y = 0.0
        gear_pose.position.z = GEAR_CENTER_Z 
        gear_pose.orientation.w = 1.0 

        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand" 
        
        aco.object.header.frame_id = "world"
        aco.object.id = "first_gear"
        aco.object.operation = CollisionObject.ADD 
        
        # Explicitly include geometry when attaching
        aco.object.primitives.append(cylinder) 
        aco.object.primitive_poses.append(gear_pose) 

        # Define the links the attached object is allowed to touch (CRITICAL FIX)
        aco.touch_links = ['panda_link7', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        
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
    def plan_with_moveit(self, target_joints=None, target_pose=None): 
        """Use MoveIt to plan a trajectory"""
        self.get_logger().info("Planning with MoveIt...")
        
        if not self.moveit_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server not available!")
            return None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 100 
        request.allowed_planning_time = 60.0 
        request.max_velocity_scaling_factor = 5.0
        request.max_acceleration_scaling_factor = 5.0
        
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
        
        planning_options = PlanningOptions()
        planning_options.plan_only = True
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 100
        
        # <<< REMOVED: planning_options.allow_start_state_collision = allow_start_state_collision >>>
        
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
        orient_constraint.absolute_x_axis_tolerance = 5e-5
        orient_constraint.absolute_y_axis_tolerance = 5e-5
        orient_constraint.absolute_z_axis_tolerance = 5e-5
        orient_constraint.weight = 0.8
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
                 self.get_logger().info("Gripper movement stalled (SUCCESS): Assumed gear grasped!")
                 return True
            elif gripper_result.reached_goal:
                self.get_logger().info("Gripper movement completed! (SUCCESS)")
                return True
            
        self.get_logger().error(f"Gripper movement failed with status: {status}")
        return False

    # --- MODIFIED FUNCTION: allow_start_state_collision ARGUMENT REMOVED ---
    def move_to_pose(self, target_pose: Pose): 
        """Move the end-effector to a specified Pose (position and orientation) using MoveIt planning."""
        if not self.wait_for_joint_state():
            self.get_logger().warn("Continuing with default joint state")
            
        trajectory = self.plan_with_moveit(target_pose=target_pose)
        if trajectory:
            return self.execute_trajectory(trajectory)
        return False
        
    def execute_complete_sequence(self):
        """
        Execute the complete motion sequence.
        """
        from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
        self.get_logger().info("Starting complete motion sequence...")
        
        # --- STEP 0: FORCEFUL CLEANUP ---
        self.clear_gear_references()
        
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

        # Define Poses
        PICK_Z = 0.255
        PRE_PICK_Z = 0.35
        # Quaternion for the gripper facing straight down (x=sqrt(2)/2, y=sqrt(2)/2)
        face_down_orientation = Quaternion(x=np.sqrt(2)/2, y=np.sqrt(2)/2, z=0.0, w=0.0)

        # 4A. Move to Pre-Pick Waypoint (High Z)
        pre_pick_pose = Pose(position=Point(x=0.0, y=0.0, z=PRE_PICK_Z), orientation=face_down_orientation)
        
        self.get_logger().info(f"Step 4A: Moving to PRE-PICK pose (Z={PRE_PICK_Z}m)...")
        if not self.move_to_pose(pre_pick_pose):
            self.get_logger().error("FAILED: Could not reach PRE-PICK pose!")
            return False
        time.sleep(1.0)

        # 4B. Move down to Final Pick Position (Low Z)
        target_pose = Pose(position=Point(x=0.0, y=0.0, z=PICK_Z), orientation=face_down_orientation)
        
        self.get_logger().info(f"Step 4B: Moving to FINAL PICK pose (Z={PICK_Z}m)...")
        if self.move_to_pose(target_pose):
            self.get_logger().info("SUCCESS: Final pick pose reached!")
        else:
            self.get_logger().error("FAILED: Could not reach FINAL PICK pose!")
            return False
        time.sleep(2.0)
        
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
        
        # 6. LIFT STRAIGHT UP 0.3m
        LIFT_DISTANCE = 0.30
        LIFT_Z = PICK_Z + LIFT_DISTANCE 
        lift_pose = Pose(position=Point(x=0.0, y=0.0, z=LIFT_Z), orientation=target_pose.orientation)
        
        self.get_logger().info(f"Step 6: Lifting gear straight up {LIFT_DISTANCE}m to Z={LIFT_Z}...")
        
        # <<< CRITICAL FIX: The call is now simplified, relying on touch_links >>>
        if self.move_to_pose(lift_pose):
            self.get_logger().info("SUCCESS: Lift complete!")
        else:
            self.get_logger().error("FAILED: Could not lift gear!")
            return False

        time.sleep(2.0)
        
        # 7. Move arm back to ready position (safe endpoint)
        self.get_logger().info("Step 7: Moving arm back to ready position...")
        move_success = self.move_to_joints(self.poses['ready'])
        
        if move_success:
            self.get_logger().info("SUCCESS: Ready position reached!")
        else:
            self.get_logger().error("FAILED: Could not reach ready position! Proceeding to cleanup.")
            # Do NOT return here, ensuring cleanup runs.

        self.get_logger().info("--- SCENE CLEANUP: Clearing all gear references ---\n")
        self.clear_gear_references() 

        if not move_success:
            self.get_logger().error("SEQUENCE FAILED: Final arm move failed.")
            return False
            
        self.get_logger().info("COMPLETE: All motion sequences finished successfully!")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItPanda()
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