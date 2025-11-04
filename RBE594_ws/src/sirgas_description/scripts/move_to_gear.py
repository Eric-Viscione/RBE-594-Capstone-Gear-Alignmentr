#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, RobotState

# ---------- small helpers ----------
def quat_to_mat(qx,qy,qz,qw):
    x,y,z,w = qx,qy,qz,qw
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)

def rpy_to_quat(roll, pitch, yaw):
    cy, sy = np.cos(yaw*0.5),   np.sin(yaw*0.5)
    cp, sp = np.cos(pitch*0.5), np.sin(pitch*0.5)
    cr, sr = np.cos(roll*0.5),  np.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw

def pose_to_mat44(ps: PoseStamped):
    T = np.eye(4, dtype=np.float64)
    T[:3,3] = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
    q = ps.pose.orientation
    T[:3,:3] = quat_to_mat(q.x, q.y, q.z, q.w)
    return T

def mat_to_pose(T):
    # Convert 3x3 rotation to quaternion (robust branch)
    R = T[:3,:3]
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
            qw = (R[2,1] - R[1,2]) / S
        elif i == 1:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
            qw = (R[0,2] - R[2,0]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S
            qw = (R[1,0] - R[0,1]) / S

    p = Pose()
    p.position.x, p.position.y, p.position.z = T[:3,3].tolist()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p
# -----------------------------------

class MoveToGearTip(Node):
    def __init__(self):
        super().__init__('move_to_gear_tip')

        # ---- params you might tune quickly ----
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('planning_group', 'panda_arm')
        self.declare_parameter('start_joint_fallback', [0.0]*7)
        self.declare_parameter('tag_topic', '/black_tag/pose_world')
        self.declare_parameter('base_frame', 'panda_link0')  # target pose frame for MoveIt
        # Tag -> TCP offset (desired TCP above/at the gear tip) IN TAG FRAME
        self.declare_parameter('tag_to_tcp_xyz', [0.00, 0.00, 0.05])         # meters
        self.declare_parameter('tag_to_tcp_rpy', [0.00, 0.00, 0.00])         # radians

        self.world_frame = self.get_parameter('world_frame').value
        self.group_name  = self.get_parameter('planning_group').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.tag_topic   = self.get_parameter('tag_topic').value
        self.tag_to_tcp_xyz = np.array(self.get_parameter('tag_to_tcp_xyz').value, dtype=float)
        self.tag_to_tcp_rpy = tuple(self.get_parameter('tag_to_tcp_rpy').value)

        # action clients
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client   = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        self.grip_client   = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')

        # robot state
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.current_joint_state = JointState()
        self.create_subscription(JointState, '/joint_states', self._js_cb, qos)

        # tag pose
        self.last_tag_world: PoseStamped | None = None
        self.create_subscription(PoseStamped, self.tag_topic, self._tag_cb, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("move_to_gear_tip node ready")

    # ---------- callbacks ----------
    def _js_cb(self, msg: JointState):
        # filter to panda 7 joints if you wish; or just keep as-is
        self.current_joint_state = msg

    def _tag_cb(self, msg: PoseStamped):
        self.last_tag_world = msg

    # ---------- moveit helpers ----------
    def _wait_action(self, client: ActionClient, name: str, timeout=10.0):
        if not client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f"Action server {name} not available")
            return False
        return True

    def _plan(self, target_pose: Pose):
        if not self._wait_action(self.moveit_client, 'MoveGroup', 15.0):
            return None

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # set start state (fallback if none received)
        rs = RobotState()
        if len(self.current_joint_state.name) >= 7:
            rs.joint_state = self.current_joint_state
        else:
            rs.joint_state.name = [
                'panda_joint1','panda_joint2','panda_joint3',
                'panda_joint4','panda_joint5','panda_joint6','panda_joint7'
            ]
            rs.joint_state.position = self.get_parameter('start_joint_fallback').value
        req.start_state = rs

        # pose goal in base frame
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        goal = Constraints()

        # position (small sphere tolerance)
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = 'panda_hand'
        sph = SolidPrimitive()
        sph.type = SolidPrimitive.SPHERE
        sph.dimensions = [0.01]  # 1 cm tolerance
        pc.constraint_region.primitives.append(sph)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        goal.position_constraints.append(pc)

        # orientation (loose-ish)
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = 'panda_hand'
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.7
        oc.absolute_y_axis_tolerance = 0.7
        oc.absolute_z_axis_tolerance = 0.7
        oc.weight = 1.0
        goal.orientation_constraints.append(oc)

        req.goal_constraints.append(goal)

        opts = PlanningOptions()
        opts.plan_only = True
        opts.replan = True
        opts.replan_attempts = 3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = opts

        self.get_logger().info("Sending plan request…")
        fut = self.moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Planning goal rejected")
            return None

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()
        if res and res.result and res.result.error_code.val == res.result.error_code.SUCCESS:
            jt: JointTrajectory = res.result.planned_trajectory.joint_trajectory
            self.get_logger().info(f"Plan OK: {len(jt.points)} points")
            return jt
        self.get_logger().error(f"Planning failed (code={getattr(res.result.error_code,'val','?')})")
        return None

    def _execute(self, jt: JointTrajectory):
        if not self._wait_action(self.exec_client, 'FollowJointTrajectory', 10.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        gh_fut = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut)
        gh = gh_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Execution goal rejected")
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("Execution result received")
        return True

    # ---------- main logic ----------
    def _build_target_from_tag(self) -> Pose | None:
        if self.last_tag_world is None:
            self.get_logger().warn("No tag pose yet")
            return None

        # world -> tag
        T_w_tag = pose_to_mat44(self.last_tag_world)

        # tag -> tcp (offset/orientation in tag frame)
        r,p,y = self.tag_to_tcp_rpy
        qx,qy,qz,qw = rpy_to_quat(r,p,y)
        R_tag_tcp = quat_to_mat(qx,qy,qz,qw)
        T_tag_tcp = np.eye(4)
        T_tag_tcp[:3,:3] = R_tag_tcp
        T_tag_tcp[:3, 3] = self.tag_to_tcp_xyz

        # world -> tcp
        T_w_tcp = T_w_tag @ T_tag_tcp

        # panda_link0 <- world (lookup)
        try:
            # note: lookup target->source (child, parent) shape = target wrt source
            tf_p0_w = self.tf_buffer.lookup_transform(self.base_frame, self.world_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"No TF {self.base_frame} <- {self.world_frame}: {e}")
            return None
        T_p0_w = np.eye(4)
        t = tf_p0_w.transform.translation
        q = tf_p0_w.transform.rotation
        T_p0_w[:3,:3] = quat_to_mat(q.x,q.y,q.z,q.w)
        T_p0_w[:3, 3] = [t.x, t.y, t.z]

        # target in panda_link0
        T_p0_tcp = T_p0_w @ T_w_tcp
        NUDGE_BASE = np.array([0.0, 0.0, 1], dtype=float)  # x,y,z in panda_link0
        T_p0_tcp[:3, 3] += NUDGE_BASE
        target_pose = mat_to_pose(T_p0_tcp)
        return target_pose

    def run_once(self):
        self.get_logger().info("Waiting for tag and TF…")
        # give the subscriptions and TF a moment
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_tag_world is not None:
                break

        if self.last_tag_world is None:
            self.get_logger().error("No /black_tag/pose_world received; aborting.")
            return

        target = self._build_target_from_tag()
        if target is None:
            self.get_logger().error("Could not build target from tag")
            return

        self.get_logger().info(
            f"Target in {self.base_frame}: "
            f"pos=({target.position.x:.3f},{target.position.y:.3f},{target.position.z:.3f})"
        )
        plan = self._plan(target)
        if plan is None:
            return
        self._execute(plan)


def main():
    rclpy.init()
    node = MoveToGearTip()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
