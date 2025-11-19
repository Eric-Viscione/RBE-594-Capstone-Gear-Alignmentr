import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber


class TagAxisComparator(Node):
    """
    Subscribes to the world-frame long axis vector of the green and black tags,
    calculates the difference (angle) between them, and publishes the result.
    """
    def __init__(self):
        super().__init__('tag_axis_comparator')
        
        # 1. FIX: Define the local test flag here
        self.test_mode = True 
        
        # ---- Parameters for Topic Names ----
        self.declare_parameter('green_axis_topic', '/tag_long_axis_world/green')
        self.declare_parameter('black_axis_topic', '/tag_long_axis_world/black')
        self.declare_parameter('output_topic', '/tag_axis_difference')
        
        green_topic = self.get_parameter('green_axis_topic').value
        black_topic = self.get_parameter('black_axis_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.fixed_green_vector = np.array([1.0, 0.0, 0.0])

        # ---- Publishers ----
        self.diff_pub = self.create_publisher(PoseStamped, output_topic, 10)
        

        # ---- Subscriptions & Synchronization ----
        if not self.test_mode:
            # LIVE Mode: Use ApproximateTimeSynchronizer (ATS)
            self.get_logger().info("Running in LIVE mode: Synchronizing Green and Black axes.")
            self.sub_green = Subscriber(self, PoseStamped, green_topic, qos_profile=qos_profile_sensor_data)
            self.sub_black = Subscriber(self, PoseStamped, black_topic, qos_profile=qos_profile_sensor_data)

            self.ts = ApproximateTimeSynchronizer(
                [self.sub_green, self.sub_black], 
                queue_size=10, 
                slop=0.05
            )
            self.ts.registerCallback(self.comparison_callback)

        else:
            # TEST Mode: ONLY subscribe to the black axis
            self.get_logger().warn("Running in TEST mode: Green axis fixed to (1, 0, 0). Subscribing only to Black axis.")
            
            self.sub_black = self.create_subscription(
                PoseStamped,
                black_topic,
                self.test_mode_callback,
                qos_profile_sensor_data
            )

        self.get_logger().info(f"Publishing difference to {output_topic}")


    def test_mode_callback(self, msg_black: PoseStamped):
        """
        Callback triggered by Black axis message when in TEST mode.
        """
        # Use the fixed green vector and call the helper
        self._calculate_and_publish(self.fixed_green_vector, msg_black, is_test=True)


    def comparison_callback(self, msg_green: PoseStamped, msg_black: PoseStamped):
        """
        Callback triggered by ATS when synchronized messages arrive (LIVE mode).
        """
        # 1. Extract Vectors (stored in the position field of PoseStamped)
        v_green = np.array([
            msg_green.pose.position.x,
            msg_green.pose.position.y,
            msg_green.pose.position.z
        ])
        
        # 2. FIX: Use the unified helper function
        self._calculate_and_publish(v_green, msg_black, is_test=False)


    def _calculate_and_publish(self, v_green_source, msg_black: PoseStamped, is_test):
        """
        Core logic to calculate the angle and publish the result, used by both modes.
        """
        # 1. Extract Vectors
        v_green = v_green_source
        v_black = np.array([
            msg_black.pose.position.x,
            msg_black.pose.position.y,
            msg_black.pose.position.z
        ])

        # 2. Normalize 
        v_green = v_green / np.linalg.norm(v_green)
        v_black = v_black / np.linalg.norm(v_black)

        # 3. Calculate the Angle Difference (in radians)
        dot_product = np.dot(v_green, v_black)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)

        mode_str = "TEST" if is_test else "LIVE"
        self.get_logger().info(
            f"[{mode_str}] Axis Diff (deg): {angle_deg:.2f}, Green Vector: {v_green.round(3)}, Black Vector: {v_black.round(3)}"
        )

        # 4. Publish the Result
        diff_msg = PoseStamped()
        diff_msg.header = msg_black.header
        diff_msg.header.frame_id = 'world'
        
        diff_msg.pose.position.x = angle_rad
        diff_msg.pose.position.y = dot_product
        diff_msg.pose.position.z = angle_deg 

        self.diff_pub.publish(diff_msg)


def main(args=None):
    rclpy.init(args=args)
    tag_axis_comparator = TagAxisComparator()
    rclpy.spin(tag_axis_comparator)
    tag_axis_comparator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()