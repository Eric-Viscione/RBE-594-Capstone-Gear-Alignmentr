# tag_axis_comparator.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import numpy as np

# Import message filters for time synchronization
from message_filters import ApproximateTimeSynchronizer, Subscriber


class TagAxisComparator(Node):
    """
    Subscribes to the world-frame long axis vector of the green and black tags,
    calculates the difference (angle) between them, and publishes the result.
    """
    def __init__(self):
        super().__init__('tag_axis_comparator')

        # ---- Parameters for Topic Names ----
        self.declare_parameter('green_axis_topic', '/tag_long_axis_world/green')
        self.declare_parameter('black_axis_topic', '/tag_long_axis_world/black')
        self.declare_parameter('output_topic', '/tag_axis_difference')
        
        green_topic = self.get_parameter('green_axis_topic').value
        black_topic = self.get_parameter('black_axis_topic').value
        output_topic = self.get_parameter('output_topic').value

        # ---- Publishers ----
        # Using PoseStamped for difference as well, storing the difference in position.x (angle)
        self.diff_pub = self.create_publisher(PoseStamped, output_topic, 10)

        # ---- Subscriptions & Synchronization ----
        # Subscribers for the two long axis vectors
        self.sub_green = Subscriber(self, PoseStamped, green_topic, qos_profile=qos_profile_sensor_data)
        self.sub_black = Subscriber(self, PoseStamped, black_topic, qos_profile=qos_profile_sensor_data)

        # Approximate Time Synchronizer (ATS)
        # Use a queue size of 10 and a slop (time tolerance) of 0.05 seconds
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_green, self.sub_black], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.comparison_callback)

        self.get_logger().info(f"Listening to {green_topic} and {black_topic}")
        self.get_logger().info(f"Publishing difference to {output_topic}")


    def comparison_callback(self, msg_green: PoseStamped, msg_black: PoseStamped):
        """
        Callback triggered when synchronized green and black axis messages arrive.
        """
        # 1. Extract Vectors (stored in the position field of PoseStamped)
        v_green = np.array([
            msg_green.pose.position.x,
            msg_green.pose.position.y,
            msg_green.pose.position.z
        ])
        v_black = np.array([
            msg_black.pose.position.x,
            msg_black.pose.position.y,
            msg_black.pose.position.z
        ])

        # 2. Normalize (though they should be unit vectors from the estimator)
        v_green = v_green / np.linalg.norm(v_green)
        v_black = v_black / np.linalg.norm(v_black)

        # 3. Calculate the Angle Difference (in radians)
        # Angle $\theta$ is found using the dot product: $\cos(\theta) = \frac{A \cdot B}{||A|| \cdot ||B||}$
        # Since A and B are unit vectors, $\cos(\theta) = A \cdot B$
        dot_product = np.dot(v_green, v_black)
        
        # Clamp the value to $[-1, 1]$ due to potential float inaccuracies
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        # Calculate angle in radians
        angle_rad = np.arccos(dot_product)
        
        # Convert to degrees for easier logging/interpretation
        angle_deg = np.degrees(angle_rad)

        self.get_logger().info(
            f"Axis Diff (deg): {angle_deg:.2f}, Green Vector: {v_green.round(3)}, Black Vector: {v_black.round(3)}"
        )

        # 4. Publish the Result
        diff_msg = PoseStamped()
        diff_msg.header = msg_green.header # Use the header from the most recent message
        diff_msg.header.frame_id = 'world'
        
        # Store the calculated angle in the x-position field for simplicity
        diff_msg.pose.position.x = angle_rad
        
        # We can also store the dot product and the angle in degrees in the y and z fields if needed
        diff_msg.pose.position.y = dot_product
        diff_msg.pose.position.z = angle_deg # angle in degrees

        self.diff_pub.publish(diff_msg)


def main(args=None):
    rclpy.init(args=args)
    tag_axis_comparator = TagAxisComparator()
    rclpy.spin(tag_axis_comparator)
    tag_axis_comparator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()