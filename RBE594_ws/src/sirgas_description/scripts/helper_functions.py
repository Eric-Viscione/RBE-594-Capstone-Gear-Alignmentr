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