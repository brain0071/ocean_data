
import tf

def quaternion_to_euler(qw, qx, qy, qz):
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([qx, qy, qz, qw]) 
        return np.array([roll, pitch, yaw])