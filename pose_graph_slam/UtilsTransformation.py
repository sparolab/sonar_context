from matplotlib.pyplot import axis
import transforms3d as tf
import numpy as np

def get_transform_quat(quat, trans):
    T         = np.eye(4)
    T[:3, :3] = tf.quaternions.quat2mat(quat)
    T[:3, 3]  = trans
    return T
    
def get_relative_transform(curr_pose, prev_pose):
    relative = np.matmul(np.linalg.inv(prev_pose), curr_pose) 
    return relative 