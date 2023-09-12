import numpy as np
import tf.transformations
import sys
from scipy.spatial.transform import Rotation
from numpy.testing import assert_allclose
import dataset_loaders as dsl

def quaternion_to_euler():
    # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
    # parse args for x y z w
    if len(sys.argv) != 5:
        print(("Usage: python transform.py x y z w. "
              "Note coloradar calib files contains sensor in base frame. "
              "The orientation is in saved in xyzw format."))
        exit(1)

    script, x, y, z, w = sys.argv
    quaternion = (x,y,z,w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print("rpy: {}, {}, {}".format(roll, pitch, yaw))

    Rs = Rotation.from_quat([x, y, z, w]).as_dcm()
    Rt = tf.transformations.quaternion_matrix(quaternion)
    d = Rs - Rt[:3, :3]
    assert_allclose(d, 0, atol=1e-10)

def quaternion_multiply():
    if len(sys.argv) != 3:
        print(("Usage: python transform.py file1 file2. "
              "Note coloradar calib files contains sensor in base frame. "
              "The orientation is in saved in xyzw format."))
        exit(1)
    script, f1, f2 = sys.argv
    t1, r1 = dsl.read_tf_file(f1)
    T1 = tf.transformations.quaternion_matrix(r1)
    T1[:3, 3] = t1
    print('T1\n{}'.format(T1))
    t2, r2 = dsl.read_tf_file(f2)
    T2 = tf.transformations.quaternion_matrix(r2)
    T2[:3, 3] = t2
    print("T2\n{}".format(T2))

    T = np.matmul(np.linalg.inv(T1), T2)
    print("T1.inverse * T2\n{}".format(T))
    print("t: {}".format(T[:3, 3]))
    print("R: {}".format(T[:3, :3]))


if __name__ == '__main__':
    quaternion_multiply()




