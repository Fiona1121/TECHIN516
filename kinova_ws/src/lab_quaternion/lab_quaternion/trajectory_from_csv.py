import numpy as np
from pyquaternion import Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm
from geometry_msgs.msg import Pose
import rclpy

# load position from csv
data_path = './data.csv'
data = np.loadtxt(data_path, delimiter=',')

# interpolate quaternion orientations
qStart = Quaternion(w=0.0, x=0.000, y=0.000, z=0.000)
qEnd = Quaternion(w=1.000, x=0.000, y=0.000, z=0.000)

qList = []
for q in Quaternion.intermediates(qStart, qEnd, len(data)-2, include_endpoints=True):
    qList.append(q.elements)

def main():
    # move arm to interpolated positions
    rclpy.init()
    arm = Gen3LiteArm()
    for i in range(len(data)):
        pose = Pose()
        pose.position.x = data[i][0]
        pose.position.y = data[i][1]
        pose.position.z = data[i][2]
        pose.orientation.x = qList[i][0]
        pose.orientation.y = qList[i][1]
        pose.orientation.z = qList[i][2]
        pose.orientation.w = qList[i][3]
        arm.inverse_kinematic_movement(pose)
    rclpy.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()