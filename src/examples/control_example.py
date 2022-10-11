
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import numpy as np
import time

# Import utils.py from src folder
import sys, os, time
print('/'.join(os.path.realpath(__file__).split('/')[:-2]))
sys.path.append('/'.join(os.path.realpath(__file__).split('/')[:-2]))

from utils import *
from coppelia_sim_ros_interface.srv import AddOrEditObject, AddOrEditObjectResponse, RemoveObject, RemoveObjectResponse, GripperControl, GripperControlResponse
from coppelia_sim_ros_interface.msg import ObjectInfo
from sensor_msgs.msg import JointState, Image

from coppelia_sim_ros_client import CoppeliaROSInterface

rospy.init_node("coppeliaSimPublisherTopic", anonymous=True)

# Prepare scene
pose = Pose()
pose.position = Point(0.4, 0.0, 0.0)
pose.orientation = Quaternion(0.7071067811865476, 0.7071067811865476, 0.0, 0.0)
CoppeliaROSInterface.add_or_edit_object(name="object1",pose=pose, shape='sphere', color='r', dynamic='false', size=[0.1,0.1,0.1], collision='false')

CoppeliaROSInterface.add_or_edit_object(name='camera', pub_info='true')

sim = CoppeliaROSInterface()

sim.gripper_control(0.5)
time.sleep(1)
sim.gripper_control(0.5)
time.sleep(1)
sim.gripper_control(0.5)
time.sleep(1)
sim.gripper_control(0.5)
time.sleep(1)
sim.gripper_control(0.5)
input("1")
sim.gripper_control(0.0)
sim.gripper_control(1.0)

input("2")

sim.gripper_control(0.0)
sim.gripper_control(0.0)

input("3")

sim.gripper_control(1.0)
sim.gripper_control(1.0)
time.sleep(2.0)
sim.gripper_control(0.0)
sim.gripper_control(0.0)

exit()

print("OBJECT MOVING")
pose.position = Point(0.4, 0.0, 0.02)
sim.go_to_pose(pose, blocking=True)
for i in range(1000):
    t1 = time.perf_counter()
    pose.position = Point(0.4, np.cos(i/100)*0.35, np.sin(i/100)*0.35+0.36)
    sim.go_to_pose(pose, blocking=False)
    sim.gripper_control(0.0, eef_rot=np.cos(i/100)*2.0)
    CoppeliaROSInterface.add_or_edit_object(name="object1",pose=pose)
    t2 = time.perf_counter()
    print(f"tt {i}: {t2-t1}")

print("DONE")
