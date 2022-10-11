
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import numpy as np

# Import utils.py from src folder
import sys, os, time
sys.path.append('/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/src/'))

from utils import *
from coppelia_sim_ros_interface.srv import AddOrEditObject, AddOrEditObjectResponse, RemoveObject, RemoveObjectResponse, GripperControl, GripperControlResponse
from coppelia_sim_ros_interface.msg import ObjectInfo
from sensor_msgs.msg import JointState, Image

from coppelia_sim_ros_client import CoppeliaROSInterface

rospy.init_node("coppeliaSimPublisherTopic", anonymous=True)

# Prepare scene
pose = Pose()
pose.position = Point(0., 0.3, 0.0)
pose.orientation.w = 1.
CoppeliaROSInterface.add_or_edit_object(name="object1",pose=pose, shape='sphere', color='r', dynamic='false', size=[0.1,0.1,0.1], collision='false')
#CoppeliaROSInterface.gripper_control(position=0.0)

CoppeliaROSInterface.add_or_edit_object(name='camera', pub_info='true')

time.sleep(20.0)
print("OBJECT MOVING")
for i in range(200):
    pose.position = Point(0., 0.3, 0.01*i)
    CoppeliaROSInterface.add_or_edit_object(name="object1",pose=pose)
    time.sleep(0.01)

print("DONE")
