#!/usr/bin/env python
''' Example showcase of PyRep and Panda Robot with services/topics via ROS
    - Run server: `python coppelia_sim_ros_server.py`
Updated & tested for Python3.9 and ROS2 Galactic
'''
import rclpy
import numpy as np
import sys, os, time
# Import utils.py from src folder
sys.path.append('/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/coppelia_sim_ros_interface/'))
''' For jupyter notebook, when __file__ is None
sys.path.append("/home/<user>/<your ws>/src/coppelia_sim_ros_interface/coppelia_sim_ros_interface/")
sys.path.append("/home/petr/ros2_ws/src/coppelia_sim_ros_interface/coppelia_sim_ros_interface/")
'''

from utils import *
from geometry_msgs.msg import Pose, Point, Quaternion, Polygon
from coppelia_sim_ros_client import CoppeliaROSInterfaceNode
import numpy as np

rclpy.init()
cop = CoppeliaROSInterfaceNode().r

input("Example started! Press any button to proceed")
input(">> Rotate eef")
off = np.pi/4
input(">>> 0")
cop.set_gripper(eef_rot=0-off)
input(">>> 45")
cop.set_gripper(eef_rot=np.pi/4-off)
input(">>> 90")
cop.set_gripper(eef_rot=np.pi/2-off)
input(">>> 0")
cop.set_gripper(eef_rot=0-off)
input(">>> 180")
cop.set_gripper(eef_rot=np.pi-off)
input(">>> 0")
cop.set_gripper(eef_rot=0-off)

input(">> Add line")

cop.add_line(name='line1', points=[[0.,0.,0.],[1.,1.,1.]])

input(">> Add drawer objects")

cop.add_or_edit_object(name=f"drawer1",pose=Pose(position=Point(x=0.5,y=0.0,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), object_state='closed')
input(">>> Open and move the drawer")
cop.add_or_edit_object(name=f"drawer1",pose=Pose(position=Point(x=0.5,y=0.2,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), object_state='opened')
input(">>> Create second drawer")
cop.add_or_edit_object(name=f"drawer", pose=Pose(position=Point(x=0.5,y=-0.2,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), object_state='opened')

cop.remove_object(name='drawer')
cop.remove_object(name='drawer1')

input(">> Add YCB objects")

cop.add_or_edit_object(name=f"object1",pose=Pose(position=Point(x=0.5,y=0.0,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), file='master chef can')

cop.add_or_edit_object(name=f"object2",pose=Pose(position=Point(x=0.5,y=0.2,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), file='tomato soup can')

cop.add_or_edit_object(name=f"object3",pose=Pose(position=Point(x=0.5,y=-0.2,z=0.1),orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)), file='orange')

input(">> Position target control")

cop.go_to_pose(pose=[0.5,0.0,0.2])


input(">> Add some objects to scene")

colors = ['r', 'g', 'b']
positions = [[0.3, 0.3, 0.1], [-0.5, -0.3, 0.1], [0.2, -0.3, 0.1]]
shapes = ['cylinder', 'cone', 'sphere']
for i in range(0,3):
    pose = Pose()
    pose.position = Point(x=positions[i][0],y=positions[i][1],z=positions[i][2])
    pose.orientation.w = 1.
    cop.add_or_edit_object(name=f"object{i}",pose=pose, shape=shapes[i], size=[0.075,0.075,0.075], color=colors[i], friction=0.1, mass=0.1, pub_info='false')

input(">> Move the gripper")

print("Gripper close")
cop.close_gripper()
time.sleep(2)
print("Gripper open")
cop.open_gripper()

input(">> Control Panda via IK")

cop.go_to_pose(pose=[0.5,0.0,0.3])
time.sleep(5.)
DELTA = 0.01
input(">>>")
[cop.move_above_axis(2, -DELTA) for _ in range(10)]
input(">>>")
[cop.move_above_axis(1, -DELTA) for _ in range(10)]
input(">>>")
[cop.move_above_axis(2, DELTA) for _ in range(10)]
input(">>>")
[cop.move_above_axis(1, DELTA) for _ in range(10)]

input(">> Reach the targets with panda")

pose = Pose()
pose.position = Point(x=positions[i][0], y=positions[i][1], z=positions[i][2])
pose.orientation.w = 1.
cop.add_or_edit_object(name="target_object",pose=pose, size=[0.05, 0.05, 0.05], shape='sphere', color='c', collision='false', dynamic='false')

position_min, position_max = [0.6, -0.2, 0.0], [0.8, 0.2, 0.4]

starting_pose = Pose()
starting_pose.position = Point(x=0.4,y=0.,z=0.6)
starting_pose.orientation = Quaternion(x=0.,y=1.,z=0.,w=0.)
LOOPS = 10
cop.go_to_pose(starting_pose)
for i in range(LOOPS):
    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))

    target_pose = Pose()
    target_pose.position = Point(x=pos[0],y=pos[1],z=pos[2])
    target_pose.orientation = Quaternion(x=0.,y=1.,z=0.,w=0.)

    # Change position to created object
    cop.add_or_edit_object(name="target_object",pose=target_pose, collision='false', dynamic='false')

    cop.go_to_pose(target_pose)
    input(">>>")

input(">> Add object and set the texture")

pose = Pose()
pose.position = Point(x=2.0, y=0.0, z=1.0)
pose.orientation.w = 1.
texture_file = 'wood.jpg'
cop.add_or_edit_object(name="wooden_box",pose=pose, shape='cube', size=[1.0, 1.0, 1.0], texture_file=texture_file, pub_info='true')


input(">> Exit tutorial")

## Possibly can be needed
# Listen and refresh joint states as "joints_now"
'''
joints_now = []
def joint_states_callback(data):
    joints_now = data.position

self.create_subscription(JointState, "/joint_states_coppelia", joint_states_callback)
'''
