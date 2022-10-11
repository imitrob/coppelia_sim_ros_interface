#!/usr/bin/env python3
''' Example showcase of PyRep and Panda Robot with services/topics via ROS
    - coppelia_sim.launch needed to launch interface, use example.launch
'''

from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
import numpy as np

# Import utils.py from src folder
import sys
import os
sys.path.append('/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/src/'))

from utils import *
from coppelia_sim_ros_interface.srv import AddOrEditObject, AddOrEditObjectResponse, RemoveObject, RemoveObjectResponse, GripperControl, GripperControlResponse
from coppelia_sim_ros_interface.msg import ObjectInfo
from sensor_msgs.msg import JointState, Image

rospy.init_node("coppeliaSimPublisherTopic", anonymous=True)

input("Example started! Press any button to proceed")
input(">> Add some objects to scene")

def add_or_edit_object_call(file="", name=None, pose=None, shape=None, size=[0.075,0.075,0.075], color="", friction=0.1, frame_id="", init_collision='true', mass=0.1, inertia=np.zeros(9), inertiaTransformation=np.zeros(12), dynamic='true', pub_info='false', texture_file="", object_state=""):
    rospy.wait_for_service('add_or_edit_object')
    try:
        add_or_edit_object = rospy.ServiceProxy('add_or_edit_object', AddOrEditObject)
        response = add_or_edit_object(init_file=file, name=name, pose=pose, init_shape=shape, init_size=size, color=color, friction=friction, frame_id=frame_id, init_collision=init_collision, mass=mass, inertia=inertia, inertiaTransformation=inertiaTransformation, dynamic=dynamic, pub_info=pub_info, texture_file=texture_file, object_state=object_state)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

colors = ['r', 'g', 'b']
positions = [[0.3, 0.3, 0.1], [-0.5, -0.3, 0.1], [0.2, -0.3, 0.1]]
shapes = ['cylinder', 'cone', 'sphere']
for i in range(0,3):
    pose = Pose()
    pose.position = Point(*positions[i])
    pose.orientation.w = 1.
    add_or_edit_object_call(name="object"+str(i),pose=pose, shape=shapes[i], color=colors[i])

#input("Get object data")
#add_or_edit_object_call(name="wooden_box",pose=pose, shape='cube', size=[1.0,1.0,1.0], texture_file=texture_file)


input(">> Move the gripper")

def gripper_control_call(position, effort=0.04, action="", object=""):
    rospy.wait_for_service('gripper_control')
    try:
        gripper_control = rospy.ServiceProxy('gripper_control', GripperControl)
        response = gripper_control(position, effort, action, object)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

gripper_control_call(position=0.0)

input(">> Control Panda via IK")

# Listen and refresh endeffector poses as "eef_poses_now"
eef_pose_now = []
def eef_pose_callback(data):
    global eef_pose_now
    eef_pose_now = data


# Subscribe for endeffector poses and wait for first message
rospy.Subscriber("/pose_eef", Pose, eef_pose_callback)

# Publisher for target endeffector pose
ee_pose_goals_pub = rospy.Publisher("/ee_pose_goals", Pose, queue_size=5)

DELTA = 0.01
def go_to_pose(pose):
    ''' Execute movement to desired pose, waits until done
    '''
    rate = rospy.Rate(1)
    while (not samePoses(pose, eef_pose_now)):
        ee_pose_goals_pub.publish(pose)
        rate.sleep()
        print("pose", pose, "  eef pose now ", eef_pose_now)

def move_above_axis(index, delta):
    pos = list(extv(eef_pose_now.position))
    quat = list(extq(eef_pose_now.orientation))
    pos[index] += delta
    pose = Pose()
    pose.position = Point(*pos)
    pose.orientation = Quaternion(*quat)
    go_to_pose(pose)


rospy.wait_for_message("/pose_eef", Pose)

[move_above_axis(2, -DELTA) for _ in range(30)]
[move_above_axis(1, -DELTA) for _ in range(10)]
[move_above_axis(2, DELTA) for _ in range(10)]
[move_above_axis(1, DELTA) for _ in range(10)]

input(">> Reach the targets with panda")

pose = Pose()
pose.position = Point(*positions[i])
pose.orientation.w = 1.
add_or_edit_object_call(name="target_object",pose=pose, size=[0.05, 0.05, 0.05], shape='sphere', color='c')

#TODO: Add static/respondable into object creation

position_min, position_max = [0.6, -0.2, 0.0], [0.8, 0.2, 0.4]

starting_pose = Pose()
starting_pose.position = Point(0.4,0.,0.6)
starting_pose.orientation.y = 1.
LOOPS = 10
go_to_pose(starting_pose)
for i in range(LOOPS):

    # Reset the arm at the start of each 'episode'
    go_to_pose(starting_pose)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))

    target_pose = Pose()
    target_pose.position = Point(*pos)
    target_pose.orientation.y = 1.

    # Change position to created object
    add_or_edit_object_call(name="target_object",pose=target_pose, init_collision='false', dynamic='false')

    go_to_pose(target_pose)

input(">> Add object and set the texture")

pose = Pose()
pose.position = Point(1.0, 0.0, 1.0)
pose.orientation.w = 1.
texture_file = 'wood.jpg'
add_or_edit_object_call(name="wooden_box",pose=pose, shape='cube', size=[1.0, 1.0, 1.0], texture_file=texture_file, pub_info='true')


input(">> Exit tutorial")

## Possibly can be needed
# Listen and refresh joint states as "joints_now"
'''
joints_now = []
def joint_states_callback(data):
    joints_now = data.position

rospy.Subscriber("/joint_states_coppelia", JointState, joint_states_callback)
'''
