#!/usr/bin/env python3
'''
Coppelia publisher. It communicates with CoppeliaSim via PyRep.

Purpose: Compatibility communication with Python2.7 via services.
         After transfer to Python3, communication can be direct

Loads Scene from dir var 'COPPELIA_SCENES_PATH' set to <pkg>/include/scenes/.
Three scenes are created based on ROSparam 'mirracle_config/config_file_scene' chosen by ROSparam 'mirracle_config/gripper':
    - 'none' gripper -> 'scene_panda.ttt' loaded
    - 'franka_hand' gripper -> 'scene_panda_franka_gripper.ttt' loaded
    - 'franka_hand_with_camera' gripper -> 'scene_panda_custom_gripper.ttt'

Inverse Kinematics based on ROSparam 'mirracle_config/ik_solver' as:
    - 'custom' -> Output joints to be updated by ROSparam
    - 'relaxed_ik' -> RelaxedIK, computed in separate node, here receiving '/relaxed_ik/joint_angle_solutions'
    - 'pyrep' -> Uses PyRep IK, computed here, receiving '/ee_pose_goals'
'''
import sys
import os

from utils import *

from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.objects.shape import Shape
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.camera import Camera
from pyrep.objects.joint import Joint
from pyrep.objects.force_sensor import ForceSensor
from pyrep.const import PrimitiveShape, TextureMappingMode
from pyrep.errors import ConfigurationPathError, IKError, ConfigurationError
from pyrep.objects.dummy import Dummy
from pyrep.objects.cartesian_path import CartesianPath
import numpy as np
import math
import time

from threading import Thread
import threading
sem = threading.Semaphore()

# ROS imports
from std_msgs.msg import Int8, Float64MultiArray, Header, Float32, Bool, UInt8, String
from sensor_msgs.msg import JointState, Image
from coppelia_sim_ros_interface.msg import EEPoseGoals, JointAngles
from geometry_msgs.msg import Pose, Point, Quaternion
from coppelia_sim_ros_interface.srv import AddOrEditObject, AddOrEditObjectResponse, RemoveObject, RemoveObjectResponse, GripperControl, GripperControlResponse, AddLine, AddLineResponse
from coppelia_sim_ros_interface.msg import ObjectInfo
import rospy

# Real Sense publish frequency
IMAGE_PUBLISH_FREQ = float(rospy.get_param("/coppelia/image_publish_freq"))
OBJECT_PUBLISH_FREQ = float(rospy.get_param("/coppelia/object_publish_freq"))

COPPELIA_SCENES_PATH = getCoppeliaScenePath()
COPPELIA_TEXTURES_PATH = getCoppeliaTexturesPath()
ROBOT_NAME = rospy.get_param("/mirracle_config/gripper")
GRIPPER_NAME = rospy.get_param("/mirracle_config/gripper")
IK_SOLVER = rospy.get_param("/mirracle_config/ik_solver")
IK_TOPIC = rospy.get_param("/mirracle_config/ik_topic")
# PyRep control type
PYREP_CONTROL_MODE = rospy.get_param("/mirracle_config/pyrep_control_type", 'PATH')
if PYREP_CONTROL_MODE != 'PATH': print("!!! WARNING PYREP_CONTROL_MODE is not PATH !!!")
if PYREP_CONTROL_MODE == 'PID':
    from simple_pid import PID
SCENE_FILE = rospy.get_param("/mirracle_config/config_file_scene")


class SceneObjectWrapper():
    def __init__(self, name, obj, publishInfo=False, shape=None, sensor_obj=None):
        ''' Class for wrapping info about Coppelia object
        Parameters:
            obj (Coppelia Sim object handle)
            name (Str): object name
            publishInfo (Bool): Enables publishing info about object (thread function objects_publisher)
            shape (Str): If it is a shape object, it saves the name, to be able to set the right texture transformation based on shape
        '''
        self.name = name
        self.obj = obj
        self.publishInfo = publishInfo
        self.shape = shape
        self.sensor_obj = sensor_obj

'''
self = CoppeliaSim()
from pyrep.objects import Object
drawer1 = Shape('drawer1')
drawer1.set_position([0.0,0.0,0.3])

drawer2 = Shape('drawer2')
drawer2.set_position([0.5,0.0,0.3], reset_dynamics=True)

'''
class CoppeliaSim():

    def __init__(self, dt:float=0.01, rate_to_realtime:float=1.0):
        '''
        dt: Time of simulation step
        rate_to_realtime: real time when =1
        '''
        self.joints = [0.,0.,0.,0.,0.,0.,0.] # Updated by relaxed_ik or another topic
        # Received goal pose, the program will converge to this pose
        self.eef_pose = None
        # Received goal velocities
        self.qd = None
        # Additional rotation for end-effector (joint 7), to be able to control independently
        self.eef_rot = 0.0

        # These are object types saved inside dictionary
        self.SceneObjects = {}
        if PYREP_CONTROL_MODE == 'PID':
            self.controller = SimpleController()
        # image publishers
        self.left_seq = 0
        self.right_seq = 0
        self.leftimagerpub = None
        self.rightimagerpub = None
        self.camera_pub_bool = True
        self.objectinfopub = rospy.Publisher('/coppelia/object_info', ObjectInfo, queue_size=5)

        self.pr = PyRep()
        if GRIPPER_NAME == 'none':
            SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+SCENE_FILE
        elif GRIPPER_NAME == 'franka_hand':
            SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+SCENE_FILE
        elif GRIPPER_NAME == 'franka_hand_with_camera':
            SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+SCENE_FILE
        else: raise Exception("Wrong selected gripper, (probably in demo.launch file)")
        self.dt = dt
        self.rate_to_realtime = rate_to_realtime

        self.pr.launch(SCENE_FILE_PATH, headless=False) # Run coppelia
        self.pr.set_simulation_timestep(dt=self.dt)
        self.pr.start()
        self.pr.step()

        # ROBOT loading
        self.panda = Panda()
        self.planner = PyRepPlanner(self)
        if GRIPPER_NAME == 'franka_hand':
            self.panda_gripper = PandaGripper()
        elif GRIPPER_NAME == 'franka_hand_with_camera':
            self.panda_gripper = PandaGripper()
            # Publisher for Intel Realsense D435 image
            self.leftimagerpub = rospy.Publisher('/coppelia/left_camera', Image, queue_size=5)
            self.rightimagerpub = rospy.Publisher('/coppelia/right_camera', Image, queue_size=5)
            self.LeftImager = VisionSensor("LeftImager")
            self.RightImager = VisionSensor("RightImager")
        self.panda_target = Dummy("Panda_target")
        self.focus_target = Dummy('Focus_target')
        self.focus_target_visual = Shape('Focus_target_visual')

        self.camera = Camera("DefaultCamera")
        self.panda_sensor = ProximitySensor('Panda_gripper_attachProxSensor')

        if SCENE_FILE == 'scene_panda_franka_gripper_drawers.ttt':
            self.drawer = Shape('drawer')
            self.drawer_slider = Joint('drawer_joint_bottom')
            self.drawer1 = Shape("drawer1")
            self.drawer1_slider = Joint('drawer1_joint_bottom')

        # For PyRep solver, this will publish joints solution to system
        self.ik_solution_pub = None
        if IK_SOLVER == 'relaxed_ik':
            # Receives IK solutions (computed in relaxed ik node)
            rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, self.callback_output_nodeIK)
        elif IK_SOLVER == 'pyrep':
            # Publishes IK solutions (computed here)
            self.ik_solution_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=5)
        elif IK_SOLVER == 'velocity_control':
            # Publishes IK solutions (computed here)
            rospy.Subscriber('/velocity_control', JointAngles, self.callback_velocity_control)
            self.ik_solution_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=5)
        elif IK_SOLVER == 'custom':
            # Received IK solutions (computed in specified topic)
            self.ik_solution_pub = rospy.Publisher(IK_TOPIC, JointAngles, queue_size=5)
        else: raise Exception("[ERROR*] Wrong 'ik_solver' used in demo.launch!")
        # Config subscriber
        rospy.Subscriber('/coppelia/config', String, self.callback_config)
        # Receives the end-effector pose goal
        rospy.Subscriber('/ee_pose_goals', Pose, self.callback_goal_poses)
        # Publish joint states and end-effector
        self.joint_states_pub = rospy.Publisher('/joint_states_coppelia', JointState, queue_size=5)
        self.eef_pub = rospy.Publisher('/pose_eef', Pose, queue_size=5)

        self.at_target_pub = rospy.Publisher('/coppelia_sim/at_target', Bool, queue_size=5)
        self.camera_orientation_pub = rospy.Publisher('/coppelia/camera_angle', Vector3, queue_size=5)

        # Listen for service
        rospy.Service('add_or_edit_object', AddOrEditObject, self.add_or_edit_object_callback)
        rospy.Service('add_line', AddLine, self.add_line_callback)
        rospy.Service('remove_object', RemoveObject, self.remove_object_callback)
        rospy.Service('gripper_control', GripperControl, self.gripper_control_callback)

        pose = Pose()
        pose.position = Point(*self.panda.get_tip().get_position())
        pose.orientation = Quaternion(*self.panda.get_tip().get_quaternion())
        self.eef_pose = pose

        if False and SCENE_FILE == 'scene_panda_franka_gripper_drawers.ttt':
            # 4 x 4 x 4 viz
            for x in range(4):
                for y in range(4):
                    for z in range(4):
                        object = Shape.create(type=PrimitiveShape.SPHERE, size=[0.01,0.01,0.01])

                        def position_real(position, scene_lens=[4,4,4], max_scene_len=0.8):
                            ''' Duplicite function in object.py
                            '''
                            scene_lens = np.array(scene_lens)

                            one_tile_lens = max_scene_len/scene_lens
                            y_translation = (scene_lens[1]-1)*one_tile_lens[1]/2

                            position_scaled = position * one_tile_lens
                            position_translated = position_scaled - [-0.2, y_translation, 0.]

                            return position_translated
                        object.set_position(position_real(position=[x,y,z]))
                        object.set_color([1.,1.,0.])
                        object.set_dynamic(False)
                        object.set_respondable(False)

    @property
    def frequency(self):
        return 1/self.dt

    def __enter__(self):

        rospy.init_node('coppeliaSim', anonymous=True)
        threadSimulation = Thread(target = self.simulate)
        threadSimulation.start()

        # Publishes informations about selected objects
        threadObjectPub = Thread(target = self.objects_publisher)
        threadObjectPub.start()
        # Publishes camera images
        if GRIPPER_NAME == 'franka_hand_with_camera':
            threadCameraPub = Thread(target = self.camera_publisher)
            threadCameraPub.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("Ending in 5s")
        coppeliasim.pr.stop()
        coppeliasim.pr.shutdown()

    def callback_output_nodeIK(self, data):
        ''' RelaxedIK solver option. When received joints, publish to coppeliaSim
        '''
        j = []
        for ang in data.angles:
            j.append(ang.data)
        self.joints = j

    def publish_as_output_ik(self, joints):
        ''' PyRep solver option. Publish joints as JointAngles type msg
        Parameters:
            joints (Float[7]): joints
        '''
        if np.array(joints).any():
            msg = JointAngles()
            for n,j in enumerate(joints):
                msg.angles.append(Float32(j))
            self.ik_solution_pub.publish(msg)

    def callback_goal_poses(self, data):
        ''' PyRep solver option. Receved goal poses will be applied InverseKinematics in this node
        '''
        self.eef_pose = data

    def callback_velocity_control(self, data):
        ''' PyRep solver option. Receved goal poses will be applied InverseKinematics in this node
        '''
        self.qd = data

    def callback_config(self, data):
        if data.data == 'reset':
            self.panda.set_joint_positions([0.,0.,0.,0.,1.,1.,0.], disable_dynamics=True)
            print("Joints reset!")
        else:
            print("Config was not recognized")

    def add_line_callback(self, msg):
        object = CartesianPath.create(show_line = True)
        point_array = []
        for point in msg.points.points:
            point_array.append([point.x, point.y, point.z, 0., 0., 0.])
        print(point_array)
        object.insert_control_points(point_array)
        publishInfo = False
        sem.acquire()
        self.SceneObjects[msg.name] = objw = SceneObjectWrapper(msg.name, object, publishInfo, shape=None, sensor_obj=None)
        sem.release()
        return AddLineResponse(True)

    def add_or_edit_object_callback(self, msg):
        ''' Receives service callback of object creation
            - msg defined in AddOrEditObject.srv
        '''
        # Handle special cases (Camera)
        if msg.name in ['camera', 'Camera', 'Realsense', 'realsense']:
            with sem:
                if msg.pub_info=='true': self.camera_pub_bool=True
                elif msg.pub_info=='false': self.camera_pub_bool=False
            return AddOrEditObjectResponse(True)
        if msg.name == 'Focus_target':
            with sem:
                self.focus_target_visual.set_position(extv(msg.pose.position))
                self.focus_target.set_position(extv(msg.pose.position))
            return AddOrEditObjectResponse(True)
        if msg.name == 'drawer':
            sem.acquire()
            self.drawer.set_position(extv(msg.pose.position), reset_dynamics=True)
            if msg.object_state == 'opened': self.drawer_slider.set_joint_position(position=0.2, disable_dynamics=True)
            if msg.object_state == 'semi-opened': self.drawer_slider.set_joint_position(position=0.1, disable_dynamics=True)
            if msg.object_state == 'slightly-opened': self.drawer_slider.set_joint_position(position=0.03, disable_dynamics=True)
            elif msg.object_state == 'closed': self.drawer_slider.set_joint_position(position=0.0, disable_dynamics=True)
            sem.release()
            return AddOrEditObjectResponse(True)
        if msg.name == 'drawer1':
            sem.acquire()
            self.drawer1.set_position(extv(msg.pose.position), reset_dynamics=True)
            if msg.object_state == 'opened': self.drawer1_slider.set_joint_position(position=0.2, disable_dynamics=True)
            elif msg.object_state == 'semi-opened': self.drawer1_slider.set_joint_position(position=0.1, disable_dynamics=True)
            if msg.object_state == 'slightly-opened': self.drawer1_slider.set_joint_position(position=0.03, disable_dynamics=True)
            elif msg.object_state == 'closed': self.drawer1_slider.set_joint_position(position=0.0, disable_dynamics=True)
            sem.release()
            return AddOrEditObjectResponse(True)

        # Set specified color
        if msg.color:
            if   msg.color == 'r': color_rgb = [1.,0.,0.]
            elif msg.color == 'g': color_rgb = [0.,1.,0.]
            elif msg.color == 'b': color_rgb = [0.,0.,1.]
            elif msg.color == 'c': color_rgb = [0.,1.,1.]
            elif msg.color == 'm': color_rgb = [1.,0.,1.]
            elif msg.color == 'y': color_rgb = [1.,1.,0.]
            elif msg.color == 'k': color_rgb = [0.,0.,0.]
            else: raise Exception("Picked wrong color! color is not ('r','g','b',...) rgb is"+str(msg.color))
        init_shape = ''
        if msg.init_shape:
            if msg.init_shape == 'cube':
                init_shape = PrimitiveShape.CUBOID
            elif msg.init_shape == 'sphere':
                init_shape = PrimitiveShape.SPHERE
            elif msg.init_shape == 'cylinder':
                init_shape = PrimitiveShape.CYLINDER
            elif msg.init_shape == 'cone':
                init_shape = PrimitiveShape.CONE
            else: raise Exception("Picked wrong init_shape! Shape is not '', 'cube', 'sphere', 'cylinder', or 'cone', it is: "+str(msg.init_shape))
        object = None

        sem.acquire()
        # 1. Edit existing object
        if msg.name in self.SceneObjects.keys(): # Check if object exists
            # Object won't be created, only edited
            objw = self.SceneObjects[msg.name]
            object = objw.obj
        else:
            # 2. Create Mesh
            if msg.init_file:
                if not msg.init_size: msg.init_size = [1., 1., 1.]
                if msg.init_file == 'cup':
                    object = Shape.import_mesh(filename = getCoppeliaObjectsPath()+"/cup.ply", scaling_factor=msg.init_size[0])
                else:
                    object = Shape.import_mesh(filename = msg.init_file, scaling_factor=msg.init_size[0])
            # 3. Create Shape
            elif msg.init_shape:
                if not msg.init_size: msg.init_size = [0.03, 0.03, 0.03]
                object = Shape.create(type=init_shape, size=[msg.init_size[0], msg.init_size[1], msg.init_size[2]])
            # 4. Create Box
            else:
                if not msg.init_size: msg.init_size = [0.03, 0.03, 0.03]
                object = Shape.create(type=PrimitiveShape.CUBOID, size=[msg.init_size[0], msg.init_size[1], msg.init_size[2]])

            if msg.init_collision in ['', 'true']: object.set_respondable(True)
            if msg.init_collision=='false': object.set_respondable(False)
            if msg.dynamic == '': object.set_dynamic(True)
            publishInfo = False
            # Create ForceSensor attached to the object
            sensor = ForceSensor.create()
            sensor.set_model_dynamic(True)
            sensor.set_name(f"{msg.name}_sensor")
            object.set_name(f"{msg.name}")
            object.set_parent(sensor)

            self.SceneObjects[msg.name] = objw = SceneObjectWrapper(msg.name, object, publishInfo, shape=init_shape, sensor_obj=sensor)

        if msg.pose: object.set_position(extv(msg.pose.position), reset_dynamics=True)
        if msg.pose: object.set_quaternion(quaternion_multiply(object.get_quaternion(), extq(msg.pose.orientation)))
        if msg.color: object.set_color(color_rgb)
        if msg.friction >= 0.: object.set_bullet_friction(msg.friction)
        if sum(msg.inertia) != 0:
            object.set_inertia(itertia, inertiaTransformation)

        if msg.mass >= 0.: object.set_mass(msg.mass)
        if msg.dynamic=='true': object.set_dynamic(True)
        elif msg.dynamic=='false': object.set_dynamic(False)
        if msg.pub_info=='true': objw.publishInfo = True
        elif msg.pub_info=='false': objw.publishInfo = False

        ## Testing loading texture and applying it on object
        if msg.texture_file:
            shape1,texture1 = self.pr.create_texture(filename=COPPELIA_TEXTURES_PATH+msg.texture_file)
            try:
                mapping_mode = shapeObjToWrappingMode(objw.shape)
            except KeyError:
                mapping_mode = TextureMappingMode.PLANE
            object.set_texture(texture=texture1, mapping_mode=mapping_mode, repeat_along_u=True, repeat_along_v=True)
        sem.release()

        return AddOrEditObjectResponse(True)

    def remove_object_callback(self, msg):
        ''' Receives service callback of object deletion
            - msg defined in Remove Object.srv
        '''
        if msg.name in self.SceneObjects.keys():
            sem.acquire()
            obj = self.SceneObjects.pop(msg.name)
            obj.sensor_obj.remove()
            obj.obj.remove()
            sem.release()
            return RemoveObjectResponse(True)
        else:
            print("[WARN*][Coppelia Pub] No object with name ", msg.name, " found!")
            return RemoveObjectResponse(False)

    def gripper_control_callback(self, msg):
        ''' Control the gripper with values
            - msg defined in GripperControl.srv
            - position 0.0 -> closed, 1.0 -> open
            - (pseudo) effort <0.0-1.0>
        '''
        print(f"Gripper move start to pos: {msg.position}, msg.effort: {round(msg.effort,2)}, action: {msg.action}, obj: {msg.object}, eefrot: {msg.eef_rot}")
        assert (0. <= msg.position <= 1.) or msg.position == -1, f"Not the right gripper position {msg.position} is not between 0-1"
        # TODO: Add applying force

        if msg.position != -1:
            self.panda_gripper.actuate(msg.position, # open position
                                       msg.effort)   # (pseudo) effort

        if msg.eef_rot != -1.0: self.eef_rot = msg.eef_rot

        if msg.action == 'grasp':
            grasped = self.panda_gripper.grasp(self.SceneObjects[msg.object].obj)
            i = 0
            while not grasped:
                i+=1
                time.sleep(0.1)
                grasped = self.panda_gripper.grasp(self.SceneObjects[msg.object].obj)
                if i > 50: GripperControlResponse(False)
            print("grasped", grasped)

        elif msg.action == 'release':
            self.panda_gripper.release()

        if msg.position != -1:
            i = 0
            while (self.panda_gripper.get_open_amount()[0] - msg.position) > 0.1 or (self.panda_gripper.get_open_amount()[1] - msg.position) > 0.1:
                i += 1
                time.sleep(1)
                print(f"Gripper not in final state! real: {self.panda_gripper.get_open_amount()}, desired: {msg.position}")
                self.panda_gripper.actuate(msg.position, # open position
                                           msg.effort)   # (pseudo) effort
                if self.panda_sensor.read() != -1.0:
                    break
                if i > 50: return GripperControlResponse(False)
            print(f"-- Gripper move done -- {self.panda_gripper.get_open_amount()}, {msg.position}\n")
        return GripperControlResponse(True)

    def simulate(self):
        while type(self.eef_pose) == type(False) or type(self.eef_pose) == type(None):
            time.sleep(1)

        sim_rate = rospy.Rate(self.frequency * self.rate_to_realtime)
        while not rospy.is_shutdown(): # measured 2-5ms loop

            sem.acquire()
            # viz the eef
            if self.eef_pose:
                self.panda_target.set_position([self.eef_pose.position.x, self.eef_pose.position.y, self.eef_pose.position.z])
            self.at_target = False
            # MODE 1
            if IK_SOLVER == 'relaxed_ik' or IK_SOLVER == 'custom':
                # Check if near the target
                if sameJoints(self.joints, self.panda.get_joint_positions()):
                    self.at_target = True
                else:
                    self.panda.set_joint_target_positions(self.joints)
            # MODE 2
            elif IK_SOLVER == 'pyrep':
                if samePoses(self.eef_pose, self.panda.get_tip().get_position(), accuracy=0.001) and (self.panda.get_joint_positions()[-1]-self.eef_rot)<0.05:
                    self.at_target = True
                else:
                    self.sim_step_pyrep_options()
            elif IK_SOLVER == 'velocity_control':
                self.sim_step_velocity_control()
            else: raise Exception("[ERROR*] Wrong 'ik_solver' used in demo.launch!")
            self.pr.step()
            self.pub_all_states_fun()

            sem.release()
            time.sleep(0.005)
            sim_rate.sleep()


    def set_joint_target_positions(self, joints_next):
        if joints_next is None: return
        joints_next = np.array(joints_next)
        if joints_next.any() != None:
            self.panda.set_joint_target_positions(joints_next)

    def sim_step_pyrep_options(self):
        joints_next = self.panda.get_joint_positions()
        if PYREP_CONTROL_MODE == 'PATH':
            if self.planner.done:
                plan = self.planner.pathPlan(self.eef_pose)
                if plan is False:
                    print("path not valid -> iksolve")
                    joints_next = self.pyrep_ik_solver(self.eef_pose)
                    self.set_joint_target_positions(joints_next)
            else:
                self.planner.pathStep()
        elif PYREP_CONTROL_MODE == 'PID':
            joints_next = self.controller(joints_now=self.panda.get_joint_positions(), joints_goal=self.pyrep_ik_solver(self.eef_pose))
            self.set_joint_target_positions(joints_next)
        else:
            joints_next = np.array(self.pyrep_ik_solver(self.eef_pose))
            # TODO: Add real boundaries
            if self.eef_rot is not None and joints_next.any(): joints_next[-1] = np.clip(self.eef_rot, -2.7, 2.7)
            self.set_joint_target_positions(joints_next)
        self.joints = joints_next

    def sim_step_velocity_control(self):
        joints = self.panda.get_joint_positions()
        if np.array(self.qd).any():
            for n in range(joints):
                joints[n] += self.dt * self.qd[n]
            self.panda.set_joint_positions(joints, disable_dynamics=True)

    def pub_all_states_fun(self):
        self.eef_pub_fun()
        self.joint_states_pub_fun()
        if IK_SOLVER == 'pyrep':
            self.publish_as_output_ik(self.joints) # publishes ik output to other nodes

        ## Publish at_target Bool
        self.at_target_pub.publish(Bool(self.at_target))


    def eef_pub_fun(self):
        ''' Publish eef pose
        '''
        eef_msg = Pose()
        eef_msg.position = Point(*self.panda.get_tip().get_position())
        eef_msg.orientation = Quaternion(*self.panda.get_tip().get_quaternion())
        self.eef_pub.publish(eef_msg)

    def joint_states_pub_fun(self):
        ''' Publish joint states
        '''
        joint_state_msg = JointState()
        joint_state_msg.position = self.panda.get_joint_positions()
        joint_state_msg.velocity = self.panda.get_joint_velocities()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(joint_state_msg)


    def pyrep_ik_solver(self, pose):
        try:
            if pose.orientation.w == 1000: # Orientation def: Euler
                joints = self.panda.solve_ik_via_jacobian(list(extv(pose.position)), euler=list(extq(pose.orientation))[0:3])
            else: # Orientation def: Quaternion
                joints = self.panda.solve_ik_via_jacobian(list(extv(pose.position)), quaternion=list(extq(pose.orientation)))
        except IKError:
            print("[Coppelia] Solving via jacobian failed")
            try:
                if pose.orientation.w == 1000: # Orientation def: Euler
                    joints = self.panda.solve_ik_via_sampling(list(extv(pose.position)), euler=list(extq(pose.orientation))[0:3])[0]
                else: # Orientation def: Quaternion
                    joints = self.panda.solve_ik_via_sampling(list(extv(pose.position)), quaternion=list(extq(pose.orientation)))[0]

            except ConfigurationError:
                print("[Coppelia] No configuration found for goal pose:", pose)
                joints = None
        return joints

    def camera_publisher(self):
        rate = rospy.Rate(IMAGE_PUBLISH_FREQ)
        while not rospy.is_shutdown():
            while (not rospy.is_shutdown()) and self.camera_pub_bool:
                img = Image()
                img.data = (255*np.array(self.LeftImager.capture_rgb()).reshape(2764800)).astype(int).tolist()
                img.step = 1280 * 1 * 3
                img.height = 720
                img.width = 1280
                img.encoding='rgb8'
                img.header.seq = self.left_seq = self.left_seq + 1
                img.header.stamp = rospy.Time.now()
                img.header.frame_id = ""
                self.leftimagerpub.publish(img)

                img = Image()
                img.data = (255*np.array(self.RightImager.capture_rgb()).reshape(2764800)).astype(int).tolist()
                img.step = 1280 * 1 * 3
                img.height = 720
                img.width = 1280
                img.encoding='rgb8'
                img.header.seq = self.right_seq = self.right_seq + 1
                img.header.stamp = rospy.Time.now()
                img.header.frame_id = ""
                self.rightimagerpub.publish(img)
            rate.sleep()

    def objects_publisher(self):
        '''
        '''

        def prepareObjectInfoMsg(objw):
            msg = ObjectInfo()
            msg.name = objw.name
            msg.forces, msg.torques = objw.sensor_obj.read()
            #msg.forces, msg.torques = sensor.read()
            msg.pose = Pose()
            msg.pose.position = Point(*objw.obj.get_position())
            msg.pose.orientation = Quaternion(*objw.obj.get_quaternion())
            msg.bounding_box = objw.obj.get_bounding_box()
            msg.collision = objw.obj.check_collision()
            ''' TODO: It is possible to get more contact info
            print("contact force and info ", objw.obj.get_contact())
            msg.contact_force_info = objw.obj.get_contact()
            '''
            return msg

        rate = rospy.Rate(OBJECT_PUBLISH_FREQ)
        while not rospy.is_shutdown():
            for objn in list(self.SceneObjects):
                if self.SceneObjects[objn].publishInfo:
                    msg = prepareObjectInfoMsg(self.SceneObjects[objn])
                    self.objectinfopub.publish(msg)


            self.camera_orientation_pub.publish(Vector3(*self.camera.get_orientation()))
            rate.sleep()

class PyRepPlanner():
    def __init__(self, sim):
        self.path = None
        self.done = True
        self.panda = sim.panda

    def pathPlan(self, eef_pose):
        try:
            if eef_pose.orientation.w == 1000: # Orientation def: Euler
                self.path = self.panda.get_path(
                    position=list(extv(eef_pose.position)),
                    euler=list(extq(eef_pose.orientation))[0:3])
            else: # Orientation def: Quaternion
                self.path = self.panda.get_path(
                    position=list(extv(eef_pose.position)),
                    quaternion=list(extq(eef_pose.orientation)))
            self.done = False
        except (ConfigurationPathError, RuntimeError) as e:
            print('[Coppelia] Could not find path')
            return False

    def pathStep(self):
        self.done = self.path.step()



class SimpleController():
    ''' Uses simple_pid pkg
        TODO: Tune PID parameters
    '''
    def __init__(self):
        # Joints Controller
        self.pid_joints = []
        for i in range(0,7):
            self.pid_joints.append(PID(0.01,0.005,0.002, setpoint=1))

    def __call__(self, joints_now, joints_goal):
        joints_new = []
        for i in range(0,7):
            self.pid_joints[i].setpoint = joints_goal[i]
            joints_new.append(self.pid_joints[i](joints_now[i]))
        return joints_new

    def pidControllerUpdate(self, eef_pose):
        ''' DEPRECATED
        '''
        posenow = list(self.panda.get_tip().get_position())
        posenow.extend(list(self.panda.get_tip().get_quaternion()))
        poselist = list(extv(eef_pose.position))
        poselist.extend(list(extq(eef_pose.orientation)))
        new_poselist = []
        for i in range(0,7):
            self.pid[i].setpoint = poselist[i]
            new_poselist.append(self.pid[i](posenow[i]))
        new_pose = Pose()
        new_pose.position = Point(*new_poselist[0:3])
        new_pose.orientation = Quaternion(*new_poselist[3:7])
        return new_pose

    def propController(self, pose):
        ''' DEPRECATED
            Proporcional controller, max p=0.1 meters per iteration in every axis
            - Probably not very good for this situation
        '''
        assert type(pose) == type(Pose()), "propController input not type Pose"
        p1, q1 = self.panda.get_tip().get_position(), self.panda.get_tip().get_quaternion()
        p2, q2 = list(extv(pose.position)), list(extq(pose.orientation))
        p_diff = np.subtract(p2,p1)
        q_diff = np.subtract(q2,q1)
        p_diff_cropped = np.clip(p_diff, -0.01, 0.01) # set max step
        q_diff_cropped = np.clip(q_diff, -0.01, 0.01) # set max step

        p_out = np.add(p1, p_diff_cropped)
        q_out = np.add(q1, q_diff_cropped)

        pose = Pose()
        pose.position = Point(*p_out)
        pose.orientation = Quaternion(*q_out)
        print("p1 ", p1, " pout ", p_out ," p2 ", p2)
        return pose


# __enter__ and __exit__ functions to be called
with CoppeliaSim() as coppeliasim:
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


TESTING = False
if TESTING:
    pr = PyRep()
    SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+SCENE_FILE
    pr.launch(SCENE_FILE_PATH, headless=False) # Run coppelia
    pr.start()
    panda = Panda()
    path = panda.get_path(position=[0.5,0.0,0.2], euler=[0.0,np.pi/2,0.0])
    done = False
    while not done:
        done = path.step()
        pr.step()
    done

    drawer = Shape('drawer')
    drawer.set_position([0.5,0.0,0.15])
    drawer_joint = Joint('drawer_joint_bottom')
    drawer_joint.set_joint_position([0.2])

    drawer1 = Shape('drawer1')
    drawer1.set_position([0.5,0.8,0.15])
    drawer1_joint = Joint('drawer1_joint_bottom')
    drawer1_joint.set_joint_position(0.1, disable_dynamics=True)
    drawer1_joint.is_motor_enabled()
    drawer1_joint.is_control_loop_enabled()
    drawer1_joint.get_joint_type()
    drawer1_joint.get_joint_target_position()

    Shape.import_mesh('/home/petr/Downloads/mug1.ply')



    while True:
        pr.step()
