#!/usr/bin/env python
'''
Coppelia publisher. It communicates with CoppeliaSim via PyRep.

Purpose: Compatibility communication with Python2.7 via services.
         After transfer to Python3, communication can be direct

Loads Scene from dir var 'COPPELIA_SCENES_PATH' set to <pkg>/include/scenes/.
Three scenes are created based on ROSparam 'project_config/config_file_scene' chosen by ROSparam 'project_config/gripper':
    - 'none' gripper -> 'scene_panda.ttt' loaded
    - 'franka_hand' gripper -> 'scene_panda_franka_gripper.ttt' loaded
    - 'franka_hand_with_camera' gripper -> 'scene_panda_custom_gripper.ttt'

Inverse Kinematics based on ROSparam 'project_config/ik_solver' as:
    - 'custom' -> Output joints to be updated by ROSparam
    - 'relaxed_ik' -> RelaxedIK, computed in separate node, here receiving '/relaxed_ik/joint_angle_solutions'
    - 'pyrep' -> Uses PyRep IK, computed here, receiving '/ee_pose_goals'
'''
import sys, os, time
import numpy as np

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
from pyrep.backend import sim

from threading import Thread
import threading
sem = threading.Semaphore()

# ROS imports
from std_msgs.msg import Int8, Float64MultiArray, Header, Float32, Bool, UInt8, String
from sensor_msgs.msg import JointState, Image
from coppelia_sim_ros_interface.msg import JointAngles
from geometry_msgs.msg import Pose, Point, Quaternion
from coppelia_sim_ros_interface.srv import AddOrEditObject, RemoveObject, GripperControl, AddLine
from coppelia_sim_ros_interface.msg import ObjectInfo
from coppelia_sim_ros_interface.msg import EEPoseGoals
import rclpy
import rclpy.node

COPPELIA_SCENES_PATH = getCoppeliaScenePath()
COPPELIA_TEXTURES_PATH = getCoppeliaTexturesPath()
COPPELIA_OBJECTS_PATH = getCoppeliaObjectsPath()

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

class CoppeliaSim(rclpy.node.Node):

    def __init__(self, dt:float=0.01, rate_to_realtime:float=1.0):
        super().__init__("CoppeliaSimServerRosNode")

        # Real Sense publish frequency
        if self.has_parameter("image_publish_freq"):
            self.IMAGE_PUBLISH_FREQ = float(self.get_parameter("image_publish_freq"))
        else:
            self.IMAGE_PUBLISH_FREQ = 2.
        if self.has_parameter("/coppelia/object_publish_freq"):
            self.OBJECT_PUBLISH_FREQ = float(self.get_parameter("/coppelia/object_publish_freq"))
        else:
            self.OBJECT_PUBLISH_FREQ = 2
        if self.has_parameter("/project_config/ik_solver"):
            self.IK_SOLVER = rclpy.get_parameter("/project_config/ik_solver")
        else:
            self.IK_SOLVER = "pyrep"
        if self.has_parameter("/project_config/ik_topic"):
            self.IK_TOPIC = rclpy.get_parameter("/project_config/ik_topic")
        else:
            self.IK_TOPIC = ""

        if self.has_parameter("/project_config/composite_objects"):
            self.COMPOSITE_OBJECTS = rclpy.get_parameter("/project_config/composite_objects")
        else:
            self.COMPOSITE_OBJECTS = "drawer,drawer1"
        # PyRep control type
        self.PYREP_CONTROL_MODE = 'PATH'

        if self.has_parameter("/project_config/config_file_scene"):
            self.SCENE_FILE = rclpy.get_parameter("/project_config/config_file_scene")
        else:
            self.SCENE_FILE = "scene_panda_gripper_mirracle_lite.ttt" # "scene_panda_gripper_cbgo.ttt" #
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
        # image publishers
        self.left_seq = 0
        self.right_seq = 0
        self.leftimagerpub = None
        self.rightimagerpub = None
        self.camera_pub_bool = True
        self.objectinfopub = self.create_publisher(ObjectInfo, '/coppelia/object_info', 5)

        self.pr = PyRep()

        SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+self.SCENE_FILE
        self.dt = dt
        self.rate_to_realtime = rate_to_realtime
        self.pr.launch(SCENE_FILE_PATH, headless=False) # Run coppelia
        self.pr.set_simulation_timestep(dt=self.dt)
        self.pr.start()
        self.pr.step()

        # ROBOT loading
        self.panda = Panda()
        self.planner = PyRepPlanner(self)
        self.panda_gripper = PandaGripper()
        if 'cameras' in self.SCENE_FILE:
            # Publisher for Intel Realsense D435 image
            self.leftimagerpub = self.create_publisher(Image, '/coppelia/left_camera', 5)
            self.rightimagerpub = self.create_publisher(Image, '/coppelia/right_camera', 5)
            self.LeftImager = VisionSensor("LeftImager")
            self.RightImager = VisionSensor("RightImager")
        self.panda_target = Dummy("Panda_target")
        self.focus_target = Dummy('Focus_target')
        self.focus_target_visual = Shape('Focus_target_visual')

        self.camera = Camera("DefaultCamera")
        self.panda_sensor = ProximitySensor('Panda_gripper_attachProxSensor')

        self.init_composite_objects()

        # For PyRep solver, this will publish joints solution to system
        self.ik_solution_pub = None
        if self.IK_SOLVER == 'relaxed_ik':
            # Receives IK solutions (computed in relaxed ik node)
            self.create_subscription(JointAngles, '/relaxed_ik/joint_angle_solutions', self.callback_output_nodeIK, 5)
        elif self.IK_SOLVER == 'pyrep':
            # Publishes IK solutions (computed here)
            self.ik_solution_pub = self.create_publisher(JointAngles, '/relaxed_ik/joint_angle_solutions', 5)
        elif self.IK_SOLVER == 'velocity_control':
            # Publishes IK solutions (computed here)
            self.create_subscription(JointAngles, '/velocity_control', self.callback_velocity_control, 5)
            self.ik_solution_pub = self.create_publisher(JointAngles, '/relaxed_ik/joint_angle_solutions', 5)
        elif self.IK_SOLVER == 'custom':
            # Received IK solutions (computed in specified topic)
            self.ik_solution_pub = self.create_publisher(JointAngles, self.IK_TOPIC, 5)
        else: raise Exception("[ERROR*] Wrong 'ik_solver' used in demo.launch!")
        # Config subscriber
        self.create_subscription(String, '/coppelia/config', self.callback_config, 5)
        # Receives the end-effector pose goal
        self.create_subscription(EEPoseGoals, '/ee_pose_goals', self.callback_goal_poses, 5)
        # Publish joint states and end-effector
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states_coppelia', 5)
        self.eef_pub = self.create_publisher(Pose, '/pose_eef', 5)

        self.at_target_pub = self.create_publisher(Bool, '/coppelia_sim/at_target', 5)
        self.camera_orientation_pub = self.create_publisher(Vector3, '/coppelia/camera_angle', 5)

        # Listen for service
        self.create_service(AddOrEditObject, 'add_or_edit_object', self.add_or_edit_object_callback)
        self.create_service(AddLine, 'add_line', self.add_line_callback)
        self.create_service(RemoveObject, 'remove_object', self.remove_object_callback)
        self.create_service(GripperControl, 'gripper_control', self.gripper_control_callback)

        pose = Pose()
        p = self.panda.get_tip().get_position()
        pose.position = Point(x=p[0],y=p[1],z=p[2])
        q = self.panda.get_tip().get_quaternion()
        pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        self.eef_pose = pose

        if 'cbgo' in self.SCENE_FILE:
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

        self.line_reqs = []
        self.line_objs = []

        # Hide sidebars
        sim.simSetBoolParameter(sim.sim_boolparam_browser_visible,False)
        sim.simSetBoolParameter(sim.sim_boolparam_hierarchy_visible,False)
        sim.simSetBoolParameter(sim.sim_boolparam_console_visible,False)


    @property
    def frequency(self):
        return 1/self.dt

    def __enter__(self):
        spinning_thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        spinning_thread.start()

        threadSimulation = Thread(target = self.simulate)
        threadSimulation.start()

        # Publishes informations about selected objects
        threadObjectPub = Thread(target = self.objects_publisher)
        threadObjectPub.start()
        # Publishes camera images
        if 'cameras' in self.SCENE_FILE:
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
                msg.angles.append(Float32(data=j))
            self.ik_solution_pub.publish(msg)

    def callback_goal_poses(self, data):
        ''' PyRep solver option. Receved goal poses will be applied InverseKinematics in this node
        '''
        self.eef_pose = data.ee_poses[0]

    def callback_velocity_control(self, data):
        ''' PyRep solver option. Receved goal poses will be applied InverseKinematics in this node
        '''
        self.qd = data

    def callback_config(self, data):
        if data.data == 'reset':
            self.panda.set_joint_positions([-2.23, 0.26, 2.44, -2.48, -0.20, 2.18, 1.13], disable_dynamics=True)
            print("Joints reset!")
            # Delete all objects
            with sem:
                for key in list(self.SceneObjects.keys()):
                    obj = self.SceneObjects.pop(key)
                    if not ('line' in key):
                        obj.sensor_obj.remove()
                    obj.obj.remove()
        else:
            print("Config was not recognized")

    def add_line_callback(self, msg, req):
        point_array = []
        for point in msg.points.points:
            point_array.append([point.x, point.y, point.z, 0., 0., 0.])

        sem.acquire()
        self.line_reqs.append([msg.name, point_array])
        sem.release()

        req.success = True
        return req

    def get_ycb_names(self):
        # load ycb names
        dirnames = None
        for (_, dirnames, _) in os.walk(f'{COPPELIA_OBJECTS_PATH}/ycb'):
            break
        if dirnames is None:
            print("No YCB objects found!")
            return []
        dirnames.sort()
        ycbnames = [dirname[4:] for dirname in dirnames]
        return ycbnames

    def add_or_edit_object_callback(self, msg, req):
        ''' Receives service callback of object creation
            - msg defined in AddOrEditObject.srv
        '''
        # Handle special cases (Camera)
        if msg.name in ['camera', 'Camera', 'Realsense', 'realsense']:
            with sem:
                if msg.pub_info=='true': self.camera_pub_bool=True
                elif msg.pub_info=='false': self.camera_pub_bool=False
            req.success = True
            return req
        if msg.name == 'Focus_target':
            with sem:
                self.focus_target_visual.set_position(extv(msg.pose.position))
                self.focus_target.set_position(extv(msg.pose.position))
            req.success = True
            return req
        ''' Check if we want to add_or_edit composite object '''
        if msg.name in self.COMPOSITE_OBJECTS:
            if msg.name not in self.co_names:
                raise Exception(f"Want to change composite object: {msg.name}, it is in list {self.COMPOSITE_OBJECTS}, but it is not in co_names: {self.co_names}!")
            with sem:
                self.edit_composite_object(msg)
                req.success = True
                return req
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
                if msg.init_size[0] == -1: msg.init_size = [1., 1., 1.]

                if msg.init_file == 'cup':
                    object = Shape.import_mesh(filename = getCoppeliaObjectsPath()+"/cup.ply", scaling_factor=msg.init_size[0])
                elif "_".join(msg.init_file.split(" ")) in self.get_ycb_names():
                    init_file = msg.init_file
                    init_file = init_file.split(" ")
                    init_file = "_".join(init_file)

                    ycb_names = self.get_ycb_names()
                    ycb_id = int(ycb_names.index(init_file)+1)

                    path = f"{COPPELIA_OBJECTS_PATH}/ycb/{ycb_id:03}_{init_file}/google_16k/textured.dae"
                    object = Shape.import_mesh(filename = path, scaling_factor=msg.init_size[0])
                else:
                    object = Shape.import_mesh(filename = msg.init_file, scaling_factor=msg.init_size[0])
            # 3. Create Shape
            elif msg.init_shape:
                if msg.init_size[0] == -1: msg.init_size = [0.03, 0.03, 0.03]
                object = Shape.create(type=init_shape, size=[msg.init_size[0], msg.init_size[1], msg.init_size[2]])
            # 4. Create Box
            else:
                if msg.init_size[0] == -1: msg.init_size = [0.03, 0.03, 0.03]
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
        if msg.pose and msg.pose.orientation.w != 1000:
            object.set_quaternion(extq(msg.pose.orientation))
        if msg.color: object.set_color(color_rgb)
        if msg.friction >= 0.: object.set_bullet_friction(msg.friction)
        if sum(msg.inertia) != 0:
            object.set_inertia(itertia, inertia_transformation)

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

        req.success = True
        return req

    def remove_object_callback(self, msg, req):
        ''' Receives service callback of object deletion
            - msg defined in Remove Object.srv
        '''

        if msg.name in self.SceneObjects.keys():
            with sem:
                obj = self.SceneObjects.pop(msg.name)
                if not ('line' in msg.name):
                    obj.sensor_obj.remove()
                obj.obj.remove()
            req.success = True
            return req
        elif msg.name in self.co_names:
            with sem:
                self.get_co_by_name(msg.name).move_away()
            req.success = True
            return req
        else:
            print("[WARN*][Coppelia Pub] No object with name ", msg.name, " found!")

            req.success = False
            return req

    def gripper_control_callback(self, msg, req):
        ''' Control the gripper with values
            - msg defined in GripperControl.srv
            - position 0.0 -> closed, 1.0 -> open
            - (pseudo) effort <0.0-1.0>
        ```
        panda.gripper.actuate(...)
        time.sleep(1)
        panda.gripper.actuate(...)
        ```
        '''
        print(f"[Gripper move] [Start] to pos: {msg.position}, msg.effort: {round(msg.effort,2)}, action: {msg.action}, obj: {msg.object}, eefrot: {msg.eef_rot}")
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
                if i > 50:
                    print("[gripper] return false - error")
                    req.success = False
                    return req
            print(f"[Gripper move] Grasped: {grasped}")

        elif msg.action == 'release':
            self.panda_gripper.release()

        if msg.position != -1:
            time.sleep(1)
            self.panda_gripper.actuate(msg.position, # open position
                                       msg.effort)   # (pseudo) effort
            '''
            i = 0
            while (self.panda_gripper.get_open_amount()[0] - msg.position) > 0.1 or (self.panda_gripper.get_open_amount()[1] - msg.position) > 0.1:
                i += 1
                time.sleep(1)
                print(f"[Gripper move] [WARNING] Not in final state! Sim: {self.panda_gripper.get_open_amount()}, Desired: {msg.position}")
                self.panda_gripper.actuate(msg.position, # open position
                                           msg.effort)   # (pseudo) effort
                if self.panda_sensor.read() != -1.0:
                    print(f"panda sensor values: {self.panda_sensor.read()} -> cant read sensor data (?)")
                    break
                if i > 50:
                    print(f"[Gripper move] [ERROR] Timeout")
                    req.success = False
                    return req
            '''
            print(f"[Gripper move] [Done] {self.panda_gripper.get_open_amount()}, {msg.position}\n")
        req.success = True
        return req

    def simulate(self):
        while type(self.eef_pose) == type(False) or type(self.eef_pose) == type(None):
            time.sleep(1)

        sim_rate = self.create_rate(self.frequency * self.rate_to_realtime)
        i =0
        while rclpy.ok(): # measured 2-5ms loop
            i+=1
            sem.acquire()
            if i%50==0:
                pass
            # viz the eef
            if self.eef_pose:
                self.panda_target.set_position([self.eef_pose.position.x, self.eef_pose.position.y, self.eef_pose.position.z])
            self.at_target = False
            # MODE 1
            if self.IK_SOLVER == 'relaxed_ik' or self.IK_SOLVER == 'custom':
                # Check if near the target
                if sameJoints(self.joints, self.panda.get_joint_positions()):
                    self.at_target = True
                else:
                    self.panda.set_joint_target_positions(self.joints)
            # MODE 2
            elif self.IK_SOLVER == 'pyrep':
                if samePoses(self.eef_pose, self.panda.get_tip().get_position(), accuracy=0.01):
                    ## Plan is finished -> do eef movement if available
                    joints_next = self.panda.get_joint_positions()
                    if self.eef_rot is not None: joints_next[-1] = np.clip(self.eef_rot, -2.7, 2.7)
                    self.set_joint_target_positions(joints_next)
                    self.joints = joints_next
                    self.at_target = True
                else:
                    self.sim_step_pyrep_options()
            elif self.IK_SOLVER == 'velocity_control':
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
        joints_next = np.array(self.panda.get_joint_positions())
        if self.PYREP_CONTROL_MODE == 'PATH':
            if self.planner.done:
                plan = self.planner.pathPlan(self.eef_pose)
                if plan is False:
                    print("[Coppelia] [PyRep controller] Path not valid -> iksolve")
                    joints_next = self.pyrep_ik_solver(self.eef_pose)
                    if self.eef_rot is not None and joints_next.any(): joints_next[-1] = np.clip(self.eef_rot, -2.7, 2.7)
                    self.set_joint_target_positions(joints_next)
            else:
                self.planner.pathStep()
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
        if self.IK_SOLVER == 'pyrep':
            self.publish_as_output_ik(self.joints) # publishes ik output to other nodes

        ## Publish at_target Bool
        self.at_target_pub.publish(Bool(data=self.at_target))


    def eef_pub_fun(self):
        ''' Publish eef pose
        '''
        eef_msg = Pose()
        p = self.panda.get_tip().get_position()
        eef_msg.position = Point(x=p[0],y=p[1],z=p[2])
        q = self.panda.get_tip().get_quaternion()
        eef_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.eef_pub.publish(eef_msg)

    def joint_states_pub_fun(self):
        ''' Publish joint states
        '''
        joint_state_msg = JointState()
        joint_state_msg.position = self.panda.get_joint_positions()
        joint_state_msg.velocity = self.panda.get_joint_velocities()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
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
                print("[Coppelia] No configuration found for goal pose: ", pose)
                joints = None
        return joints

    def camera_publisher(self):
        rate = self.create_rate(self.IMAGE_PUBLISH_FREQ)
        while rclpy.ok():
            while rclpy.ok() and self.camera_pub_bool:
                img = Image()
                img.data = (255*np.array(self.LeftImager.capture_rgb()).reshape(2764800)).astype(int).tolist()
                img.step = 1280 * 1 * 3
                img.height = 720
                img.width = 1280
                img.encoding='rgb8'
                #img.header.seq = self.left_seq = self.left_seq + 1
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = ""
                self.leftimagerpub.publish(img)

                img = Image()
                img.data = (255*np.array(self.RightImager.capture_rgb()).reshape(2764800)).astype(int).tolist()
                img.step = 1280 * 1 * 3
                img.height = 720
                img.width = 1280
                img.encoding='rgb8'
                #img.header.seq = self.right_seq = self.right_seq + 1
                img.header.stamp = self.get_clock().now().to_msg()
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
            p = objw.obj.get_position()
            msg.pose.position = Point(x=p[0], y=p[1], z=p[2])
            q = objw.obj.get_quaternion()
            msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            msg.bounding_box = objw.obj.get_bounding_box()
            msg.collision = objw.obj.check_collision()
            ''' TODO: It is possible to get more contact info
            print("contact force and info ", objw.obj.get_contact())
            msg.contact_force_info = objw.obj.get_contact()
            '''
            return msg

        rate = self.create_rate(self.OBJECT_PUBLISH_FREQ)
        while rclpy.ok():
            for objn in list(self.SceneObjects):
                if self.SceneObjects[objn].publishInfo:
                    msg = prepareObjectInfoMsg(self.SceneObjects[objn])
                    self.objectinfopub.publish(msg)

            o = self.camera.get_orientation()
            self.camera_orientation_pub.publish(Vector3(x=o[0], y=o[1], z=o[2]))
            rate.sleep()

    def init_composite_objects(self, CO_CLASSES = ['drawer', ]):
        '''
        self.COMPOSITE_OBJECTS - ['drawer,drawer1'] - objects on which I check
        if they are in the scene and potentionally initialize them
        CO_CLASSES - classes below of different composite_objects definitions
        self.co - composite objects saved here
        '''
        objs = self.COMPOSITE_OBJECTS
        objs = objs.split(",")

        self.co = []
        for obj in objs:
            if not self.check_if_composite_object_in_scene(obj):
                continue
            n_classes_obj_belong = 0
            for co_class in CO_CLASSES:
                # 'drawer' is included in 'drawer1' or 'drawerx'
                if co_class in obj:
                    n_classes_obj_belong+=1
            if n_classes_obj_belong < 1:
                continue
            elif n_classes_obj_belong > 1:
                raise Exception(f"More classes which obj: {obj} belong in, classes: {CO_CLASSES}")

            cls_ = None
            for cls in CO_CLASSES:
                if cls in obj:
                    cls_ = cls
            cls_ = cls
            cls_cap = cls_.capitalize()

            ''' o = self.Drawer(name) '''
            o = getattr(self,cls_cap)(obj)
            self.co.append(o)
            self.get_logger().info(f"inited_composite object {obj}")


    @property
    def co_names(self):
        return [o.name for o in self.co]
    def get_co_by_name(self, name):
        for o in self.co:
            if o.name == name:
                return o
        raise Exception(f"Cannot return composite_object with name: {name}, there are only {self.co_names}")

    def edit_composite_object(self,msg):
        o = self.get_co_by_name(msg.name)
        o.edit_object(msg)

    def check_if_composite_object_in_scene(self, name):
        ## TODO:
        return True

    ''' Classes of the Coposite Objects
    To add new class object:
    1. Add class name to: CO_CLASSES in init_composite_objects function parameter
    '''
    class Drawer():
        def __init__(self, name):
            self.name = name
            self.drawer = Shape(f'{name}')
            self.drawer_slider = Joint(f'{name}_joint_bottom')

            self.default_position = self.drawer.get_position()
        def edit_object(self,msg):
            self.drawer.set_position(extv(msg.pose.position), reset_dynamics=True)

            if msg.object_state == 'opened': self.drawer_slider.set_joint_position(position=0.2, disable_dynamics=True)
            if msg.object_state == 'semi-opened': self.drawer_slider.set_joint_position(position=0.1, disable_dynamics=True)
            if msg.object_state == 'slightly-opened': self.drawer_slider.set_joint_position(position=0.03, disable_dynamics=True)
            elif msg.object_state == 'closed': self.drawer_slider.set_joint_position(position=0.0, disable_dynamics=True)
        def move_away(self):
            self.drawer.set_position(self.default_position, reset_dynamics=True)



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
        new_pose.position = Point(x=new_poselist[0],y=new_poselist[1],z=new_poselist[2])
        new_pose.orientation = Quaternion(x=new_poselist[3], y=new_poselist[4], z=new_poselist[5], w=new_poselist[6])
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
        pose.position = Point(x=p_out[0], y=p_out[1], z=p_out[2])
        pose.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])
        print("p1 ", p1, " pout ", p_out ," p2 ", p2)
        return pose


rclpy.init()
# __enter__ and __exit__ functions to be called
with CoppeliaSim() as coppeliasim:
    rate = coppeliasim.create_rate(10) # Hz
    while rclpy.ok():
        with sem:
            if coppeliasim.line_reqs != []:
                line_req = coppeliasim.line_reqs.pop()

                new_line_name = line_req[0]
                old_line_name = line_req[0]+"_"
                if line_req[0] in coppeliasim.SceneObjects:
                    new_line_name = line_req[0]+"_"
                    old_line_name = line_req[0]

                coppeliasim.line_objs.append(CartesianPath.create(automatic_orientation=False))
                coppeliasim.line_objs[-1].insert_control_points(line_req[1])
                if len(coppeliasim.line_objs) > 1:
                    obj = coppeliasim.line_objs.pop(0)
                    obj.remove()
        rate.sleep()




TESTING = False
if TESTING:
    pr = PyRep()
    SCENE_FILE_PATH = COPPELIA_SCENES_PATH+'/'+self.SCENE_FILE
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
