import rclpy, time
from rclpy.node import Node
import numpy as np

from coppelia_sim_ros_interface.srv import AddOrEditObject, AddLine, RemoveObject, GripperControl
from coppelia_sim_ros_interface.msg import ObjectInfo
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Polygon, Point32, Pose, Point, Quaternion
from coppelia_sim_ros_interface.msg import EEPoseGoals, JointAngles

try:
    import os_and_utils.move_lib as ml
    import os_and_utils.scenes as sl
except ModuleNotFoundError:
    pass




def samePoses(pose1, pose2, accuracy=0.05):
    assert isinstance(pose1,(Pose,Point,np.ndarray,list,tuple)), "Not right datatype, pose1: "+str(pose1)
    assert isinstance(pose2,(Pose,Point,np.ndarray,list,tuple)), "Not right datatype, pose2: "+str(pose2)

    if isinstance(pose1,(list,tuple,np.ndarray)):
        pose1 = pose1[0:3]
    elif isinstance(pose1,Point):
        pose1 = [pose1.x, pose1.y, pose1.z]
    elif isinstance(pose1,Pose):
        pose1 = [pose1.position.x, pose1.position.y, pose1.position.z]
    if isinstance(pose2,(list,tuple,np.ndarray)):
        pose2 = pose2[0:3]
    elif isinstance(pose2,Point):
        pose2 = [pose2.x, pose2.y, pose2.z]
    elif isinstance(pose2,Pose):
        pose2 = [pose2.position.x, pose2.position.y, pose2.position.z]

    if np.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2 + (pose1[2] - pose2[2])**2) < accuracy:
        return True
    return False

class CoppeliaROSInterfaceNode(Node):
    def __init__(self):
        print("[CoppeliaROSInterface] Creating new ROS node")
        super().__init__("CoppeliaSimRosInterfaceClient")
        self.r = CoppeliaROSInterface(rosnode=self)

class CoppeliaROSInterface():
    def __init__(self, rosnode):
        '''
        Parameters:
            rosnode () <- ROS node handle
        '''
        print("[Coppelia Client] Waiting for Coppelia to be initialized")
        self.rosnode = rosnode
        # Listen and refresh endeffector poses as "eef_poses_now"
        self.eef_pose_now = []

        # Subscribe for endeffector poses and wait for first message
        rosnode.create_subscription(Pose, "/pose_eef", self.eef_pose_callback, 5)

        #self.ee_pose_goals_pub = rosnode.create_publisher(EEPoseGoals, "/ee_pose_goals", 5)

        self.config_pub = self.rosnode.create_publisher(String, "/coppelia/config", 5)

        ## Service clients
        self.gripper_control_client = rosnode.create_client(GripperControl, 'gripper_control')
        while not self.gripper_control_client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.gripper_control_req = GripperControl.Request()

        self.add_line_client = rosnode.create_client(AddLine, 'add_line')
        while not self.add_line_client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.add_line_req = AddLine.Request()

        self.add_or_edit_object_client = rosnode.create_client(AddOrEditObject, 'add_or_edit_object')
        while not self.add_or_edit_object_client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.add_or_edit_object_req = AddOrEditObject.Request()

        self.remove_object_client = rosnode.create_client(RemoveObject, 'remove_object')
        while not self.remove_object_client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        self.remove_object_req = RemoveObject.Request()

        print("[Coppelia Client] Coppelia initialized!")

    def ready(self):
        if self.eef_pose_now == []:
            return False
        else:
            return True

    def eef_pose_callback(self, data):
        self.eef_pose_now = data

    def execute_trajectory_with_waypoints(self, trajectory, waypoints):
        trajectory = np.array(trajectory)
        if trajectory.any():
            # TODO: Rotation
            pose = Pose()
            pose.orientation = Quaternion(x=0.0,y=1.0,z=0.0,w=0.0)

            lentrajectory = len(trajectory)
            for n,point in enumerate(trajectory):
                portion = n / lentrajectory # portion of trajectory executed
                wpkeys = list(waypoints.keys()) # check if new waypoint action will be executed
                if wpkeys and wpkeys[0] <= portion:
                    self.handle_waypoint_action(waypoints.pop(wpkeys[0]))
                # TEMP: Function before will check the trajectory
                if np.array(point).any() == None: continue
                if point[2] < 0.0:
                    point[2] = 0.
                    print('position z coordinate < 0: will be set to zero')
                pose.position = Point(x=point[0], y=point[1], z=point[2])
                self.go_to_pose(pose, blocking=False)
        else: # not trajectory
            wpkeys = list(waypoints.keys())
            for wpkey in waypoints:
                self.handle_waypoint_action(waypoints[wpkey])


    def handle_waypoint_action(self, sem=None, waypoint=None):
        ''' DEPRECATED (?)
        '''
        assert waypoint is not None, "Waypoint not given"
        action = ''
        if waypoint.gripper is not None: # handle gripper action
            if waypoint.gripper < 0.1 and not ml.md.attached: action = 'grasp'; ml.md.attached = True
            if waypoint.gripper > 0.9 and ml.md.attached: action = 'release'; ml.md.attached = False
            if action: print(f"--- Action {action} {waypoint.gripper}")
            if sl:
                self.set_gripper(sem=sem, position=waypoint.gripper, action=action, object=sl.scene.object_names[ml.md.object_focus_id])


    def go_to_pose(self, pose=None, blocking=False):
        ''' Execute movement to desired pose
        '''
        if isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 3:
            pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]), orientation=Quaternion(x=0.,y=1.,z=0.,w=0.))
        elif isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 7:
            pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]), orientation=Quaternion(x=pose[3],y=pose[4],z=pose[5],w=pose[6]))

        eeposegoals = EEPoseGoals()
        eeposegoals.ee_poses = [pose]
        if blocking:
            while (not samePoses(pose, self.eef_pose_now)):
                time.sleep(5)
                self.ee_pose_goals_pub.publish(eeposegoals)
                print(f"[Coppelia Client] Not in target! {eeposegoals.ee_poses[0].position}, eef pose now: {self.eef_pose_now.position}")
        else:
            self.ee_pose_goals_pub.publish(eeposegoals)

    def move_above_axis(self, index, delta):
        while not self.ready():
            time.sleep(2)
            print("Waiting for /pose_eef message")
        coords = ['x','y','z']

        new_val = getattr(self.eef_pose_now.position, coords[index]) + delta
        setattr(self.eef_pose_now.position, coords[index], new_val)

        self.go_to_pose(self.eef_pose_now)

    def reset(self):
        self.config_pub.publish(String(data="reset"))

    def gripper_control(self, sem=None, position=None, effort=0.4, eef_rot=-1, action="", object=""):
        assert position is not None, "Position not given!"
        self.set_gripper(sem=sem, position=position, effort=effort, eef_rot=eef_rot, action=action, object=object)

    def open_gripper(self, sem=None):
        self.set_gripper(sem=sem, position=1.0)

    def close_gripper(self, sem=None):
        self.set_gripper(sem=sem, position=0.0)

    def pick_object(self, sem=None, object=None):
        assert object is not None, "Object not given!"
        self.set_gripper(sem=sem, position=0.0, action='grasp', object=object)

    def release_object(self, sem=None):
        self.set_gripper(sem=sem, position=1.0, action='release')

    def toggle_object(self, attached):
        if attached:
            simhandle.release_object()
        else:
            simhandle.pick_object(sl.paths[pp].actions[self.currentPose])

    def set_gripper(self, sem=None, position=-1, effort=0.4, eef_rot=-1, action="", object=""):
        '''
        Parameters:
            position (Float): 0. -> gripper closed, 1. -> gripper opened
            effort (Float): Range <0.0, 1.0>
            action (Str): 'grasp' attach object specified as 'object', 'release' will release previously attached object, '' (no attach/detach anything)
            object (Str): Name of object specified to attach
        Returns:
            success (Bool)
        '''
        self.gripper_control_req.position = float(np.float32(position))
        self.gripper_control_req.effort = float(np.float32(effort))
        self.gripper_control_req.eef_rot = float(np.float32(eef_rot))
        self.gripper_control_req.action = action
        self.gripper_control_req.object = object
        try:
            self.set_gripper_future = self.gripper_control_client.call_async(self.gripper_control_req)
            self.set_gripper_future = self.gripper_control_client.call_async(self.gripper_control_req)
            self.spin_until_future_complete_(sem, self.set_gripper_future)
            return self.set_gripper_future.result()
        except Exception as e:
            print("Service call failed: %s"%e)
            return False

    def add_line(self, sem=None, name='', points=[]):
        '''
        '''
        assert points!=[], "No points when calling add_line!"
        plgn = Polygon()
        for point in points:
            plgn.points.append(Point32(x=point[0], y=point[1], z=point[2]))
        self.add_line_req.name = name
        self.add_line_req.points = plgn
        try:
            self.add_line_future = self.add_line_client.call_async(self.add_line_req)
            self.spin_until_future_complete_(sem, self.add_line_future)
            return self.add_line_future.result()
        except Exception as e:
            print("Service call failed: %s"%e)
            return False

    def add_or_edit_object(self, sem=None, file='', name='', pose=Pose(), shape="", size=-1, collision='', color='', friction=-1, frame_id='', mass=-1, inertia=np.zeros(9), inertia_transformation=np.zeros(12), dynamic='', pub_info='', texture_file="", object_state="", timeout=10):
        ''' Adds shape/mesh based on configuration
            - If no shape & no mesh specified -> box created
            - If both shape & mesh specified -> mesh is used

            - Use name='camera' to enable/disable cameras publishing
        Parameters:
            file (Str): Mesh file string (def. in dir include/models/) as .obj file
            name (Str): Name of object
            pose (Pose()): Pose of object to be spawned
            shape (Str): 'cube', 'sphere', 'cylinder', 'cone'
            collision (Str): 'true', 'false' - Enable/Disable collisions, '' - keep unchanged
            color (Str): 'r', 'g', 'b', 'c', 'm', 'y', 'k', '' - keep unchanged
            frame_id (Str): By respect to what frame_id to be spawned
            friction (Float): Coefficient [-], use value -1 to keep unchanged
            mass (Float): [Kg], use value -1 to keep unchanged
            inertia (Matrix, 3x3): use zeros to keep unchanged
            inertia_transformation (Matrix, 4x3): use zeros to keep unchanged
            dynamic (Str): 'true', 'false' - Enable/Disable dynamics, '' - keep unchanged
            pub_info (Str): 'true', 'false', '' - keep unchanged, if enabled, publishes info about object to topic (/object_info)
            texture_file (Str): Loads texture_file from '/include/textures/' folder, '' - keep unchanged
            timeout (Float): Raise exception after [sec]

        Returns:
            success (Bool)
        '''
        if isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 3:
            pose = list(np.array(pose, dtype=float))
            pose = Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation=Quaternion(x=0.,y=0.,z=0.,w=1.))
        elif isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 7:
            pose = list(np.array(pose, dtype=float))
            pose = Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation=Quaternion(x=pose[3],y=pose[4],z=pose[5],w=pose[6]))

        if isinstance(size, (float,int)):
            size = [size,size,size]
        self.add_or_edit_object_req.name = name
        self.add_or_edit_object_req.init_file = file
        self.add_or_edit_object_req.init_shape = shape
        self.add_or_edit_object_req.init_size = np.array(size, dtype='float32')
        if isinstance(collision, bool): collision = 'true' if collision == True else 'false'
        self.add_or_edit_object_req.init_collision = collision
        self.add_or_edit_object_req.pose = pose
        if color not in ['r', 'g', 'b', 'c', 'm', 'y', 'k', '']:
            print("[Add or Edit object ERROR] Color is not in ['r', 'g', 'b', 'c', 'm', 'y', 'k', '']")
            return False
        self.add_or_edit_object_req.color = color
        self.add_or_edit_object_req.friction = float(friction)
        self.add_or_edit_object_req.frame_id = frame_id
        self.add_or_edit_object_req.mass = float(np.float32(mass))
        self.add_or_edit_object_req.inertia = np.array(inertia, dtype='float32')
        self.add_or_edit_object_req.inertia_transformation = np.array(inertia_transformation, dtype='float32')
        if isinstance(dynamic, bool): dynamic = 'true' if dynamic == True else 'false'
        self.add_or_edit_object_req.dynamic = dynamic
        self.add_or_edit_object_req.pub_info = pub_info
        self.add_or_edit_object_req.texture_file = texture_file
        self.add_or_edit_object_req.object_state = object_state
        try:
            self.add_or_edit_object_future = self.add_or_edit_object_client.call_async(self.add_or_edit_object_req)
            self.spin_until_future_complete_(sem, self.add_or_edit_object_future)
            return self.add_or_edit_object_future.result()
        except Exception as e:
            print("Service call failed: %s"%e)
            return False

    def remove_object(self, sem=None, name=None):
        ''' Removing Objects from the Scene
        Parameters:
            name (Str): Name of object to remove
        Returns:
            success (Bool)
        '''
        self.remove_object_req.name = name
        try:
            self.remove_object_future = self.remove_object_client.call_async(self.remove_object_req)
            self.spin_until_future_complete_(sem, self.remove_object_future)
            return self.remove_object_future.result()
        except Exception as e:
            print("Service call failed: %s"%e)
            return False

    def spin_until_future_complete_(self, sem, future):
        if sem is not None:
            while rclpy.spin_until_future_complete(self.rosnode, future, timeout_sec=0.01) is not None:
                sem.release()
                time.sleep(0.01)
                sem.acquire()
        else:
            print("Semaphore not added, possible deadlocks")
            rclpy.spin_until_future_complete(self.rosnode, future, timeout_sec=0.01)

if __name__ == '__main__':
    rclpy.init()
    cop = CoppeliaROSInterfaceNode()

    input("Gripper close test!")
    cop.r.close_gripper()
    input("Gripper open test!")
    cop.r.open_gripper()
    input("Add object test!")
    cop.r.add_or_edit_object(name='box1', pose=[0.5,0.2,0.1])
    input("Add object test!")
    cop.r.add_or_edit_object(name='box2', shape='cone', color='r', pose=[0.5,0.0,0.1])
    input("Edit object test!")
    cop.r.add_or_edit_object(name='box1', color='b', pose=[0.5,0.2,0.1])
    input("Remove object test!")
    cop.r.remove_object(name='box1')
    input("Go to pose test!")
    cop.r.go_to_pose(pose=[0.5,0.0,0.2])
    input("Go to pose 2 test!")
    cop.r.go_to_pose(pose=[0.5,0.0,0.1])

    print("Done")


class CoppeliaROSInterfaceWithSem():
    '''
    '''
    def __init__(self, sem, *args, **kwargs):
        self.r = CoppeliaROSInterface(*args, **kwargs)
        self.sem = sem

    def ready(self, *args, **kwargs):
        with self.sem:
            return self.r.ready(*args, **kwargs)

    def eef_pose_callback(self, *args, **kwargs):
        with self.sem:
            return self.r.eef_pose_callback(*args, **kwargs)

    def execute_trajectory_with_waypoints(self, *args, **kwargs):
        with self.sem:
            return self.r.execute_trajectory_with_waypoints(*args, **kwargs)

    def handle_waypoint_action(self, *args, **kwargs):
        with self.sem:
            return self.r.handle_waypoint_action(sem=self.sem, *args, **kwargs)

    def go_to_pose(self, *args, **kwargs):
        with self.sem:
            return self.r.go_to_pose(*args, **kwargs)

    def move_above_axis(self, *args, **kwargs):
        with self.sem:
            return self.r.move_above_axis(*args, **kwargs)

    def reset(self, *args, **kwargs):
        with self.sem:
            return self.r.reset(*args, **kwargs)

    def gripper_control(self, *args, **kwargs):
        with self.sem:
            return self.r.open_gripper(sem=self.sem, *args, **kwargs)

    def open_gripper(self, *args, **kwargs):
        with self.sem:
            return self.r.open_gripper(sem=self.sem, *args, **kwargs)

    def close_gripper(self, *args, **kwargs):
        with self.sem:
            return self.r.close_gripper(sem=self.sem, *args, **kwargs)

    def pick_object(self, *args, **kwargs):
        with self.sem:
            return self.r.pick_object(sem=self.sem, *args, **kwargs)

    def release_object(self, *args, **kwargs):
        with self.sem:
            return self.r.release_object(sem=self.sem, *args, **kwargs)

    def toggle_object(self, *args, **kwargs):
        with self.sem:
            return self.r.toggle_object(*args, **kwargs)

    def set_gripper(self, *args, **kwargs):
        with self.sem:
            return self.r.set_gripper(sem=self.sem, *args, **kwargs) # 1

    def add_line(self, *args, **kwargs):
        with self.sem:
            return self.r.add_line(sem=self.sem, *args, **kwargs) # 2

    def add_or_edit_object(self, *args, **kwargs):
        with self.sem:
            return self.r.add_or_edit_object(sem=self.sem, *args, **kwargs) # 3

    def remove_object(self, *args, **kwargs):
        with self.sem:
            return self.r.remove_object(sem=self.sem, *args, **kwargs) # 4
