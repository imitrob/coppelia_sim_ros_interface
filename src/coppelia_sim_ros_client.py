import rospy
import numpy as np

from coppelia_sim_ros_interface.srv import AddOrEditObject, AddOrEditObjectResponse, AddLine, AddLineResponse, RemoveObject, RemoveObjectResponse, GripperControl, GripperControlResponse
from coppelia_sim_ros_interface.msg import ObjectInfo
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Polygon, Point32, Pose, Point, Quaternion

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


class CoppeliaROSInterface():
    def __init__(self):
        # Listen and refresh endeffector poses as "eef_poses_now"
        self.eef_pose_now = []

        # Subscribe for endeffector poses and wait for first message
        rospy.Subscriber("/pose_eef", Pose, self.eef_pose_callback)

        #rospy.wait_for_message("/pose_eef", Pose, timeout=5)
        ''' improvement
        try:
            rospy.wait_for_message("/pose_eef", Pose, timeout=5)
        except rospy.ROSException:
            self.eef_pose_now.append(Pose(Point(0.3,0.0,0.3), Quaternion(0.707, 0.0, 0.707, 0.0)))
        # Publisher for target endeffector pose
        '''
        self.ee_pose_goals_pub = rospy.Publisher("/ee_pose_goals", Pose, queue_size=5)

        self.config_pub = rospy.Publisher("/coppelia/config/", String, queue_size=5)

        self.add_or_edit_object(name="Focus_target", pose=Pose(Point(0.0,0.0,0.0),Quaternion(0.0,0.0,0.0,1.0)))


    def eef_pose_callback(self, data):
        self.eef_pose_now = data

    def execute_trajectory_with_waypoints(self, trajectory, waypoints):
        trajectory = np.array(trajectory)
        if trajectory.any():
            # TODO: Rotation
            pose = Pose()
            pose.orientation.x = 0.7071067811865476
            pose.orientation.y = 0.7071067811865476

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
                pose.position = Point(*point)
                self.go_to_pose(pose, blocking=False)
        else: # not trajectory
            wpkeys = list(waypoints.keys())
            for wpkey in waypoints:
                self.handle_waypoint_action(waypoints[wpkey])


    def handle_waypoint_action(self, waypoint):
        action = ''
        if waypoint.gripper is not None: # handle gripper action
            if waypoint.gripper < 0.1 and not ml.md.attached: action = 'grasp'; ml.md.attached = True
            if waypoint.gripper > 0.9 and ml.md.attached: action = 'release'; ml.md.attached = False
            if action: print(f"--- Action {action} {waypoint.gripper}")
            if sl:
                CoppeliaROSInterface.set_gripper(waypoint.gripper, action=action, object=sl.scene.object_names[ml.md.object_focus_id])


    def go_to_pose(self, pose, blocking=False):
        ''' Execute movement to desired pose
        '''
        if isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 3:
            pose = Pose(Point(*pose), Quaternion(0.,1.,0.,0.))
        elif isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 7:
            pose = Pose(Point(*pose[:3]), Quaternion(*pose[3:]))

        if blocking:
            rate = rospy.Rate(10)
            while (not samePoses(pose, self.eef_pose_now)):
                self.ee_pose_goals_pub.publish(pose)
                rate.sleep()
                print("pose", pose, "  eef pose now ", self.eef_pose_now)
        else:
            self.ee_pose_goals_pub.publish(pose)

    def reset(self):
        msg = String("reset")
        self.config_pub.publish(msg)

    @staticmethod
    def gripper_control(position, effort=0.04, eef_rot=-1, action="", object=""):
        CoppeliaROSInterface.set_gripper(position, effort=effort, eef_rot=eef_rot, action=action, object=object)

    @staticmethod
    def open_gripper():
        CoppeliaROSInterface.set_gripper(position=1.0)

    @staticmethod
    def close_gripper():
        CoppeliaROSInterface.set_gripper(position=0.0)

    @staticmethod
    def pick_object(object):
        CoppeliaROSInterface.set_gripper(position=0.0, action='grasp', object=object)

    @staticmethod
    def release_object():
        CoppeliaROSInterface.set_gripper(position=1.0, action='release')

    @staticmethod
    def toggle_object(attached):
        if self.attached:
            simhandle.release_object()
        else:
            simhandle.pick_object(sl.paths[pp].actions[self.currentPose])


    @staticmethod
    def set_gripper(position=-1, effort=0.04, eef_rot=-1, action="", object=""):
        '''
        Parameters:
            position (Float): 0. -> gripper closed, 1. -> gripper opened
            effort (Float): Range <0.0, 1.0>
            action (Str): 'grasp' attach object specified as 'object', 'release' will release previously attached object, '' (no attach/detach anything)
            object (Str): Name of object specified to attach
        Returns:
            success (Bool)
        '''
        rospy.wait_for_service('gripper_control')
        try:
            gripper_control = rospy.ServiceProxy('gripper_control', GripperControl)
            response = gripper_control(position, effort, action, object, eef_rot)
            response = gripper_control(position, effort, action, object, eef_rot)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    @staticmethod
    def add_line(name='', points=[]):
        assert points!=[], "No points when calling add_line!"
        plgn = Polygon()
        for point in points:
            plgn.points.append(Point32(point[0], point[1], point[2]))
        rospy.wait_for_service('add_line')
        try:
            add_line = rospy.ServiceProxy('add_line', AddLine)
            response = add_line(name=name, points=plgn)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    @staticmethod
    def add_or_edit_object(file='', name='', pose=Pose(), shape="", size=None, collision='', color='', friction=-1, frame_id='', mass=-1, inertia=np.zeros(9), inertiaTransformation=np.zeros(12), dynamic='', pub_info='', texture_file="", object_state="", timeout=10):
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
            inertiaTransformation (Matrix, 4x3): use zeros to keep unchanged
            dynamic (Str): 'true', 'false' - Enable/Disable dynamics, '' - keep unchanged
            pub_info (Str): 'true', 'false', '' - keep unchanged, if enabled, publishes info about object to topic (/object_info)
            texture_file (Str): Loads texture_file from '/include/textures/' folder, '' - keep unchanged
            timeout (Float): Raise exception after [sec]

        Returns:
            success (Bool)
        '''
        if isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 3:
            pose = Pose(Point(*pose), Quaternion(0.,0.,0.,1.))
        elif isinstance(pose, (list,tuple,np.ndarray)) and len(pose) == 7:
            pose = Pose(Point(*pose[:3]), Quaternion(*pose[3:]))

        if isinstance(size, float):
            size = [size,size,size]

        try:
            rospy.wait_for_service('add_or_edit_object', timeout)
        except rospy.ROSException:
            print("Calling Service: Timeout, CoppeliaSim lagging (probably many cameras in view)")
            return False
        try:
            add_or_edit_object = rospy.ServiceProxy('add_or_edit_object', AddOrEditObject)
            response = add_or_edit_object(name=name, init_file=file, init_shape=shape, init_size=size, init_collision=collision, pose=pose, color=color, friction=friction, frame_id=frame_id, mass=mass, inertia=inertia, inertiaTransformation=inertiaTransformation, dynamic=dynamic, pub_info=pub_info, texture_file=texture_file, object_state=object_state)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    @staticmethod
    def remove_object(name=None):
        ''' Removing Objects from the Scene
        Parameters:
            name (Str): Name of object to remove
        Returns:
            success (Bool)
        '''
        rospy.wait_for_service('remove_object')
        try:
            remove_object = rospy.ServiceProxy('remove_object', RemoveObject)
            response = remove_object(name)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
