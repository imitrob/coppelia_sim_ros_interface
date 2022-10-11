#!/usr/bin/env python3
''' IMPORTANT: This file should stay in <coppelia_sim_ros_interface(pkg)>/src in order to get full path
'''
import os
import numpy as np

from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int8, Float64MultiArray

from pyrep.const import PrimitiveShape, TextureMappingMode

def getCoppeliaScenePath():
    HOME = os.path.expanduser("~")
    # searches for the WS name + print it
    THIS_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
    THIS_FILE_TMP = os.path.abspath(os.path.join(THIS_FILE_PATH, '..', '..', '..'))
    WS_FOLDER = THIS_FILE_TMP.split('/')[-1]
    SCENES_PATH = HOME+"/"+WS_FOLDER+"/src/coppelia_sim_ros_interface/include/scenes/"
    return SCENES_PATH

def getCoppeliaTexturesPath():
    HOME = os.path.expanduser("~")
    # searches for the WS name + print it
    THIS_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
    THIS_FILE_TMP = os.path.abspath(os.path.join(THIS_FILE_PATH, '..', '..', '..'))
    WS_FOLDER = THIS_FILE_TMP.split('/')[-1]
    TEXTURES_PATH = HOME+"/"+WS_FOLDER+"/src/coppelia_sim_ros_interface/include/textures/"
    return TEXTURES_PATH

def getCoppeliaObjectsPath():
    HOME = os.path.expanduser("~")
    # searches for the WS name + print it
    THIS_FILE_PATH = os.path.dirname(os.path.realpath(__file__))
    THIS_FILE_TMP = os.path.abspath(os.path.join(THIS_FILE_PATH, '..', '..', '..'))
    WS_FOLDER = THIS_FILE_TMP.split('/')[-1]
    TEXTURES_PATH = HOME+"/"+WS_FOLDER+"/src/coppelia_sim_ros_interface/include/objects/"
    return TEXTURES_PATH

# Handy functions
def extq(q):
    ''' Extracts Quaternion object
    Parameters:
        q (Quaternion()): From geometry_msgs.msg
    Returns:
        x,y,z,w (Floats tuple[4]): Quaternion extracted
    '''
    if type(q) == type(Quaternion()):
        return q.x, q.y, q.z, q.w
    elif (type(q) == dict and 'w' in q.keys()):
        return q['x'], q['y'], q['z'], q['w']
    else: raise Exception("extq input arg q: Not Quaternion or dict with 'x'..'w' keys!")


def extv(v):
    ''' Extracts Point/Vector3 to Cartesian values
    Parameters:
        v (Point() or Vector3() or dict with 'x'..'z' in keys): From geometry_msgs.msg or dict
    Returns:
        [x,y,z] (Floats tuple[3]): Point/Vector3 extracted
    '''
    if type(v) == type(Point()) or type(v) == type(Vector3()):
        return v.x, v.y, v.z
    elif (type(v) == dict and 'x' in v.keys()):
        return v['x'], v['y'], v['z']
    else: raise Exception("extv input arg v: Not Point or Vector3 or dict! It is: ", type(v))


def quaternion_multiply(quaternion1, quaternion0):
    ''' Better to list that here, than from importing it externally via pkg tf
    Parameters:
        quaternion1, quaternion0 (Float[4]), Format (x,y,z,w)
    Outputs:
        quaternion output (Float[4]), Format (x,y,z,w)
    '''
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)

def samePoses(pose1, pose2, accuracy=0.05):
    ''' Checks if two type poses are near each other
        (Only for cartesian (xyz), not orientation wise)
    Parameters:
        pose1 (type Pose(), Point(), list or tuple)
        pose2 (type Pose(), Point(), list or tuple)
        accuracy (Float): threshold of return value
    Returns:
        same poses (Bool)
    '''
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

def sameJoints(joints1, joints2, accuracy=0.1):
    ''' Checks if two type joints are near each other
    Parameters:
        joints1 (type float[7])
        joints2 (type float[7])
        threshold (Float): sum of joint differences threshold
    '''
    assert isinstance(joints1[0],float) and len(joints1)==7, "Not datatype List w len 7, joints 1: "+str(joints1)
    assert isinstance(joints2[0],float) and len(joints2)==7, "Not datatype List w len 7, joints 2: "+str(joints2)

    if sum([abs(i[0]-i[1]) for i in zip(joints1, joints2)]) < accuracy:
        return True
    return False

def shapeObjToWrappingMode(shape):
    dict = {
        PrimitiveShape.CUBOID: TextureMappingMode.CUBE,
        PrimitiveShape.SPHERE: TextureMappingMode.SPHERE,
        PrimitiveShape.CYLINDER: TextureMappingMode.CYLINDER,
        PrimitiveShape.CONE: None,
        'plane': TextureMappingMode.PLANE
        }
    return dict[shape]
