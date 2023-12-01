# Coppelia Sim ROS Interface

Package build on top of _CoppeliaSim_ simulator using mostly [PyRep](https://github.com/stepjam/PyRep) package to communicate with the simulator. Enables control of a simulator via ROS2 messages and services.

Package provides possibility to manage scene objects and robot control.

Tested version: Ubuntu 20.04, ROS2 Galactic, CoppeliaSim 4.1.0 Edu.

##### Diagram of the nodes in package:

```mermaid
graph TD;

subgraph "Scene control (ROS services)"
S1
S2
S3
end

subgraph "Scene info (ROS messages)"
T1
T2
end

subgraph "Robot control (ROS messages) (2 modes)"
T3
T4
end

S1("/add_or_edit_object") --> I
S2("/remove_object") --> I
S3("/gripper_control") --> I

I --> T1("/coppelia/object_info")
I --> T2("/coppelia/left_camera <br> /coppelia/right_camera")

T3("Joint states control <br> /relaxed_ik/joint_angle_solutions <br> OR <br> CUSTOM_TOPIC") --> I
T4("Pose control <br> /ee_pose_goals") --> I

I("mirracle sim ROS interface <br> Python3.X") --> P("PyRep <br> Python3.X")
P --> C("Coppelia Sim (Main)")
C --> P
P --> I

CSE("Coppelia Sim Example * <br> (examples/coppelia_sim_example.py) <br> * launched without ROS interface") --> P
P --> CSE
CSRE("Coppelia Sim ROS Example <br> (examples/coppelia_sim_ros_example.py)") --> I
I --> CSRE
```

## Installation

Install alongside [teleop_gesture_toolbox](https://github.com/imitrob/teleop_gesture_toolbox).

## Usage:


1. Check if _CoppeliaSim_ running correctly without ROS2, run given example:

```
cd <coppelia_sim_ros_interface>/examples
python coppelia_sim_example.py
```

2. Run _CoppeliaSim_ example with ROS.

  1. Terminal: Server
```
cd <coppelia_sim_ros_interface>/<coppelia_sim_ros_interface>
python coppelia_sim_ros_server.py
```

  2. Terminal: Client
```
cd <coppelia_sim_ros_interface>/examples
python coppelia_sim_ros_example.py
```


Example Python function that can be used to communicate with interface are in [lib file](src/coppelia_sim_ros_client.py).

## Interface via ROS:



### Camera images

##### Message

Subscribe on `/coppelia/left_camera` and `/coppelia/right_camera` to receive Camera Images type `Image`.

To view camera image, `rqt_image_view` can be used with command:
`ros2 run rqt_image_view rqt_image_view /coppelia/left_camera`

Call service `/add_or_edit_object` with argument `name='camera'` and argument `pub_info='true'`. (Template function in [lib](src/coppelia_sim_ros_client.py))

### Object info

##### Message

Subscribe on `/coppelia/object_info` to receive info about objects.

Call service `/add_or_edit_object` with argument `name='<object_name>'` and argument `pub_info='true'` to receive info about object. (Template function in [lib](src/coppelia_sim_ros_client.py))

### Add or Edit object

##### Service

Creates or edit object on scene.
If name given does not exist, the object will be created.
Service name: `/add_or_edit_object`
Service type: **AddOrEditObject** from coppelia_sim_ros_interface.srv
Service description: [file link](srv/AddOrEditObject.srv)
For calling, template function in [lib](src/coppelia_sim_ros_client.py)

Creation priority is:
 1. mesh file
 2. shape
 3. if both mesh & shape not given, a box is created


### Remove object

##### Service

Removes an object from the scene.

Service name: `/remove_object`
Service msg type: **RemoveObject** from coppelia_sim_ros_interface.srv
Service description: [file link](srv/RemoveObject.srv)
For calling, template function in [lib](src/coppelia_sim_ros_client.py)


### Gripper control

##### Service

Service name: `/gripper_control`
Service msg type: **GripperControl** from coppelia_sim_ros_interface.srv
Service description: [file link](srv/GripperControl.srv)
For calling, template function in [lib](src/coppelia_sim_ros_client.py)

Use parameter `position` as `0.0` for closed gripper and `1.0`  for opened gripper. Use effort between `0.0-1.0`.
Use action `grasp` or `release` to attach object to eef. Specify object as `object` parameter.
