#!/usr/bin/env python
''' Example showcase of PyRep and Panda Robot without ROS
Updated & tested for Python3.9
'''
import numpy as np
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape, TextureMappingMode
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.errors import ConfigurationPathError

import os, time
# Load Scenes/Textures directory
SCENES='/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/include/scenes/')
TEXTURES='/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/include/textures/')

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch(SCENES+'scene_panda_gripper_cbgo.ttt', headless=False, responsive_ui=True)
''' For ipykernel notebook, when __file__ is None
>>> pr.launch('/home/<user>/<your ws>/src/coppelia_sim_ros_interface/include/scenes/scene_panda_gripper_cbgo.ttt', headless=False, responsive_ui=True)
'''
pr.start()  # Start the simulation
pr.step()

panda = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

input("Example started! Press any button to proceed")
input(">> Target Robot Joints control!")

panda.set_joint_positions([0.,0.,0.,-np.pi/2,0.,np.pi/3,np.pi])
for i in range(40):
    pr.step() # simulate behaviour

print("Info:")
print(f"Number of joints: {panda.get_joint_count()}")
print(f"Joint forces: {panda.get_joint_forces()}")
print(f"Joint modes: {panda.get_joint_modes()}")
print(f"Is Panda in collision: {panda.check_arm_collision()}")
print(f"Joints enabled: {[panda.joints[j].is_motor_enabled() for j in range(7)]}")

input(">> Velocity control!")

panda.set_control_loop_enabled(False)
panda.set_joint_target_velocities([2,0.,0.,0.,0.,0.,0.])
for i in range(40):
    pr.step() # simulate behaviour
panda.set_joint_target_velocities([-2,0.,0.,0.,0.,0.,0.])
for i in range(40):
    pr.step() # simulate behaviour
panda.set_control_loop_enabled(True)

input(">> Add some objects to scene")
colors = [[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]
positions = [[0.3, 0.3, 0.1], [-0.5, -0.3, 0.1], [0.2, -0.3, 0.1]]
shapes = [PrimitiveShape.CYLINDER, PrimitiveShape.CONE, PrimitiveShape.SPHERE]
for i in range(0,3):
    object = Shape.create(type=shapes[i],
                          color=colors[i], size=[0.075, 0.075, 0.075],
                          position=positions[i])

for i in range(20):
    pr.step() # simulate behaviour

input(">> Move the gripper")
def move_gripper(pos):
    done = False
    while not done:
        gripper.actuate(pos, velocity=0.4)
        for i in range(10):
            pr.step() # simulate behaviour
        done = gripper.actuate(pos, velocity=0.4)
        for i in range(10):
            pr.step() # simulate behaviour
        print(f"done?: {done}")
print("Opening the gripper")
move_gripper(pos=1.0)
print("Closing the gripper")
move_gripper(pos=0.0)

input(">> Control Panda via IK")
DELTA = 0.01
starting_joint_positions = panda.get_joint_positions()
pos, quat = panda.get_tip().get_position(), panda.get_tip().get_quaternion()

def move(index, delta):
    pos[index] += delta
    new_joint_angles = panda.solve_ik_via_jacobian(pos, quaternion=quat)
    panda.set_joint_target_positions(new_joint_angles)
    pr.step()



panda.get_joint_positions()
panda.set_joint_target_positions([-2.23, 0.26, 2.44, -2.48, -0.20, 2.18, 1.13])
panda.set_joint_target_positions([0.,0.,0.,-np.pi/2,0.,np.pi/3,np.pi])

[move(0, -DELTA) for _ in range(10)]
try:
    [move(2, DELTA) for _ in range(10)]
    [move(2, -DELTA) for _ in range(10)]
    [move(1, -DELTA) for _ in range(10)]
    [move(2, DELTA) for _ in range(10)]
    [move(1, DELTA) for _ in range(10)]
except:
    print("[Error] Solving jacobian failed")

input(">> Reach the targets with panda")
# We could have made this target in the scene, but lets create one dynamically
target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)

position_min, position_max = [0.6, -0.2, 0.0], [0.8, 0.2, 0.4]

starting_joint_positions = panda.get_joint_positions()

LOOPS = 10
panda.set_joint_positions(starting_joint_positions)
for i in range(LOOPS):

    # Reset the arm at the start of each 'episode'
    #panda.set_joint_positions(starting_joint_positions)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    target.set_position(pos)

    # Get a path to the target (rotate so z points down)
    try:
        path = panda.get_path(
            position=pos, euler=[0, np.radians(180), 0])
    except ConfigurationPathError as e:
        print('Could not find path')
        print("For pos:", pos)
        continue

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()
    print('Reached target %d!' % i)

input(">> Add object and set the texture")

object = Shape.create(type=PrimitiveShape.CUBOID, size=[0.5, 0.5, 0.5], position=[1.0,0.0,0.5], orientation=[0.,0.,0.])
''' For ipykernel notebook, when __file__ is None
>>> shape1,texture1 = pr.create_texture(filename='/home/<user>/<your ws>/src/coppelia_sim_ros_interface/include/textures/wood.jpg')
'''
shape1,texture1 = pr.create_texture(filename=TEXTURES+'wood.jpg')
object.set_texture(texture=texture1, mapping_mode=TextureMappingMode.CUBE, repeat_along_u=True, repeat_along_v=True)

for i in range(20):
    pr.step() # simulate behaviour

input(">> Exit tutorial")

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
