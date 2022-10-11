#!/usr/bin/env python3
''' Example showcase of PyRep and Panda Robot without ROS
'''
import numpy as np
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape, TextureMappingMode
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.errors import ConfigurationPathError

import os
# Load Scenes/Textures directory
SCENES='/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/include/scenes/')
TEXTURES='/'.join(os.path.realpath(__file__).split('/')[:-2])+str('/include/textures/')

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch(SCENES+'scene_panda_franka_gripper.ttt', headless=False, responsive_ui=True)
pr.start()  # Start the simulation
pr.step()

panda = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

velocities = [.1, .2, .3, .4, .5, .6, .7]
panda.set_joint_target_velocities(velocities)
panda.set_joint_positions([0.,0.,0.,-np.pi/2,0.,np.pi/3,np.pi])


panda.set_joint_target_positions([0.,0.,0.,0.,0.,0.,0.])
for i in range(200):

    pr.step() # simulate behaviour

panda.set_joint_positions([0.,0.,0.,0.,0.,0.,0.])

for i in range(200):
    pr.step() # simulate behaviour

panda.get_joint_count()
panda.get_joint_forces()
panda.get_joint_modes()
[panda.joints[j].is_motor_enabled() for j in range(7)]
panda.joints[0].set_joint_target_position(1.)
panda.set_joint_target_velocities([0.2,0.02,0.02,0.02,0.02,0.02,0.02])
panda.set_joint_target_positions([0.,-np.pi/4,0.,-np.pi/2,0.,np.pi/3,np.pi])



for i in range(200):
    pr.step() # simulate behaviour


input("Example started! Press any button to proceed")
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

done = False
# Open the gripper halfway at a velocity of 0.04.
while not done:
    done = gripper.actuate(0.0, velocity=0.04)
    pr.step()


input(">> Control Panda via IK")
DELTA = 0.01
starting_joint_positions = panda.get_joint_positions()
pos, quat = panda.get_tip().get_position(), panda.get_tip().get_quaternion()

def move(index, delta):
    pos[index] += delta
    new_joint_angles = panda.solve_ik_via_jacobian(pos, quaternion=quat)
    panda.set_joint_target_positions(new_joint_angles)
    pr.step()

[move(2, DELTA) for _ in range(10)]
[move(2, -DELTA) for _ in range(10)]
[move(1, -DELTA) for _ in range(10)]
[move(2, DELTA) for _ in range(10)]
[move(1, DELTA) for _ in range(10)]

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
shape1,texture1 = pr.create_texture(filename=TEXTURES+'wood.jpg')
object.set_texture(texture=texture1, mapping_mode=TextureMappingMode.CUBE, repeat_along_u=True, repeat_along_v=True)

for i in range(20):
    pr.step() # simulate behaviour

input(">> Exit tutorial")

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
