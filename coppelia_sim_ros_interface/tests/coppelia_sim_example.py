import os
import numpy as np
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape, TextureMappingMode, JointMode
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.errors import ConfigurationPathError

pr = PyRep()
# Launch the application with a scene file in headless mode
pr.launch(f'{os.getcwd()}/../../include/scenes/scene_panda_gripper_cbgo.ttt', headless=False, responsive_ui=True)
pr.start()  # Start the simulation
pr.step()

panda = Panda()  # Get the panda from the scene
gripper = PandaGripper()  # Get the panda gripper from the scene

input("Test started, Continue with Enter")
input("(1) Test of setting positions without dynamics")
from pyrep.objects.shape import Shape
obj = Shape.create(type=PrimitiveShape.CUBOID, size=[0.05,0.05,0.05])
obj.set_position([0.8,0.1,0.05])

for _ in range(100): pr.step()

input(">>> 100 ")
jj = [0.,1.2,0.,-1.,0.,2.,0.]
panda.set_joint_positions(jj, disable_dynamics=True)
obj.set_position([0.75,0.1,0.05])
for _ in range(20): pr.step()
for i in range(100):
    jj[0]+=0.01
    panda.set_joint_positions(jj, disable_dynamics=True)

input(">>> 10 ")
jj = [0.,1.2,0.,-1.,0.,2.,0.]
panda.set_joint_positions(jj, disable_dynamics=True)
obj.set_position([0.75,0.1,0.05])
for _ in range(20): pr.step()
for i in range(10):
    jj[0]+=0.1
    panda.set_joint_positions(jj, disable_dynamics=True)

input(">>> 1 ")
jj = [0.,1.2,0.,-1.,0.,2.,0.]
panda.set_joint_positions(jj, disable_dynamics=True)
obj.set_position([0.75,0.1,0.05])
for _ in range(20): pr.step()
for i in range(1):
    jj[0]+=1.0
    panda.set_joint_positions(jj, disable_dynamics=True)
for _ in range(100): pr.step()


input("(2) Velocity control")

for j in range(7):
    panda.joints[j].set_control_loop_enabled(False)
    print(panda.joints[j].is_control_loop_enabled())

input(">>> test ")
    for i in range(50):
        panda.set_joint_target_velocities(-0.1 * np.ones((7)))
        pr.step()
    panda.get_joint_target_velocities()

print(f"Joint positions: {panda.get_joint_positions()}")
print(f"Simulation timestep default: {pr.get_simulation_timestep()}")
pr.set_simulation_timestep(dt=0.05)
print(f"Simulation timestep set to: {pr.get_simulation_timestep()}")

print("Done")
exit()

panda.joints[0].set_joint_target_velocity(0.1)
for i in range(50): pr.step()

panda.set_joint_target_positions([-1.5936896800994873,
 0.6666249632835388,
 1.3229182958602905,
 -2.5812854766845703,
 -1.1160962581634521,
 2.4099419116973877,
 0.6573420763015747])
panda.get_joint_target_positions()
