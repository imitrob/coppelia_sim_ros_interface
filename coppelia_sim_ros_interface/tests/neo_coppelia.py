'''
This is test to use Neo script on Coppelia Sim.
The Neo script computation takes values from CoppeliaSim as feedback.

Two controller classes created: 1) Swift-Neo, 2) Coppelia-PyRep

Credit: Peter Corke, Jesse Haviland

- [ ] Need some tuning and polishing
- [ ] Decide, if can be useful
'''
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import cProfile

# swift
import swift
# coppelia
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape, TextureMappingMode, JointMode
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.errors import ConfigurationPathError
from pyrep.objects.dummy import Dummy

class NeoController():
    def __init__(self, target, collisions, joints, panda):
        ''' New object -> new target with given parameters (joints, collisions, ..)
        Paramters:
            target (float[3]): xyz target
            collisions ([n]): Objects info
            joints (float[7]): joints of robot now
        '''
        self.panda = panda
        self.panda.q = joints
        self.n = 7

        self.collisions = collisions
        self.Tep = self.panda.fkine(self.panda.q)
        self.Tep.A[:3, 3] = target

    def step(self, joints, collisions, Y = 0.01, servo_gain=0.5, servo_threshold=0.01, ps=0.05, pi=0.9, di=0.3, ds=0.05, xi=1.0):
        ''' Based on new collisions and current joints, computes the next qd (velocities)
        Parameters:
            joints (float[7]): joints of robot now
            collisions (): Collisions update

        (Neo parameters):
            Y (default: 0.01): Gain term (lambda) for control minimisation
        - servo
            servo_gain (default: 0.5): Calulate the required end-effector spatial velocity for the robot
            to approach the goal. Gain is set to 1.0
            servo_threshold (default: 0.01)
        - joint_velocity_damper:
            ps (default: 0.05): The minimum angle (in radians) in which the joint is allowed to approach
            to its limit
            pi (default: 0.9): The influence angle (in radians) in which the velocity damper
            becomes active
        - link collisions damper
            di (default: 0.3): The influence distance in which the velocity
            damper becomes active
            ds (default: 0.05): The minimum distance in which a joint is allowed to
            approach the collision object shape
            xi (default: 1.0): The gain for the velocity damper
        '''
        # update the panda.q & collisions
        self.panda.q = joints
        #for n,collision in enumerate(collisions):
        #    x,y,z = collision
        #    self.collisions[n].base = sm.SE3(x,y,z)

        ''' This is copied code from Neo controller by Peter Corke, Jesse H. '''
        # The pose of the Panda's end-effector
        Te = self.panda.fkine(self.panda.q)

        # Transform from the end-effector to desired pose
        eTep = Te.inv() * self.Tep

        # Spatial error
        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))
        print("spatial error", e)

        v, arrived = rtb.p_servo(Te, self.Tep, servo_gain, servo_threshold)

        # Quadratic component of objective function
        Q = np.eye(self.n + 6)

        # Joint velocity component of Q
        Q[:self.n, :self.n] *= Y

        # Slack component of Q
        Q[self.n:, self.n:] = (1 / e) * np.eye(6)

        # The equality contraints
        Aeq = np.c_[self.panda.jacobe(self.panda.q), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((self.n + 6, self.n + 6))
        bin = np.zeros(self.n + 6)

        # Form the joint limit velocity damper
        Ain[:self.n, :self.n], bin[:self.n] = self.panda.joint_velocity_damper(ps, pi, self.n)

        # For each collision in the scene
        for collision in self.collisions:

            # Form the velocity damper inequality contraint for each collision
            # object on the robot to the collision in the scene
            c_Ain, c_bin = self.panda.link_collision_damper(
                collision,
                self.panda.q[:self.n],
                di,
                ds,
                xi,
                start=self.panda.link_dict["panda_link1"],
                end=self.panda.link_dict["panda_hand"],
            )

            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-self.panda.jacobm(self.panda.q).reshape((self.n,)), np.zeros(6)]

        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[self.panda.qdlim[:self.n], 10 * np.ones(6)]
        ub = np.r_[self.panda.qdlim[:self.n], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
        if qd is None:
            return self.panda.q, None, arrived
        # Apply the joint velocities to the Panda
        self.panda.qd[:self.n] = qd[:self.n]

        return self.panda.q, self.panda.qd, arrived

class Cop:
    def __init__(self, q_init, collision_positions):

        self.pr = PyRep()
        self.pr.launch('/home/petr/my_ws/src/coppelia_sim_ros_interface/include/scenes/scene_panda_franka_gripper.ttt', headless=False, responsive_ui=True)
        self.pr.start()  # Start the simulation
        self.pr.step()

        self.panda = Panda()  # Get the panda from the scene
        self.gripper = PandaGripper()  # Get the panda gripper from the scene

        self.pr.set_simulation_timestep(dt=0.05)
        # Turn on velocity control
        for j in range(7): self.panda.joints[j].set_control_loop_enabled(False)

        # scene setup
        for collision_position in collision_positions:
            object = Shape.create(type=PrimitiveShape.SPHERE, color=[1.,0.,0.], size=[0.1, 0.1, 0.1], position=collision_position)
        self.pr.step()
        self.panda_target = Dummy("Panda_target")

        self.panda.set_joint_positions(q_init, disable_dynamics=True)
        self.gripper.actuate(0.0, velocity=0.04)

    def new_target(self, target):
        self.panda_target.set_position(target)

    def update(self, q, qd, collisions):
        self.panda.set_joint_target_velocities(qd)
        self.pr.step()

    def sim_further(self, n=100):
        # Change to position control and back before simulate,
        for j in range(7): self.panda.joints[j].set_control_loop_enabled(True)
        for _ in range(n): self.pr.step()
        for j in range(7): self.panda.joints[j].set_control_loop_enabled(False)

class Swi:
    def __init__(self, q_init, collision_positions):
        self.env = swift.Swift()
        self.env.launch()
        self.panda = rtb.models.Panda()
        self.panda.q = q_init

        # scene mimic
        self.collisions = []
        for collision in collision_positions:
            x,y,z = collision
            s0 = sg.Sphere(radius=0.05, base=sm.SE3(x,y,z))
            self.collisions.append(s0)
            self.env.add(s0)

        self.env.add(self.panda)
        self.target = sg.Sphere(radius=0.02, base=sm.SE3(*[0.,0.,0.]))
        self.env.add(self.target)

    def new_target(self, target):
        self.target.base = sm.SE3(*target)

    def update(self, q, qd, collisions):
        self.panda.q = q
        for collision, c_obj in zip(collisions, self.collisions):
            x,y,z = collision
            c_obj.base = sm.SE3(x,y,z)

        self.panda.qd = qd

        self.env.step(0.05)



if __name__ == '__main__':
    # scene info
    q_init = [0.,0.0,0.0,-2.5,0.0,2.5,0.0]
    collision_positions = []#[[0.6, -0.3, 0.1], [0.6, -0.1, 0.1]]
    target = [0.4, -0.2, 0.1]

    # Two simulator objects
    swi = Swi(q_init, collision_positions)
    swi.new_target(target)
    cop = Cop(q_init, collision_positions)
    cop.new_target(target)

    nc = NeoController(target, swi.collisions, q_init, swi.panda)

    arrived, n = False, 0
    while not arrived:
        # read qs
        #q = swi.panda.q
        q = np.array(cop.panda.get_joint_positions())

        # Print info
        print(f"Time: {round(n*0.05,2)}s sims_diff: {round(np.sum(np.array(swi.panda.q) - np.array(cop.panda.get_joint_positions())),2)}")

        # Compute new values
        q, qd, arrived = nc.step(q, collisions=[])
        if qd is None:
            print("ERROR")
            break

        # Update sims
        collisions = []
        cop.update(q, qd, collisions)
        swi.update(q, qd, collisions)

        # Exit on stale
        n+=1
        if n >600: break

    np.linalg.norm(cop.panda.get_tip().get_position() - np.array(target))
    print(f"sims_diff: {np.round(np.sum(np.array(swi.panda.q) - np.array(cop.panda.get_joint_positions())),2)}")
    target

    q = _SE3_to_quaternion(sm.SE3([0.,0.,0.]))
    q
    UnitQuaternion([1.,0.,0.,0.])
    sm.SE3(UnitQuaternion([1.,0.,0.,0.]))

    a = np.array([1,2,3,4,5])
    np.roll(a, -1)

    sm.base.transl(1,2,3) @ sm.base.rpy2tr(0.1,0.2,0.3)
    ss = UnitQuaternion(np.roll(quat,1)).SE3()
    sm.SE3(ss)
    sm.SE3(ss) @ sm.base.rpy2tr(0.1,0.2,0.3)


    e = UnitQuaternion(np.roll(quat,1)).eul()
    e[0] = e[0] + np.pi*2
    UnitQuaternion.Eul(e).vec3


    UnitQuaternion(np.roll(quat,1)).Rx(np.pi).vec_xyzs
    UnitQuaternion(np.roll(quat,1)).Rx(np.pi).vec_sxyz

    cop.sim_further(n=1000)
    pos = [0.5, 0.0, 0.4]
    quat = [0.5,-0.5,-0.5,0.5] # pointing -x
    joints = [2.5816686153411865, -1.0207289457321167, -2.1855108737945557, -1.1067397594451904, 0.1925951987504959, 0.2824327051639557, 0.1741429716348648]

    quat = [0.,1.,0.,0.] # pointing -z
    joints = [2.084409236907959, 0.5525133013725281, -2.247859001159668, -2.2499310970306396, 0.4297836124897003, 1.7932137250900269, 0.442534863948822]

    quat = [0.,0.,1.,0.] # pointing z
    joints = [2.511841297149658, -0.6771138310432434, -2.673689365386963, -2.41568660736084, 2.0900797843933105, 0.3325619101524353, 0.01430448330938816]

    joints = cop.panda.solve_ik_via_jacobian(pos, quaternion=quat)
    joints = cop.panda.solve_ik_via_sampling(pos, quaternion=quat)[0]

    cop.panda.get_tip().get_position()
    cop.panda.get_tip().get_quaternion()

    swi.panda.fkine(joints).t


    import time
    ds = []
    ts = []
    quat = [0.,1.,0.,0.]
    bound_box = [[0.3,0.6], [-0.2, 0.2], [0.0, 0.4]]
    for i in range(50):
        t1 = time.perf_counter()
        # pick random sample
        pos = [round(random.uniform(*bound_box[d]),2) for d in range(3)]
        quat_ = np.roll(quat, 1) # xyzw -> wxyz
        p = sm.SE3(pos) @ UnitQuaternion(quat_).SE3()

        # generate ik
        while True:
            joints = swi.panda.ikine_LM(p, search=True).q
            if np.array([swi.panda.qlim[0][i] < joints[i] < swi.panda.qlim[1][i] for i in range(7)]).all():
                break

        # go there
        cop.panda.set_joint_target_positions(joints)
        cop.sim_further(n=200)

        # compare results
        diff_pos = np.linalg.norm(np.array(cop.panda.get_tip().get_position()) - np.array(pos))
        diff_ori = np.dot(cop.panda.get_tip().get_quaternion(), quat)
        ds.append([diff_pos, diff_ori])
        ts.append(time.perf_counter()-t1)



ds = np.array(ds)
from spatialmath import UnitQuaternion
import random
import matplotlib.pyplot as plt

UnitQuaternion([0.,0.,0.,1.0]).slerp
np.mean(np.array(ts))
import spatialmath as sm
sm.base.quaternions.qslerp

import spatialmath
spatialmath.__dict__
from spatialmath.base import qslerp

fig, ax = plt.subplots(nrows=2, ncols=1, figsize=(5, 5))
ax[0].hist(ds[:,0], bins=20)
ax[0].set_xlabel("Distance error [m]")
ax[0].set_ylabel("Number of measurements")
ax[1].hist(ds[:,1], color='b', bins=20)
ax[1].set_xlabel("Distance error [rad]")
ax[1].set_ylabel("Number of measurements")
plt.savefig('/home/petr/Pictures/ik_test_in_bounding_box2.svg', format='svg')
ds = np.load('/home/petr/Pictures/ik_test_in_bounding_box.npy')







#
