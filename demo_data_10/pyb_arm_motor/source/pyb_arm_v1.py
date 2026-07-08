"""PyBullet source experiment (the artifact to be converted): a two-link planar arm built
imperatively with createMultiBody. Link 0 (crank, 0.4 m rod, 0.5 kg) is velocity-controlled about y,
soft-started over 0.5 s to a constant 1.5 rad/s; link 1 (pendulum, 0.5 m rod, 0.3 kg) hangs FREE from the crank
tip (note the PyBullet idiom: joints are born with a velocity motor engaged, so freeing joint 1
requires an explicit motor command with force=0). A viscous elbow damper (0.05 N m s/rad) is
applied EXPLICITLY each step as a torque -b * joint rate, so the startup transient decays and a
steady driven swing remains. No collision shapes.
Logs the crank rate and the pendulum's ABSOLUTE angle from vertical (the two joint angles add).
"""
import json
import math

import pybullet as p

L1, L2 = 0.4, 0.5
M1, M2 = 0.5, 0.3
OMEGA = 1.5
B_ELBOW = 0.05
DT = 1.0 / 1000.0
T_END = 10.0

p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

arm = p.createMultiBody(
    baseMass=0.0,                                   # fixed mount
    basePosition=[0, 0, 1.5],
    linkMasses=[M1, M2],
    linkCollisionShapeIndices=[-1, -1],
    linkVisualShapeIndices=[-1, -1],
    linkPositions=[[0, 0, 0],                        # crank joint at the mount
                   [0, 0, -L1]],                     # pendulum joint at the crank TIP (positions
                                                     # are relative to the parent JOINT frame)
    linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
    linkInertialFramePositions=[[0, 0, -0.2],        # crank COM mid-rod below its joint
                                [0, 0, -0.25]],      # pendulum COM mid-rod below its joint
    linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 1, 0], [0, 1, 0]])

# explicit rod inertias (no collision shapes to derive them from)
p.changeDynamics(arm, 0, localInertiaDiagonal=[M1 * L1 ** 2 / 12, M1 * L1 ** 2 / 12, 1e-4])
p.changeDynamics(arm, 1, localInertiaDiagonal=[M2 * L2 ** 2 / 12, M2 * L2 ** 2 / 12, 1e-4])
# also disable PyBullet's default joint damping
p.changeDynamics(arm, 0, linearDamping=0, angularDamping=0, jointDamping=0)
p.changeDynamics(arm, 1, linearDamping=0, angularDamping=0, jointDamping=0)

# FREE the pendulum joint (force=0 disengages the default motor); the crank motor target is
# retargeted every step with a 0.5 s soft-start ramp (a velocity step would kick the free
# pendulum impulsively through the joint)
T_RAMP = 0.5
p.setJointMotorControl2(arm, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

rows = []
t = 0.0
n = int(round(T_END / DT))
for k in range(n):
    target = OMEGA * min(k * DT / T_RAMP, 1.0)
    p.setJointMotorControl2(arm, 0, p.VELOCITY_CONTROL, targetVelocity=target, force=500)
    qd1 = p.getJointState(arm, 1)[1]
    p.setJointMotorControl2(arm, 1, p.TORQUE_CONTROL, force=-B_ELBOW * qd1)
    p.stepSimulation()
    t = (k + 1) * DT
    q0, w0 = p.getJointState(arm, 0)[0:2]
    q1 = p.getJointState(arm, 1)[0]
    rows.append((t, w0, q0 + q1))                    # absolute pendulum angle = q0 + q1

with open("out_pyb.csv", "w") as fh:
    fh.write("t,w,theta\n")
    for r in rows:
        fh.write(f"{r[0]:.6f},{r[1]:.6e},{r[2]:.6e}\n")

print(json.dumps({"theta_max": max(abs(r[2]) for r in rows),
                  "w_mean_tail": sum(r[1] for r in rows if r[0] >= 1.0) /
                                 len([1 for r in rows if r[0] >= 1.0]),
                  "omega_drive": OMEGA}))
