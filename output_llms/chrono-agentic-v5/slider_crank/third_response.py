"""Slider-crank mechanism (NSC, pure jointed multi-body, no contact).

Models a motor-driven crank-rod-piston linkage on a fixed floor:
  - floor  : fixed support box (the ground reference / guide host)
  - crank  : cylinder spun at constant angular speed by a rotation-speed motor
  - rod    : connecting rod between crank pin and piston
  - piston : slider constrained to the x-y plane

Topology (this variant uses ball-and-socket pins and a planar piston guide):
  - crank  <-> floor  : ChLinkMotorRotationSpeed  (full motor-link, no extra revolute)
  - rod    <-> crank  : ChLinkLockSpherical       (crank-pin, ball-and-socket)
  - piston <-> rod    : ChLinkLockSpherical       (wrist-pin, ball-and-socket)
  - piston <-> floor  : ChLinkLockPlanar          (planar joint: move/rotate in x-y plane)

Expected behavior: the crank rotates at constant speed, the spherical pins
transmit the motion through the rod, and the piston reciprocates in the x-y
plane while the planar joint keeps it in that plane. No collision system is
created because the mechanism is fully described by joints (no contact).
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Parameters === geometry/physics constants, then derived pin positions
crank_center = chrono.ChVector3d(-1, 0.5, 0)   # crank rotation axis location
crank_rad = 0.4        # crank radius [m] -> crank-pin offset from center
crank_thick = 0.1      # crank disk thickness [m]
rod_length = 1.5       # connecting-rod length [m]
time_step = 1e-3       # high-precision MBS step [s]
sim_end = 20.0         # bounded recording run [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))            # precomputed once

crank_pin = crank_center + chrono.ChVector3d(crank_rad, 0, 0)            # crank-rod pin (world)
wrist_pin = crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)  # rod-piston pin (world)


# === System & gravity === plain NSC system (pure jointed MBS, no contact)
sys = chrono.ChSystemNSC()

# === Bodies === fixed floor + crank disk + connecting rod + piston slider
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(wrist_pin)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# === Joints / constraints === motor + two ball-and-socket pins + planar piston guide
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_motor.SetMotorFunction(chrono.ChFunctionConst(chrono.CH_PI))   # constant angular speed [rad/s]
sys.Add(my_motor)

mjointA = chrono.ChLinkLockSpherical()                            # crank-pin: ball-and-socket
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_pin))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockSpherical()                            # wrist-pin: ball-and-socket
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(wrist_pin))
sys.Add(mjointB)

plane_plane = chrono.ChLinkLockPlanar()                           # piston confined to x-y plane
plane_plane.Initialize(mfloor, mpiston, chrono.ChFramed(wrist_pin, chrono.Q_ROTATE_Y_TO_X))
sys.Add(plane_plane)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# === Main loop === drive the crank linkage and advance the dynamics

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
