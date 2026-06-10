import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC system, joints only
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # gravity along -Y (motion in x-y plane)

crank_len = 1.0                                                       # crank throw length
rod_len = 4.0                                                         # connecting-rod length
piston_x = crank_len + rod_len                                       # piston rides along +X guide

floor = chrono.ChBody()                                              # fixed reference (truss/floor)
floor.SetFixed(True)                                                 # ground is immovable
sys.AddBody(floor)

crank = chrono.ChBodyEasyBox(1.5, 0.5, 0.5, 1000, True, False)       # crank as a box (visual only, no collision)
crank.SetPos(chrono.ChVector3d(crank_len / 2.0, 0, 0))              # centered between origin pivot and crank-pin
sys.AddBody(crank)

rod = chrono.ChBodyEasyBox(rod_len, 0.4, 0.4, 1000, True, False)     # connecting rod box
rod.SetPos(chrono.ChVector3d(crank_len + rod_len / 2.0, 0, 0))      # centered between crank-pin and wrist-pin
sys.AddBody(rod)

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.4, 0.6, 1000, True, False)  # piston cylinder
piston.SetPos(chrono.ChVector3d(piston_x, 0, 0))                    # at the far end of the linkage
sys.AddBody(piston)

motor = chrono.ChLinkMotorRotationSpeed()                            # full motor-link: crank <-> floor about Z
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # ground pivot at origin, hinge +Z
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))        # constant pi rad/s
sys.AddLink(motor)

crank_rod = chrono.ChLinkLockSpherical()                             # crank-pin now a spherical (ball-and-socket) joint
crank_rod.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(crank_len, 0, 0)))  # at crank-pin world point
sys.AddLink(crank_rod)

rod_piston = chrono.ChLinkLockSpherical()                            # wrist-pin now a spherical (ball-and-socket) joint
rod_piston.Initialize(rod, piston, chrono.ChFramed(chrono.ChVector3d(piston_x, 0, 0)))  # at wrist-pin world point
sys.AddLink(rod_piston)

piston_floor = chrono.ChLinkLockPlanar()                            # plane-plane joint: piston confined to the x-y plane
piston_floor.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(piston_x, 0, 0), chrono.QUNIT))  # plane normal = local +Z
sys.AddLink(piston_floor)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank (spherical pins, planar piston)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, 2.5, 6), chrono.ChVector3d(2.5, 0, 0))  # look down the x-y plane
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
sim_end = 10.0                                                       # total simulated time
render_fps = 50.0                                                    # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
