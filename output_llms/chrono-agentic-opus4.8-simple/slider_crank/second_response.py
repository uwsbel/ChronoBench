import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                          # rigid-body NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up gravity, g = 9.81

crank_speed = chrono.CH_PI                                          # crank angular speed [rad/s]
crank_len = 1.0                                                     # crank radius / throw [m]
rod_len = 4.0                                                       # connecting-rod length [m]

floor = chrono.ChBodyEasyBox(3, 1, 3, 1000)                        # fixed truss / floor block
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))                         # sit just below the pivot
floor.SetFixed(True)                                               # immovable reference body
sys.AddBody(floor)                                                  # add floor to system

crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.1, crank_len, 1000) # crank link (visual cylinder)
crank.SetPos(chrono.ChVector3d(crank_len / 2.0, 0, 0))             # centered between pivot and crank-pin
crank.SetRot(chrono.QuatFromAngleZ(0))                             # crank starts along +X
sys.AddBody(crank)                                                  # add crank to system

rod = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.05, rod_len, 1000)    # connecting rod (visual cylinder)
rod.SetPos(chrono.ChVector3d(crank_len + rod_len / 2.0, 0, 0))    # spans crank-pin to wrist-pin
sys.AddBody(rod)                                                    # add rod to system

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.2, 0.3, 1000)     # piston / slider block
piston.SetPos(chrono.ChVector3d(crank_len + rod_len, 0, 0))       # at far end of the rod
sys.AddBody(piston)                                                # add piston to system

motor = chrono.ChLinkMotorRotationSpeed()                          # full motor-link, no extra revolute
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # spin about the ground pivot
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))       # constant angular speed
sys.AddLink(motor)                                                 # add motor to system

crank_rod = chrono.ChLinkLockRevolute()                            # crank-pin revolute
crank_rod.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(crank_len, 0, 0)))  # at the crank-pin
sys.AddLink(crank_rod)                                             # add crank-pin joint

rod_piston = chrono.ChLinkLockRevolute()                           # wrist-pin revolute
rod_piston.Initialize(rod, piston, chrono.ChFramed(chrono.ChVector3d(crank_len + rod_len, 0, 0)))  # at the wrist-pin
sys.AddLink(rod_piston)                                            # add wrist-pin joint

piston_floor = chrono.ChLinkLockPrismatic()                       # piston slides on the fixed guide
piston_floor.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(crank_len + rod_len, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # guide along X
sys.AddLink(piston_floor)                                          # add prismatic guide

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(sys)                                             # bind the physical system
vis.SetWindowSize(1280, 720)                                      # window resolution
vis.SetWindowTitle("Slider-Crank")                                # window title
vis.Initialize()                                                  # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # corner logo
vis.AddSkyBox()                                                    # sky background
vis.AddCamera(chrono.ChVector3d(3, 3, 6), chrono.ChVector3d(3, 0, 0))  # eye and look-at
vis.AddTypicalLights()                                            # standard lighting

time_step = 1e-3                                                   # integration step [s]
sim_end = 20.0                                                     # stop after 20 s
render_fps = 50.0                                                  # frames per second for the review video
render_every = max(1, round(1.0 / (render_fps * time_step)))      # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                             # advance one physics step
        if sys.GetChTime() >= sim_end:
            break
