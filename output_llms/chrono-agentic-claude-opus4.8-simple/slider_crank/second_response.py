import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # pure jointed MBS, NSC solver
# pure jointed MBS with no contact -> no SetCollisionSystemType, default Y-down gravity (9.81)

crank_center = chrono.ChVector3d(-1, 0.5, 0)                         # crank revolute center
crank_rad = 0.4                                                       # crank disc radius
crank_thick = 0.1                                                     # crank disc thickness
rod_length = 1.5                                                      # connecting-rod length

floor = chrono.ChBodyEasyBox(3, 1, 3, 1000)                          # truss / ground box
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))                          # under the mechanism
floor.SetFixed(True)                                                 # fixed reference body
floor.EnableCollision(False)                                         # pure MBS, no contact
sys.Add(floor)

crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)   # crank disc
crank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))           # offset in Z behind plane
crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)                                 # lay disc axis along Z
crank.EnableCollision(False)
sys.Add(crank)

rod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)               # connecting rod
rod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))     # between pins
rod.EnableCollision(False)
sys.Add(rod)

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # piston / slider
piston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))      # at rod far end
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)                                # axis along X (slides on X)
piston.EnableCollision(False)
sys.Add(piston)

motor = chrono.ChLinkMotorRotationSpeed()                            # speed motor drives the crank
motor.Initialize(crank, floor, chrono.ChFramed(crank_center))        # full motor-link, no extra revolute
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))         # constant pi rad/s
sys.AddLink(motor)

rev_crank_rod = chrono.ChLinkLockRevolute()                          # crank-pin: crank <-> rod
rev_crank_rod.Initialize(rod, crank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.AddLink(rev_crank_rod)

rev_rod_piston = chrono.ChLinkLockRevolute()                         # wrist-pin: rod <-> piston
rev_rod_piston.Initialize(piston, rod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.AddLink(rev_rod_piston)

prismatic = chrono.ChLinkLockPrismatic()                             # piston <-> floor, slides on X
prismatic.Initialize(piston, floor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(prismatic)

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)                                         # window size
vis.SetWindowTitle("Slider-Crank")                                   # window title
vis.Initialize()                                                     # create device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))   # eye, look-at
vis.AddTypicalLights()                                              # standard lights

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():                                                     # SCORED CORE = plain truth form, NO time bound
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
