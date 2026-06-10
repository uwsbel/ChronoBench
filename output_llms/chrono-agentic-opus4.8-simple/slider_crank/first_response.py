import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # g = 9.81 down (Y-up)

crank_speed = chrono.CH_PI                                            # crank angular speed [rad/s]
crank_len = 1.0                                                       # crank throw [m]
rod_len = 4.0                                                         # connecting-rod length [m]

# Floor / truss — the fixed reference body of the mechanism
floor = chrono.ChBodyEasyBox(10, 1, 10, 1000)                         # big slab, density 1000
floor.SetPos(chrono.ChVector3d(0, -3.0, 0))                          # below the mechanism
floor.SetFixed(True)                                                  # immovable truss
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # textured floor
sys.Add(floor)                                                        # add the truss

# Crankshaft — a cylinder spinning about world Z at the origin pivot
crank = chrono.ChBody()                                               # manual link body
crank.SetMass(2.0)                                                    # crank mass [kg]
crank.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))              # crank inertia
crank.SetPos(chrono.ChVector3d(crank_len * 0.5, 0, 0))               # COM at crank midpoint
crank.SetRot(chrono.QUNIT)                                            # body-local X along world X
crank_cyl = chrono.ChVisualShapeCylinder(0.1, crank_len)             # crank visual cylinder
crank_cyl.SetColor(chrono.ChColor(0.6, 0.2, 0.2))                    # red crank
crank.AddVisualShape(crank_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X
sys.Add(crank)                                                        # add the crank

# Connecting rod — links crank-pin to the piston wrist-pin
rod = chrono.ChBody()                                                 # manual link body
rod.SetMass(1.5)                                                      # rod mass [kg]
rod.SetInertiaXX(chrono.ChVector3d(0.04, 0.04, 0.04))               # rod inertia
rod.SetPos(chrono.ChVector3d(crank_len + rod_len * 0.5, 0, 0))       # COM between the two pins
rod.SetRot(chrono.QUNIT)                                              # rod along world X at start
rod_cyl = chrono.ChVisualShapeCylinder(0.08, rod_len)                # rod visual cylinder
rod_cyl.SetColor(chrono.ChColor(0.2, 0.4, 0.6))                      # blue rod
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z->X
sys.Add(rod)                                                          # add the rod

# Piston — slides along the world X guide
piston = chrono.ChBody()                                              # manual slider body
piston.SetMass(1.0)                                                   # piston mass [kg]
piston.SetInertiaXX(chrono.ChVector3d(0.03, 0.03, 0.03))            # piston inertia
piston.SetPos(chrono.ChVector3d(crank_len + rod_len, 0, 0))          # at the wrist-pin
piston_cyl = chrono.ChVisualShapeCylinder(0.3, 0.5)                  # piston visual cylinder
piston_cyl.SetColor(chrono.ChColor(0.3, 0.3, 0.3))                  # grey piston
piston.AddVisualShape(piston_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # axis along X
sys.Add(piston)                                                       # add the piston

# crank <-> floor : prescribed-speed motor (FULL motor-link, no separate revolute)
motor = chrono.ChLinkMotorRotationSpeed()                            # speed-driven crank motor
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # pivot at origin
motor.SetSpeedFunction(chrono.ChFunctionConst(crank_speed))         # constant angular speed
sys.AddLink(motor)                                                   # add the motor link

# crank <-> rod : revolute crank-pin (hinge about world Z -> QUNIT)
crank_rod = chrono.ChLinkLockRevolute()                             # crank-pin hinge
crank_rod.Initialize(crank, rod, True,
                     chrono.ChFramed(chrono.ChVector3d(crank_len * 0.5, 0, 0), chrono.QUNIT),   # crank far end
                     chrono.ChFramed(chrono.ChVector3d(-rod_len * 0.5, 0, 0), chrono.QUNIT))    # rod near end
sys.AddLink(crank_rod)                                              # add the crank-pin

# rod <-> piston : revolute wrist-pin (hinge about world Z -> QUNIT)
rod_piston = chrono.ChLinkLockRevolute()                            # wrist-pin hinge
rod_piston.Initialize(rod, piston, True,
                      chrono.ChFramed(chrono.ChVector3d(rod_len * 0.5, 0, 0), chrono.QUNIT),    # rod far end
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))                # piston center
sys.AddLink(rod_piston)                                             # add the wrist-pin

# piston <-> floor : prismatic guide along world X (map frame +Z to X)
piston_guide = chrono.ChLinkLockPrismatic()                        # piston slides on fixed guide
piston_guide.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(crank_len + rod_len, 0, 0),
                                                       chrono.Q_ROTATE_Z_TO_X))  # guide axis = X
sys.AddLink(piston_guide)                                          # add the prismatic guide

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht renderer
vis.AttachSystem(sys)                                              # bind the physical system
vis.SetWindowSize(1280, 720)                                      # window resolution
vis.SetWindowTitle("Crank-Slider Mechanism")                     # window title
vis.Initialize()                                                  # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                   # sky box backdrop
vis.AddCamera(chrono.ChVector3d(2.5, 2.0, -7), chrono.ChVector3d(2.5, 0, 0))  # camera eye + target on the stroke centerline
vis.AddTypicalLights()                                            # standard lighting

time_step = 1e-3                                                  # integration step [s]
sim_end = 10.0                                                    # total sim time [s]
render_fps = 50.0                                                 # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
