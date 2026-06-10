import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # rigid-body NSC system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity

# --- floor / truss: the fixed reference body the mechanism hangs off of
floor = chrono.ChBody()                                               # truss body
floor.SetFixed(True)                                                  # immovable ground
floor.SetName("floor")
sys.AddBody(floor)
floor_box = chrono.ChVisualShapeBox(3, 1, 3)                          # full extents (W, H, D)
floor_box.SetColor(chrono.ChColor(0.4, 0.4, 0.5))                     # bluish floor
floor.AddVisualShape(floor_box, chrono.ChFramed(chrono.ChVector3d(0, -1.5, 0)))  # below mechanism

# --- crankshaft: rotates about the ground pivot, driven by the speed motor
crank = chrono.ChBody()                                               # crank body
crank.SetPos(chrono.ChVector3d(1, 0, 0))                              # crank center between pivot and crank-pin
crank.SetName("crank")
sys.AddBody(crank)
crank_cyl = chrono.ChVisualShapeCylinder(0.1, 2.1)                    # crank bar (radius, length)
crank.AddVisualShape(crank_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # body-local Z -> X
crank_pin = chrono.ChVisualShapeCylinder(0.08, 0.4)                   # crank-pin stub
crank.AddVisualShape(crank_pin, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))  # at the crank-pin end

# --- connecting rod: links the crank-pin to the piston wrist-pin
rod = chrono.ChBody()                                                 # connecting rod body
rod.SetPos(chrono.ChVector3d(4, 0, 0))                                # rod center (between crank-pin x=2 and piston x=6)
rod.SetName("rod")
sys.AddBody(rod)
rod_cyl = chrono.ChVisualShapeCylinder(0.1, 4)                        # rod bar (radius, length)
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # along body-local X

# --- piston: slides on the fixed horizontal guide (world +X)
piston = chrono.ChBody()                                              # piston body
piston.SetPos(chrono.ChVector3d(6, 0, 0))                             # piston center on the guide
piston.SetName("piston")
sys.AddBody(piston)
piston_cyl = chrono.ChVisualShapeCylinder(0.2, 0.5)                   # piston puck (radius, length)
piston.AddVisualShape(piston_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_PI_2)))  # axis along world Z

# --- crank <-> floor: prescribed-speed motor (full motor-link, NO separate revolute)
motor = chrono.ChLinkMotorRotationSpeed()                            # constant-speed drive
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # pivot at the origin
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))        # omega = pi rad/s
sys.AddLink(motor)

# --- crank <-> rod: crank-pin revolute (hinge about world Z)
crank_rod = chrono.ChLinkLockRevolute()                             # crank-pin joint
crank_rod.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))  # at the crank-pin in world coords
sys.AddLink(crank_rod)

# --- rod <-> piston: wrist-pin revolute (hinge about world Z)
rod_piston = chrono.ChLinkLockRevolute()                            # wrist-pin joint
rod_piston.Initialize(rod, piston, chrono.ChFramed(chrono.ChVector3d(6, 0, 0)))  # at the piston pin
sys.AddLink(rod_piston)

# --- piston <-> floor: prismatic guide along world +X (frame Z -> X)
piston_guide = chrono.ChLinkLockPrismatic()                         # sliding guide
piston_guide.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(6, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # slide on +X
sys.AddLink(piston_guide)

# --- Irrlicht visualization (Initialize first, then scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, -8), chrono.ChVector3d(3, 0, 0))  # look at the mechanism span
vis.AddTypicalLights()

time_step = 1e-3                                                      # integration step
sim_end = 20.0                                                        # stop after 20 s
motor_speed = chrono.CH_PI                                            # crank angular speed [rad/s]
render_fps = 50.0                                                     # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                # advance one step
        if sys.GetChTime() >= sim_end:
            break
