import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity, g = 9.81

# --- mechanism geometry (slider-crank in the XY plane) ---
crank_len = 1.0          # crank throw (pivot -> crank pin) [m]
rod_len = 4.0            # connecting-rod length (crank pin -> wrist pin) [m]
crank_rad = 0.1          # crank/rod visual cylinder radius [m]
piston_size = 0.5        # piston cube side [m]
density = 1000.0         # body density [kg/m^3]
motor_speed = chrono.CH_PI   # crank angular speed [rad/s]

# --- floor / truss (fixed reference body) ---
floor = chrono.ChBodyEasyBox(3, 1, 3, density, True, False)           # visual-only truss block
floor.SetPos(chrono.ChVector3d(0, -0.6, 0))                           # below the crank axis
floor.SetFixed(True)                                                  # truss is the fixed frame
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # textured truss
sys.Add(floor)

# --- crankshaft (rotates about the world Z axis at the origin) ---
crank = chrono.ChBody()                                               # manual body for the rotating crank
crank.SetPos(chrono.ChVector3d(crank_len / 2, 0, 0))                  # COM at midpoint of the throw
crank.SetMass(2.0)                                                    # crank mass [kg]
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))                  # crank inertia [kg*m^2]
crank_cyl = chrono.ChVisualShapeCylinder(crank_rad, crank_len)        # crank arm cylinder
crank.AddVisualShape(crank_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # local Z -> X
crank.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))       # red crank
sys.Add(crank)

# --- connecting rod (links the crank pin to the piston wrist pin) ---
rod = chrono.ChBody()                                                 # manual body for the rod
rod.SetPos(chrono.ChVector3d(crank_len + rod_len / 2, 0, 0))         # COM at rod midpoint (start straight along +X)
rod.SetMass(1.0)                                                      # rod mass [kg]
rod.SetInertiaXX(chrono.ChVector3d(0.05, 0.5, 0.5))                   # rod inertia [kg*m^2]
rod_cyl = chrono.ChVisualShapeCylinder(crank_rad, rod_len)           # rod cylinder
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # local Z -> X
rod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.4, 0.6))        # blue rod
sys.Add(rod)

# --- piston (slides along the world X guide) ---
piston = chrono.ChBodyEasyBox(piston_size, piston_size, piston_size, density, True, False)  # piston cube
piston.SetPos(chrono.ChVector3d(crank_len + rod_len, 0, 0))          # at full extension along +X
piston.SetMass(1.0)                                                  # piston mass [kg]
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))    # grey piston
sys.Add(piston)

# --- motor: drives the crank about Z at constant speed (full motor-link, no extra revolute) ---
motor = chrono.ChLinkMotorRotationSpeed()                             # prescribed-speed motor
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # crank pivot at origin, hinge about +Z
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))          # constant angular speed [rad/s]
sys.AddLink(motor)

# --- revolute: crank pin (crank far end <-> rod near end) ---
crank_rod = chrono.ChLinkLockRevolute()                              # crank-pin hinge about +Z
crank_rod.Initialize(crank, rod, True,
                     chrono.ChFramed(chrono.ChVector3d(crank_len / 2, 0, 0), chrono.QUNIT),   # crank far end (local +X)
                     chrono.ChFramed(chrono.ChVector3d(-rod_len / 2, 0, 0), chrono.QUNIT))    # rod near end (local -X)
sys.AddLink(crank_rod)

# --- revolute: wrist pin (rod far end <-> piston) ---
rod_piston = chrono.ChLinkLockRevolute()                             # wrist-pin hinge about +Z
rod_piston.Initialize(rod, piston, True,
                      chrono.ChFramed(chrono.ChVector3d(rod_len / 2, 0, 0), chrono.QUNIT),    # rod far end (local +X)
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))              # piston center
sys.AddLink(rod_piston)

# --- prismatic: piston slides on the fixed X guide (slider <-> ground) ---
piston_floor = chrono.ChLinkLockPrismatic()                          # piston slides along world X
piston_floor.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(crank_len + rod_len, 0, 0),
                                                       chrono.Q_ROTATE_Z_TO_X))  # frame +Z -> world X guide axis
sys.AddLink(piston_floor)

# --- Irrlicht visualization (Initialize first, then scene elements; NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)                                          # window resolution
vis.SetWindowTitle("Crank-Slider Mechanism")                         # window title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo overlay
vis.AddSkyBox()                                                      # sky box backdrop
vis.AddCamera(chrono.ChVector3d(2, 2, 7), chrono.ChVector3d(2.5, 0, 0))  # eye, look-at the mechanism
vis.AddTypicalLights()                                              # standard two-light setup

time_step = 1e-3                                                     # integration step [s]
sim_end = 10.0                                                       # total simulated time [s]
render_fps = 50.0                                                    # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
