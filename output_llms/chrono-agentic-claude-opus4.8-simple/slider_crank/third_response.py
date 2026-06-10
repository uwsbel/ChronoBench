import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # rigid-body slider-crank, NSC
# pure jointed MBS (no contact) -> NO SetCollisionSystemType

crank_center = chrono.ChVector3d(-1, 0.5, 0)                         # crank rotation center
crank_rad = 0.4                                                       # crank radius
crank_thick = 0.1                                                    # crank disc thickness
rod_length = 1.5                                                      # connecting rod length

# floor / truss (fixed reference)
floor = chrono.ChBodyEasyBox(3, 1, 3, 1000)                          # 3x1x3 box, density 1000
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))                          # floor top at y=0
floor.SetFixed(True)                                                 # truss is fixed
floor.EnableCollision(False)                                         # no contact in this scene
sys.AddBody(floor)

# crank disc
crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # disc cylinder
crank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))           # slightly behind plane
crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)                                 # lay disc axis along Z
crank.EnableCollision(False)
sys.AddBody(crank)

# connecting rod
rod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)               # slender rod box
rod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # between crank-pin and wrist
rod.EnableCollision(False)
sys.AddBody(rod)

# piston
piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # short cylinder piston
piston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # at wrist-pin end
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)                                # piston axis along X
piston.EnableCollision(False)
sys.AddBody(piston)

# motor: drives crank relative to floor at the crank center (full motor-link, no companion revolute)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(crank_center))        # crank pivots on the floor
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))         # constant pi rad/s
sys.AddLink(motor)

# crank-pin: SPHERICAL (ball-and-socket) joint crank <-> rod
spherical_crank_rod = chrono.ChLinkLockSpherical()
spherical_crank_rod.Initialize(rod, crank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.AddLink(spherical_crank_rod)

# wrist-pin: SPHERICAL (ball-and-socket) joint rod <-> piston
spherical_rod_piston = chrono.ChLinkLockSpherical()
spherical_rod_piston.Initialize(piston, rod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.AddLink(spherical_rod_piston)

# piston <-> floor: PLANAR (plane-plane) joint constraining piston to move/rotate in the x-y plane
planar_piston_floor = chrono.ChLinkLockPlanar()
planar_piston_floor.Initialize(floor, piston,
                               chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                               chrono.Q_ROTATE_Y_TO_X))   # plane = world x-y
sys.AddLink(planar_piston_floor)

# Irrlicht visualization (Initialize first, then scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Slider-Crank")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # eye, target
vis.AddTypicalLights()

time_step = 1e-3                                                      # integration step
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
