import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Mechanism dimensions
crank_len = 0.5          # half-length of crank arm [m]
rod_len = 1.2            # connecting rod length [m]
piston_radius = 0.15     # piston cylinder radius [m]
piston_height = 0.3      # piston height [m]
crank_radius = 0.08      # crank cylinder radius [m]
rod_radius = 0.05        # connecting rod radius [m]

# Simulation parameters
time_step = 1e-3         # physics time step [s]
sim_end = 10.0           # end time [s]
render_fps = 50.0        # render target FPS
motor_speed = chrono.CH_PI  # crank angular speed [rad/s] (~180 deg/s)

# Create NSC system — slider-crank is a pure jointed MBS (no contact needed)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up, gravity down

# -- Floor / truss body (fixed) --
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
sys.AddBody(floor)

floor_shape = chrono.ChVisualShapeBox(3.0, 0.1, 1.0)  # wide floor slab
floor_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
floor.AddVisualShape(floor_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.5, 0)))

# -- Crank body --
# Crank spins about the origin; body COM is at the midpoint of the crank arm
crank_pos = chrono.ChVector3d(0, 0, 0)  # crank pivot at origin; COM at crank end
crank = chrono.ChBody()
crank.SetMass(0.5)
crank.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
crank.SetPos(chrono.ChVector3d(crank_len * 0.5, 0, 0))  # COM at midpoint along X
sys.AddBody(crank)

crank_shape = chrono.ChVisualShapeCylinder(crank_radius, crank_len)
crank.AddVisualShape(
    crank_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))  # Z->X
)

# -- Connecting rod body --
# Rod runs from crank pin (at x=crank_len) to piston wrist pin (at x=crank_len+rod_len)
rod_pin_pos = chrono.ChVector3d(crank_len, 0, 0)   # crank-pin world position
piston_pin_x = crank_len + rod_len                  # wrist-pin world X
rod_center = chrono.ChVector3d((crank_len + piston_pin_x) * 0.5, 0, 0)

rod = chrono.ChBody()
rod.SetMass(0.3)
rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
rod.SetPos(rod_center)
sys.AddBody(rod)

rod_shape = chrono.ChVisualShapeCylinder(rod_radius, rod_len)
rod.AddVisualShape(
    rod_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))  # Z->X
)

# -- Piston body --
piston_x = piston_pin_x                             # piston COM starts at rod far end
piston = chrono.ChBody()
piston.SetMass(0.4)
piston.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
piston.SetPos(chrono.ChVector3d(piston_x, 0, 0))
sys.AddBody(piston)

piston_shape = chrono.ChVisualShapeCylinder(piston_radius, piston_height)
piston.AddVisualShape(
    piston_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))  # Z->X, axis along slide
)

# -- Motor: drives crank relative to floor (ChLinkMotorRotationSpeed is a full motor-link) --
# Hinge at world origin; crank spins about Z (Y-up, motion in XY plane → hinge axis = Z)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunctionConst(motor_speed))  # constant angular speed
sys.AddLink(motor)

# -- Revolute: crank pin connects crank to rod --
# Pivot at crank far end (local +X/2 from crank COM = 0.5*crank_len offset)
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(crank_len * 0.5, 0, 0), chrono.QUNIT),   # crank local: far end
    chrono.ChFramed(chrono.ChVector3d(-rod_len * 0.5, 0, 0), chrono.QUNIT),    # rod local: near end
)
sys.AddLink(joint_crank_rod)

# -- Revolute: wrist pin connects rod to piston --
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(rod_len * 0.5, 0, 0), chrono.QUNIT),     # rod local: far end
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),                 # piston local: COM
)
sys.AddLink(joint_rod_piston)

# -- Prismatic: piston slides along X on the fixed floor --
# Guide axis = world X; ChLinkLockPrismatic uses frame local +Z as slide axis;
# Q_ROTATE_Z_TO_X maps local +Z onto world +X
joint_piston_floor = chrono.ChLinkLockPrismatic()
joint_piston_floor.Initialize(
    piston, floor,
    chrono.ChFramed(chrono.ChVector3d(piston_x, 0, 0), chrono.Q_ROTATE_Z_TO_X)
)
sys.AddLink(joint_piston_floor)

# -- Irrlicht visualization (Initialize FIRST, then add scene elements) --
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)      # Y-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Slider-Crank Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # Chrono logo
vis.AddSkyBox()                                        # sky texture
vis.AddCamera(
    chrono.ChVector3d(2.0, 1.5, 3.0),                 # eye position
    chrono.ChVector3d(1.0, 0.0, 0.0)                  # look at crank region
)
vis.AddTypicalLights()                                 # default two-light setup

# Render cadence constant (untagged — real-time loop needs it)
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per frame


# Main simulation loop
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
