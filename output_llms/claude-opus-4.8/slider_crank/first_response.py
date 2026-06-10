import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# ---------------------------------------------------------------------

# Create the physical system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# A material for collision/contact (not strictly needed here, but defined)
contact_material = chrono.ChContactMaterialNSC()

# ---------------------------------------------------------------------
# 2. Add the required physical systems and objects
# ---------------------------------------------------------------------

# ----- FLOOR / TRUSS -----
# The truss is the fixed reference frame for the mechanism
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000, True, False)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
# Apply a texture to the floor
mfloor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(mfloor)

# ----- CRANKSHAFT -----
# The crankshaft is the rotating element driven by the motor
mcrank = chrono.ChBodyEasyBox(1.5, 0.5, 0.5, 1000, True, False)
mcrank.SetPos(chrono.ChVector3d(1, 0, 0))
mcrank.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))
sys.Add(mcrank)

# ----- CONNECTING ROD -----
# The rod connects the crank end to the piston
mrod = chrono.ChBodyEasyBox(4, 0.3, 0.3, 1000, True, False)
mrod.SetPos(chrono.ChVector3d(4, 0, 0))
mrod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.2))
sys.Add(mrod)

# ----- PISTON -----
# The piston slides back and forth along the X axis
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.4, 0.5, 1000, True, False)
mpiston.SetPos(chrono.ChVector3d(6, 0, 0))
mpiston.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))  # orient cylinder along X
mpiston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.6))
sys.Add(mpiston)

# ---------------------------------------------------------------------
# 3. Connect bodies with joints and drive with a motor
# ---------------------------------------------------------------------

# ----- MOTOR: crank <-> truss -----
# Spin the crankshaft at constant angular speed about the Z axis
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank,
    mfloor,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
# Constant angular speed: pi rad/s (half revolution per second)
const_speed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetSpeedFunction(const_speed)
sys.Add(my_motor)

# ----- REVOLUTE JOINT: crank <-> rod -----
# Located at the outer end of the crank
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(
    mcrank,
    mrod,
    chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.QUNIT)
)
sys.Add(joint_crank_rod)

# ----- SPHERICAL/REVOLUTE JOINT: rod <-> piston -----
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(
    mpiston,
    mrod,
    chrono.ChFramed(chrono.ChVector3d(6, 0, 0), chrono.QUNIT)
)
sys.Add(joint_rod_piston)

# ----- PRISMATIC JOINT: piston <-> truss -----
# Constrains the piston to slide along the X axis
joint_piston_truss = chrono.ChLinkLockPrismatic()
# A prismatic's default sliding axis is Z, so rotate to align with X
joint_piston_truss.Initialize(
    mpiston,
    mfloor,
    chrono.ChFramed(chrono.ChVector3d(6, 0, 0),
                    chrono.QuatFromAngleY(chrono.CH_PI_2))
)
sys.Add(joint_piston_truss)

# ---------------------------------------------------------------------
# 4. Set up Irrlicht visualization
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Demo')
vis.Initialize()

# Add a logo
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

# Add a sky-box background
vis.AddSkyBox()

# Set up the camera
vis.AddCamera(chrono.ChVector3d(3, 4, -6), chrono.ChVector3d(3, 0, 0))

# Add lights
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(5, 8, -5), 12,
             chrono.ChColor(0.8, 0.8, 0.9))

# ---------------------------------------------------------------------
# 5. Run the simulation loop
# ---------------------------------------------------------------------

time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()

    # Draw a reference grid on the floor
    chronoirr.drawGrid(vis, 0.5, 0.5, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0.01, 0),
                                          chrono.QuatFromAngleX(chrono.CH_PI_2)),
                       chrono.ChColor(0.4, 0.4, 0.4), True)

    vis.EndScene()

    # Advance the simulation by one time step
    sys.DoStepDynamics(time_step)