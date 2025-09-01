import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# chrono.SetChronoDataPath('path/to/data') # Optional: set if CHRONO_DATA_DIR is not configured

# System setup
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0)) # Set gravity

# Some data shared in the following
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1 # This is the height of the cylinder along its axis
rod_length = 1.5

# Create four rigid bodies: the truss, the crank, the rod, the piston.

# Create the floor truss
mfloor = chrono.ChBodyEasyBox(4, 1, 4, 1000) # Slightly wider for planar motion
mfloor.SetPos(chrono.ChVector3d(0, -0.5 + crank_center.y, 0)) # Adjust floor if mechanism y != 0
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the flywheel crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center) # Crank CoG at motor location
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z) # Rotate to be a disk in XY plane, rotating about Z
sys.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
rod_initial_pos_x = crank_center.x + crank_rad + rod_length / 2
mrod.SetPos(chrono.ChVector3d(rod_initial_pos_x, crank_center.y, crank_center.z))
sys.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000) # radius 0.2, height 0.3
piston_initial_pos = chrono.ChVector3d(crank_center.x + crank_rad + rod_length, crank_center.y, crank_center.z)
mpiston.SetPos(piston_initial_pos)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X) # Piston cylinder axis along global X
sys.Add(mpiston)

# --- Joint Definitions ---

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,
                    mfloor,
                    chrono.ChFramed(crank_center, chrono.QUNIT)) # Motor at crank_center, rot Z-axis
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # ang.speed: pi rad/s (180°/s)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# MODIFICATION 1: Crank-Rod joint from Revolute to Spherical
mjointA = chrono.ChLinkLockSpherical()
crank_pin_abs_pos = chrono.ChVector3d(crank_center.x + crank_rad, crank_center.y, crank_center.z)
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChCoordsysd(crank_pin_abs_pos)) # Joint at absolute position
sys.Add(mjointA)

# MODIFICATION 2: Rod-Piston joint from Revolute to Spherical
mjointB = chrono.ChLinkLockSpherical()
# piston_pin_abs_pos is the same as piston_initial_pos if joint is at piston CoG
piston_pin_abs_pos = piston_initial_pos
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChCoordsysd(piston_pin_abs_pos)) # Joint at absolute position
sys.Add(mjointB)

# MODIFICATION 3: Piston-Truss joint from Prismatic to Planar
mjointC = chrono.ChLinkLockPlanar()
# Planar joint defined by an absolute frame. Piston moves in XY plane of this frame.
# Z-axis of frame is normal to plane of motion. For motion in global XY plane, Z is global Z.
planar_joint_frame_abs = chrono.ChFramed(piston_initial_pos, chrono.QUNIT)
mjointC.Initialize(mpiston,
                   mfloor,
                   planar_joint_frame_abs)
sys.Add(mjointC)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo - Modified Joints')
vis.Initialize()
try:
    vis.AddLogo(chrono.GetChrono
print("error happened with only start ```python")