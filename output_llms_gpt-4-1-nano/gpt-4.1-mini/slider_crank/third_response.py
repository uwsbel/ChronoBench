import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# chrono.SetChronoDataPath('path/to/data')

sys = chrono.ChSystemNSC()

# Some data shared in the following
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# Create four rigid bodies: the truss, the crank, the rod, the piston.

# Create the floor truss
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the flywheel crank
# Corrected the arguments of ChBodyEasyCylinder to match (radius, height, density)
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
# Rotate the crank from Y-axis aligned to Z-axis aligned cylinder:
# Q_ROTATE_Y_TO_Z rotates cylinder axis from Y to Z
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create a stylized piston
# Correct argument order for ChBodyEasyCylinder(radius, height, density)
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank,  # the first connected body
    mfloor,  # the second connected body
    chrono.ChFrameD(crank_center)  # where to create the motor in abs.space. Use ChFrameD not ChFramed.
)
my_angularspeed = chrono.ChFunctionConst(chrono.CH_C_PI)  # angular speed: 180°/s
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# --- Modify joints according to instructions ---

# Create crank-rod joint: change from revolute (hinge) to spherical (ball and socket)
mjointA = chrono.ChLinkLockSpherical()
mjointA.Initialize(
    mrod,
    mcrank,
    chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0))
)
sys.Add(mjointA)

# Create rod-piston joint: change from revolute (hinge) to spherical (ball and socket)
mjointB = chrono.ChLinkLockSpherical()
mjointB.Initialize(
    mpiston,
    mrod,
    chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
)
sys.Add(mjointB)

# Create piston-truss joint: replace prismatic joint with planar (plane-plane) joint
# The planar joint allows translation and rotation in the plane defined by two orthonormal axes.
# We want movement in x-y plane; z axis is axis of constraint.
# Create a joint that allows 3 DOF in plane: translation along 2 axes + rotation about normal to plane.
# chrono.ChLinkLockPlanePlane() is the planar joint.

# The reference frame here needs to define the plane:
# Position is at piston position, axes define plane x and y.
pivot = crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)
# Define a frame where x and y form the plane, z normal is out of plane.
# Use identity rotation (default) because default axes are x, y, z

mjointC = chrono.ChLinkLockPlanePlane()
mjointC.Initialize(
    mpiston,
    mfloor,
    chrono.ChFrameD(pivot)
)
sys.Add(mjointC)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo - modified joints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Initialize lists to store values to plot
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Run the interactive simulation loop
while vis.Run():
    # Append instantaneous values
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    # Visualization and time integration step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    # Stop simulation after 20 seconds
    if sys.GetChTime() > 20:
        vis.GetDevice().closeDevice()

# Plot results with matplotlib
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m/s]', xlabel='angle [rad]')
ax2.grid()

# Set x ticks to show multiples of pi
plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])

plt.show()