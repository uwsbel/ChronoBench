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
# ChBodyEasyCylinder expects axis as ChVector3d
mcrank = chrono.ChBodyEasyCylinder(chrono.ChVector3d(0, 1, 0), crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
# Rotate cylinder from Y to Z axis (so axis is along Z)
mcrank.SetRot(chrono.Q_from_AngAxis(np.pi/2, chrono.ChVector3d(1, 0, 0)))
sys.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChVector3d(0, 1, 0), 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
# Rotate cylinder from Y to X axis (so axis is along X)
mpiston.SetRot(chrono.Q_from_AngAxis(-np.pi/2, chrono.ChVector3d(0, 0, 1)))
sys.Add(mpiston)

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank,  # first body
    mfloor,  # second body
    chrono.ChFrameD(crank_center)
)
my_angularspeed = chrono.ChFunctionConst(np.pi)  # ang.speed: 180°/s
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create crank-rod joint (MODIFIED: Spherical joint)
mjointA = chrono.ChLinkLockSpherical()
mjointA.Initialize(
    mrod,
    mcrank,
    chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0))
)
sys.Add(mjointA)

# Create rod-piston joint (MODIFIED: Spherical joint)
mjointB = chrono.ChLinkLockSpherical()
mjointB.Initialize(
    mpiston,
    mrod,
    chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
)
sys.Add(mjointB)

# Create piston-truss joint (MODIFIED: Planar joint in x-y)
# The planar joint will constrain the piston to move and rotate in the x-y plane.
# The plane is defined by origin and normal (z axis)
plane_pos = crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)
plane_rot = chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 0, 1))  # No rotation, plane is x-y

mjointC = chrono.ChLinkLockPlanar()
mjointC.Initialize(
    mpiston,
    mfloor,
    chrono.ChFrameD(plane_pos, plane_rot)
)
sys.Add(mjointC)

# Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Initialize these lists to store values to plot.
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Run the interactive simulation loop
while vis.Run():
    # for plotting, append instantaneous values:
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    # here happens the visualization and step time integration
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    # stop simulation after 2 seconds
    if sys.GetChTime() > 2:
        vis.GetDevice().closeDevice()

# Use matplotlib to make two plots when simulation ended:
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m]', xlabel='angle [rad]')
ax2.grid()

# trick to plot \pi on x axis of plots instead of 1 2 3 4 etc.
plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])

plt.show()