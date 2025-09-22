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
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
# Rotate the cylinder from Y-up to Z-up
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create crank-motor to rotate the crank
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,  # the first connected body
                    mfloor,  # the second connected body
                    chrono.ChFrameD(crank_center))  # motor frame in abs space
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  # angular speed: 180 deg/sec
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create crank-rod joint as a spherical joint (ball-and-socket)
mjointA = chrono.ChLinkLockBall()
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create rod-piston joint as a spherical joint (ball-and-socket)
mjointB = chrono.ChLinkLockBall()
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create piston-floor joint as a planar joint (allows movement in x-y plane)
mjointC = chrono.ChLinkLockPlanePlane()
# Initialize with the piston frame and floor frame
# For plane-plane, need to define the plane frames:
# We'll set the plane in the piston frame with normal along z (since movement is in x-y)
# and in the floor frame aligned with XY plane
piston_frame = chrono.ChFrameD(mpiston.GetPos(), chrono.Q_from_AngY(0))
floor_frame = chrono.ChFrameD(chrono.ChVector3d(0, -0.5, 0), chrono.Q_from_AngY(0))
mjointC.Initialize(mpiston, mfloor, piston_frame, floor_frame)
sys.Add(mjointC)

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Initialize lists to store data for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Run the interactive simulation loop
while vis.Run():
    # Store data for plotting
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    # Render and step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    # Stop after 20 seconds
    if sys.GetChTime() > 20:
        break

# Plot the results
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m]', xlabel='angle [rad]')
ax2.grid()

# Set x-ticks to show \pi fractions
plt.xticks(np.linspace(0, 2 * np.pi, 5),
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])

plt.show()