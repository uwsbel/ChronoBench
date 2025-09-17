import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# chrono.SetChronoDataPath('path/to/data')

system = chrono.ChSystemNSC()  # Changed variable name to avoid conflict

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
system.Add(mfloor)

# Create the flywheel crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate cylinder to horizontal position
system.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
system.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Orient cylinder along X axis
system.Add(mpiston)

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,  # the first connected body
                    mfloor,  # the second connected body
                    chrono.ChFramed(crank_center))  # where to create the motor in abs.space
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # ang.speed: 180°/s
my_motor.SetMotorFunction(my_angularspeed)
system.Add(my_motor)

# Create crank-rod joint (SPHERICAL instead of revolute)
mjointA = chrono.ChLinkLockSpherical()  # Changed to spherical joint
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
system.Add(mjointA)

# Create rod-piston joint (SPHERICAL instead of revolute)
mjointB = chrono.ChLinkLockSpherical()  # Changed to spherical joint
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
system.Add(mjointB)

# Create piston-truss joint (PLANE-PLANE instead of prismatic)
mjointC = chrono.ChLinkLockPlanePlane()  # Changed to plane-plane joint
mjointC.Initialize(mpiston,
                   mfloor,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                   chrono.QUNIT))  # Identity rotation for XY plane constraint
system.Add(mjointC)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
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
    array_time.append(system.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    # here happens the visualization and step time integration
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)

    # stop simulation after 20 seconds
    if system.GetChTime() > 20:
        vis.GetDevice().closeDevice()

# Use matplotlib to make two plots when simulation ended:
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m/s]', xlabel='angle [rad]')  # Fixed unit label
ax2.grid()

# Format x-axis with pi labels
plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.show()  # Added to display plots