import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Set Chrono data path if needed
# chrono.SetChronoDataPath('path/to/data')

sys = chrono.ChSystemNSC()

# Some data shared in the following
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# Create four rigid bodies: the truss, the crank, the rod, the piston

# Create the floor truss
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the flywheel crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
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

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,  # the first connected body
                    mfloor,  # the second connected body
                    chrono.ChFramed(crank_center))  # where to create the motor in abs.space
my_angularspeed = chrono.ChFunction_Const(chrono.CH_PI)  # ang.speed: 180°/s
my_motor.SetSpeedFunction(my_angularspeed)
sys.Add(my_motor)

# Create crank-rod joint (spherical joint)
mjointA = chrono.ChLinkLockSpherical()
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create rod-piston joint (spherical joint)
mjointB = chrono.ChLinkLockSpherical()
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create piston-truss joint (planar joint)
mjointC = chrono.ChLinkLockPlane()
mjointC.Initialize(mpiston,
                   mfloor,
                   chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                   chrono.QUNIT))
sys.Add(mjointC)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Initialize data arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation loop
while vis.Run():
    # Record data points
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    
    # Update visualization and simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    # Terminate simulation after 2 full rotations (4π radians)
    if sys.GetChTime() > 4 * np.pi:
        vis.GetDevice().closeDevice()

# Plot results
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='Position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='Speed [m/s]', xlabel='Angle [rad]')
ax2.grid()

# Format x-axis with π multiples
plt.xticks(np.linspace(0, 4*np.pi, 5), ['0', r'$\pi$', r'$2\pi$', r'$3\pi$', r'$4\pi$'])

plt.show()