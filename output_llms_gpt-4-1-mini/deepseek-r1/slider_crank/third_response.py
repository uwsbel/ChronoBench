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
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate cylinder axis from Y to Z
sys.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Orient piston cylinder along X
sys.Add(mpiston)

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, 
                    mfloor, 
                    chrono.ChFrameD(crank_center))  # Corrected to ChFrameD
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # 180°/s angular speed
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create crank-rod spherical joint
mjointA = chrono.ChLinkLockSpherical()  # Changed to spherical
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create rod-piston spherical joint
mjointB = chrono.ChLinkLockSpherical()  # Changed to spherical
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create piston-truss planar joint (replaces prismatic)
mjointC = chrono.ChLinkLockPlanePlane()  # Changed to plane-plane
mjointC.Initialize(mpiston,
                   mfloor,
                   chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                   chrono.QUNIT))  # Identity rotation for x-y plane
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

# Data collection for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation loop
while vis.Run():
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    if sys.GetChTime() > 2.0:  # Corrected to 2 seconds
        vis.GetDevice().closeDevice()

# Plotting
fig, (ax1, ax2) = plt.subplots(2, sharex=True)
ax1.plot(array_angle, array_pos)
ax1.set(ylabel='Position [m]', title='Piston Motion')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='Speed [m/s]', xlabel='Crank Angle [rad]')
ax2.grid()

plt.xticks(np.linspace(0, 2 * np.pi, 5), 
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.show()