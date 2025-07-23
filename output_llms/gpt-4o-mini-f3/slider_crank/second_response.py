import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Some data shared among objects
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# Create the fixed truss
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create the connecting rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create the piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create the motor to spin the crank
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # Set angular speed to π rad/s
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create a revolute joint between the crank and the rod
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create a revolute joint between the rod and the piston
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create a prismatic joint between the piston and the floor
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Run the interactive simulation loop
time=0
while vis.Run():
    time= sys.GetChTime()
    # Append values for plotting
    array_time.append(time)
    array_angle.append(mcrank.GetWfromQ().y)
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)
    
    # Check if simulation time exceeds 20 seconds
    if time > 20:
        vis.GetDevice().closeDevice()
    
    # Visualization and time step integration
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

# Use matplotlib to create two plots after the simulation
fig, (ax1, ax2) = plt.subplots(2, sharex=True)

# Plot piston position vs. crank angle
ax1.plot(array_angle, array_pos)
ax1.set_ylabel('Position [m]')
ax1.set_title('Piston Position vs. Crank Angle')

# Plot piston speed vs. crank angle
ax2.plot(array_angle, array_speed, 'r--')
ax2.set_ylabel('Speed [m/s]')
ax2.set_title('Piston Speed vs. Crank Angle')

# Set x-axis ticks to π intervals
plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])

# Show the plots
plt.show()