import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# Create the floor (truss) body
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the crank body
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

# Create and initialize the motor
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunctionConst(np.pi)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create revolute joints with corrected ChFrameD
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create prismatic joint with corrected rotation (QUNIT)
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(
    crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
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

# Data collection arrays
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Perform dynamics step
    sys.DoStepDynamics(1e-3)
    
    # Get current simulation time
    current_time = sys.GetChTime()
    
    # Stop simulation after 20 seconds
    if current_time >= 20:
        vis.GetDevice().closeDevice()
        break
    
    # Collect data
    array_time.append(current_time)
    array_angle.append(np.pi * current_time)  # Motor angle = π * time
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetLinVel().x)

# Create plots with π-based x-axis
x_ticks = np.arange(0, 2*np.pi + np.pi/2, np.pi/2)
x_labels = ['0', 'π/2', 'π', '3π/2', '2π']

plt.figure(figsize=(12, 6))

# Position vs Angle plot
plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos)
plt.xticks(x_ticks, x_labels)
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.title('Piston Position vs Crank Angle')
plt.grid(True)

# Speed vs Angle plot
plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, color='orange')
plt.xticks(x_ticks, x_labels)
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.title('Piston Speed vs Crank Angle')
plt.grid(True)

plt.tight_layout()
plt.show()