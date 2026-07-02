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

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)

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

# Initialize arrays for data collection
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Run the interactive simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    # Collect data
    current_time = sys.GetChTime()
    array_time.append(current_time)
    array_angle.append(my_motor.GetAngle())
    piston_pos = mpiston.GetPos().x
    array_pos.append(piston_pos)
    piston_speed = mpiston.GetPosDt().x
    array_speed.append(piston_speed)
    
    # Stop simulation after 20 seconds
    if current_time >= 20.0:
        break

# Create plots using matplotlib
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# First subplot: Position vs. crank angle
ax1.plot(array_angle, array_pos, label='Piston Position', color='blue')
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.grid(True)
ax1.set_title('Piston Position vs Crank Angle')

# Second subplot: Speed vs. crank angle
ax2.plot(array_angle, array_speed, label='Piston Speed', color='red')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.grid(True)
ax2.set_title('Piston Speed vs Crank Angle')

# Adjust x-axis ticks to π-based intervals
max_angle = max(array_angle)
xticks = np.arange(0, max_angle + 0.1, np.pi/2)
xtick_labels = [f'{int(t/np.pi)}π' if t % np.pi == 0 else f'{t/np.pi:.1f}π' for t in xticks]
ax1.set_xticks(xticks)
ax1.set_xticklabels(xtick_labels)
ax2.set_xticks(xticks)
ax2.set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()