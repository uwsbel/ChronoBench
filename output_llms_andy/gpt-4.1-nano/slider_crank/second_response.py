import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import numpy as np  # Import numpy for numerical operations

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # Create a box with dimensions 3x1x3 meters and density 1000 kg/m^3
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Cylinder along Y-axis
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank
# Correct rotation: rotate from Y to Z
mcrank.SetRot(chrono.Q_from_AngZ(0))  # Initially no rotation; will rotate via motor
sys.Add(mcrank)  # Add the crank to the system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box for the connecting rod
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
# Rotate piston to align along X
mpiston.SetRot(chrono.Q_from_AngX(0))
sys.Add(mpiston)

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  # Use CH_C_PI for consistency
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Revolute joint: crank to rod
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Revolute joint: rod to piston
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Prismatic joint: piston to floor (along X axis)
mjointC = chrono.ChLinkLockPrismatic()
# The frame for prismatic joint: position at piston, axis along X
frame_for_prismatic = chrono.ChFrameD(
    chrono.ChVector3d(crank_rad + rod_length, 0, 0),
    chrono.Q_from_AngX(0)
)
mjointC.Initialize(mpiston, mfloor, frame_for_prismatic)
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
# Camera positioned at (1,1,3), looking at (0,1,0)
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Simulation loop with stop condition at 20 seconds
time_end = 20.0
dt = 1e-3
time = 0.0

while vis.Run():
    # Record data at each step
    # Get crank angle (rotation around Z axis)
    crank_rot = mcrank.GetRot()
    # Convert quaternion to Euler angle (rotation around Z)
    # Since we rotate the crank around Z, extract that angle
    angle_z = crank_rot.Get_AngleZ()
    # Get crank position
    crank_pos = mcrank.GetPos()
    # Get crank linear speed along the path (approximate)
    crank_speed = mcrank.GetPos_dt().Length()

    # Append data
    array_time.append(time)
    array_angle.append(angle_z)
    array_pos.append(crank_pos.x)
    array_speed.append(crank_speed)

    # Visualization rendering
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Step the simulation
    sys.DoStepDynamics(dt)
    time += dt

    # Stop after 20 seconds
    if time >= time_end:
        break

# Plotting after simulation
# Convert lists to numpy arrays for convenience
angle_array = np.array(array_angle)
pos_array = np.array(array_pos)
speed_array = np.array(array_speed)

# Plot position vs. crank angle
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(angle_array, pos_array)
plt.xlabel('Crank angle [rad]')
plt.ylabel('Position [m]')
plt.title('Crank Position vs. Angle')
plt.grid(True)

# Plot speed vs. crank angle
plt.subplot(2, 1, 2)
plt.plot(angle_array, speed_array)
plt.xlabel('Crank angle [rad]')
plt.ylabel('Speed [m/s]')
plt.title('Crank Speed vs. Angle')
plt.grid(True)

# Adjust x-axis ticks to display multiples of π/2
ticks = np.array([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
tick_labels = ['0', 'π/2', 'π', '3π/2', '2π']
for ax in plt.gcf().axes:
    ax.set_xticks(ticks)
    ax.set_xticklabels(tick_labels)

plt.tight_layout()
plt.show()