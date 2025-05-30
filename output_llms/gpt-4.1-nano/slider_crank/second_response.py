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
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Along Y-axis
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank
# Corrected rotation: instead of incorrect attribute, use a quaternion for rotation
# Rotate the crank to align it along the Z-axis
mcrank.SetRot(chrono.Q_from_AngY(chrono.C_PI / 2))
sys.Add(mcrank)

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Dimensions: length x 0.1 x 0.1
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
# Corrected rotation: align piston along X axis
mpiston.SetRot(chrono.Q_from_AngX(chrono.C_PI / 2))
sys.Add(mpiston)

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.C_PI)  # π rad/s
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

# Prismatic joint: piston to floor (slide along X axis)
# Rotation quaternion to align axis along X; assuming the joint at piston center
mjointC = chrono.ChLinkLockPrismatic()
joint_frame = chrono.ChFrameD(
    chrono.ChVector3d(crank_rad + rod_length, 0, 0),
    chrono.Q_from_AngX(0)  # No rotation needed; axis is along X by default
)
mjointC.Initialize(mpiston, mfloor, joint_frame)
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

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation parameters
t_end = 20.0  # seconds
dt = 1e-3  # time step

# Run the simulation with data collection
time = 0.0
while vis.Run():
    # Collect data at each step
    # Get the crank angle (rotation around Y axis)
    crank_rot = mcrank.GetRot()
    # Calculate the angle of the crank around its axis (assuming rotation about Y)
    # Using GetRot() to extract angle about Y
    # Alternatively, if the rotation is known, use GetRot().AngleY()
    # but that method isn't available. Instead, use quaternion to get the angle around Y
    # For simplicity, assuming rotation about Y, extract the angle:
    angle = 2 * np.arccos(crank_rot.w)
    # Determine the sense of the angle
    if crank_rot.y < 0:
        angle = -angle

    # Approximate the position of piston along X (assuming linear motion along X)
    piston_pos_x = mpiston.GetPos().x()

    # Piston velocity along X can be obtained from GetPos_dt()
    piston_vel_x = mpiston.GetPos_dt().x()

    # Append data
    array_time.append(time)
    array_angle.append(angle)
    array_pos.append(piston_pos_x)
    array_speed.append(piston_vel_x)

    # Check for end time
    if time >= t_end:
        break

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(dt)
    time += dt

# Plotting after simulation ends
# Convert lists to numpy arrays for convenience
array_time_np = np.array(array_time)
array_angle_np = np.array(array_angle)
array_pos_np = np.array(array_pos)
array_speed_np = np.array(array_speed)

# Create subplots
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# First plot: position vs. crank angle
axs[0]..plot(array_angle_np, array_pos_np)
axs[0].set_xlabel('Crank angle [rad]')
axs[0].set_ylabel('Position [m]')
axs[0].set_title('Piston position vs. crank angle')
axs[0].grid()

# Second plot: speed vs. crank angle
axs[1].plot(array_angle_np, array_speed_np)
axs[1].set_xlabel('Crank angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].set_title('Piston speed vs. crank angle')
axs[1].grid()

# Adjust x-axis ticks to show π intervals
ticks = np.array([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
tick_labels = ['0', 'π/2', 'π', '3π/2', '2π']

for ax in axs:
    ax.set_xticks(ticks)
    ax.set_xticklabels(tick_labels)

plt.tight_layout()
plt.show()