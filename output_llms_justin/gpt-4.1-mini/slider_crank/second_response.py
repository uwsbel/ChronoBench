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
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000, True, True)  # Add visualization and collision flags
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetBodyFixed(True)  # Fix the floor so it doesn't move (SetFixed() deprecated)
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
# Correct syntax: ChBodyEasyCylinder(radius, height, density, visualization, collision)
# However, the original code used axes arguments incorrectly for ChBodyEasyCylinder
# The PyChrono API expects: ChBodyEasyCylinder(radius, height, density, visible=True, collide=True)
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, True, True)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
# The original code used a constant Q_ROTATE_Y_TO_Z which is not defined in PyChrono.
# Instead use chrono.Q_from_AngAxis or chrono.Q_ROTATE_Y_TO_Z if it's defined.
# But looking at PyChrono docs, there is chrono.Q_ROTATE_Y_TO_Z.
# Confirm and use if available:
try:
    mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
except AttributeError:
    # If not available, create equivalent rotation
    mcrank.SetRot(chrono.Q_from_AngAxis(np.pi / 2, chrono.VECT_Y))  # rotate 90 deg around Y
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000, True, True)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # Position the rod at (x=-0.4, y=0.5, z=0)
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, True, True)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # Position the piston at (x=0.9, y=0.5, z=0)
# Rotate piston to align with X-axis (original is along Y-axis)
try:
    mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
except AttributeError:
    # Rotate -90 degrees about Z or X to get Y -> X?
    # Actually, Y->X means rotating -90 deg about Z axis:
    mpiston.SetRot(chrono.Q_from_AngAxis(-np.pi / 2, chrono.VECT_Z))
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
# Initialize requires body1, body2 and coordinate system in absolute coordinates
# ChCoordsys initializes with position and rotation
my_motor.Initialize(mcrank, mfloor, chrono.ChCoordsysD(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_C_PI)  # π rad/s constant speed (use CH_C_PI for double precision)
my_motor.SetMotorFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
# Use ChCoordsysD (the constructor requires positional vector and optional quaternion)
mjointA.Initialize(mrod, mcrank, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
try:
    rot_jointC = chrono.Q_ROTATE_Z_TO_X
except AttributeError:
    rot_jointC = chrono.Q_from_AngAxis(-np.pi / 2, chrono.VECT_Y)  # rotate Z to X axis approx.

mjointC.Initialize(mpiston, mfloor, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), rot_jointC))
sys.Add(mjointC)  # Add the joint to the simulation system

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization (1024x768 pixels)
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# === Part 3: Array Initialization for Plotting ===
array_time = []
array_angle = []
array_pos = []
array_speed = []

time_step = 1e-3  # 1 ms time step
max_time = 20.0  # Run for 20 seconds

# Helper function to get crank angle. The crank rotates at constant speed π rad/s.
# We can get rotation from mcrank's rotation quaternion around the axis
# Since crank is rotated to align along Z, its rotation is about Z axis
def get_crank_angle(body):
    # We extract the angle by converting quaternion to angle around Z axis
    # Quaternion to angle axis:
    q = body.GetRot()
    axis = chrono.Vect3d()
    angle = q.Q_to_AngAxis(axis)
    # We expect axis near (0,0,1); we take angle with sign according to rotation axis direction
    if axis.z < 0:
        angle = -angle
    # Normalize angle to [0, 2*pi]
    angle = angle % (2 * np.pi)
    return angle


# Helper function to get piston position along X axis (due to prismatic joint)
def get_piston_pos(body):
    # position of piston center in global coordinates
    pos = body.GetPos()
    return pos.x

# Helper function to get piston speed along X axis
def get_piston_speed(body):
    # linear velocity vector of piston center
    vel = body.GetPos_dt()
    return vel.x


# Run the interactive simulation loop with data collection and stopping condition
t = 0.0
while vis.Run() and t <= max_time:
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene

    # Advance the simulation by a time step
    sys.DoStepDynamics(time_step)
    t += time_step

    # Collect data
    array_time.append(t)
    angle = get_crank_angle(mcrank)
    array_angle.append(angle)
    pos = get_piston_pos(mpiston)
    array_pos.append(pos)
    speed = get_piston_speed(mpiston)
    array_speed.append(speed)

# ================= Plotting =======================
# Convert lists to numpy arrays for convenience
array_time = np.array(array_time)
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)

# Create subplots: Position vs angle and Speed vs angle
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# First subplot: Position [m] vs Crank Angle [rad]
axs[0].plot(array_angle, array_pos, 'b-')
axs[0].set_ylabel('Position [m]')
axs[0].grid(True)
axs[0].set_title('Piston Position vs Crank Angle')

# Second subplot: Speed [m/s] vs Crank Angle [rad]
axs[1].plot(array_angle, array_speed, 'r-')
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].grid(True)
axs[1].set_title('Piston Speed vs Crank Angle')

# Adjust x-axis ticks to display multiples of π/2
# We want ticks at 0, π/2, π, 3π/2, 2π

# Define ticks and labels
ticks = np.array([0, 0.5 * np.pi, np.pi, 1.5 * np.pi, 2 * np.pi])
ticklabels = [r'$0$', r'$\frac{\pi}{2}$', r'$\pi$', r'$\frac{3\pi}{2}$', r'$2\pi$']

for ax in axs:
    ax.set_xticks(ticks)
    ax.set_xticklabels(ticklabels)

plt.tight_layout()
plt.show()