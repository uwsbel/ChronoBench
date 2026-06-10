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
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Cylinder along Y-axis, radius 0.4 m, thickness 0.1 m
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # Position the rod at (x=-0.4, y=0.5, z=0)
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # Cylinder along Y-axis, radius 0.2 m, height 0.3 m
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # Position the piston at (x=0.9, y=0.5, z=0)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Rotate the piston to align it along the X-axis
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))  # Initialize the motor at the crank center
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # Set the angular speed of the motor to π rad/s
my_motor.SetMotorFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Initialize at (x=-0.6, y=0.5, z=0)
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Initialize at (x=0.9, y=0.5, z=0)
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # Initialize at (x=0.9, y=0.5, z=0), Z aligned to X
sys.Add(mjointC)  # Add the joint to the simulation system

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization (1024x768 pixels)
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # Add a camera positioned at (1,1,3) looking at (0,1,0)
vis.AddTypicalLights()  # Add typical lights for better visualization

# -----------------------------------------------------------------------------
# 3. Array Initialization for Plotting
# -----------------------------------------------------------------------------
array_time = []   # Stores simulation time [s]
array_angle = []  # Stores the crank angle [rad]
array_pos = []    # Stores the piston position [m]
array_speed = []  # Stores the piston speed [m/s]

# -----------------------------------------------------------------------------
# Run the interactive simulation loop
# -----------------------------------------------------------------------------
while vis.Run():
    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()      # Render the scene
    vis.EndScene()    # End the visualization scene

    # 4. Collect data during simulation
    array_time.append(sys.GetChTime())                  # Current simulation time
    array_angle.append(my_motor.GetMotorAngle())        # Crank angle from the motor
    array_pos.append(mpiston.GetPos().x)                # Piston position along X
    array_speed.append(mpiston.GetPosDt().x)            # Piston speed along X

    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds (1 ms)

    # 5. Conditional to stop the simulation after 20 seconds
    if sys.GetChTime() > 20:
        break

# -----------------------------------------------------------------------------
# 6. Matplotlib plotting
# -----------------------------------------------------------------------------
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)

# Build π-based ticks spanning the recorded angle range
max_angle = array_angle[-1] if len(array_angle) > 0 else 2 * np.pi
ticks = np.arange(0, max_angle + np.pi / 2, np.pi / 2)


def pi_formatter(value):
    """Return a π-based label for a given tick value."""
    n = int(round(value / (np.pi / 2)))
    if n == 0:
        return "0"
    elif n == 1:
        return r"$\pi/2$"
    elif n == 2:
        return r"$\pi$"
    elif n % 2 == 0:
        return r"${}\pi$".format(n // 2)
    else:
        return r"$\frac{{{}\pi}}{{2}}$".format(n)


tick_labels = [pi_formatter(t) for t in ticks]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# First subplot: position vs crank angle
ax1.plot(array_angle, array_pos, color='tab:blue')
ax1.set_xlabel('Crank angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.set_title('Piston position vs Crank angle')
ax1.set_xticks(ticks)
ax1.set_xticklabels(tick_labels)
ax1.grid(True)

# Second subplot: speed vs crank angle
ax2.plot(array_angle, array_speed, color='tab:red')
ax2.set_xlabel('Crank angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.set_title('Piston speed vs Crank angle')
ax2.set_xticks(ticks)
ax2.set_xticklabels(tick_labels)
ax2.grid(True)

plt.tight_layout()
plt.show()