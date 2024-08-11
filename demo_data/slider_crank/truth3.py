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
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # Position the rod at (x=-0.4, y=0.5, z=0)
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # Create a cylinder along the Y-axis with radius 0.2 meters and height 0.3 meters
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # Position the piston at (x=0.9, y=0.5, z=0)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Rotate the piston to align it along the X-axis
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))  # Initialize the motor at the crank center, connecting the crank and the floor
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # Set the angular speed of the motor to π rad/s (180°/s)
my_motor.SetMotorFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a spherical joint to connect the crank to the rod
mjointA = chrono.ChLinkLockSpherical()  # Create a spherical (ball-and-socket) joint
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Initialize the joint at (x=-0.6, y=0.5, z=0)
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a spherical joint to connect the rod to the piston
mjointB = chrono.ChLinkLockSpherical()  # Create a spherical (ball-and-socket) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Initialize the joint at (x=0.9, y=0.5, z=0)
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a plane-plane joint to constrain the piston to move and rotate in the x-y plane
plane_plane = chrono.ChLinkLockPlanar()  # Create a plane-plane (planar) joint
plane_plane.Initialize(mfloor, mpiston, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Y_TO_X))  # Initialize the joint at (x=0.9, y=0.5, z=0) with no rotation
sys.Add(plane_plane)  # Add the joint to the simulation system

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization (1024x768 pixels)
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # Add a camera to the visualization, positioned at (x=1, y=1, z=3) and looking at (x=0, y=1, z=0)
vis.AddTypicalLights()  # Add typical lights for better visualization

# Initialize lists to store simulation data for plotting
array_time = []  # List to store simulation time
array_angle = []  # List to store motor angle
array_pos = []  # List to store piston position
array_speed = []  # List to store piston speed

# Run the interactive simulation loop
while vis.Run():
    # Collect simulation data for plotting
    array_time.append(sys.GetChTime())  # Append the current simulation time to the list
    array_angle.append(my_motor.GetMotorAngle())  # Append the current motor angle to the list (in radians)
    array_pos.append(mpiston.GetPos().x)  # Append the current X position of the piston to the list
    array_speed.append(mpiston.GetPosDt().x)  # Append the current X speed of the piston to the list

    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds (1 ms)

    # Stop the simulation after 20 seconds
    if sys.GetChTime() > 20:
        vis.GetDevice().closeDevice()  # Close the visualization device

# Use matplotlib to plot the simulation results
fig, (ax1, ax2) = plt.subplots(2, sharex=True)  # Create a figure with two subplots sharing the x-axis

# Plot piston position vs. motor angle
ax1.plot(array_angle, array_pos)  # Plot piston position (y-axis) against motor angle (x-axis)
ax1.set(ylabel='Position [m]')  # Set y-axis label
ax1.grid()  # Enable grid

# Plot piston speed vs. motor angle
ax2.plot(array_angle, array_speed, 'r--')  # Plot piston speed (y-axis) against motor angle (x-axis) with red dashed line
ax2.set(ylabel='Speed [m/s]', xlabel='Angle [rad]')  # Set y-axis and x-axis labels
ax2.grid()  # Enable grid

# Customize x-axis ticks to show π intervals
plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])  # Set x-axis ticks to show 0, π/2, π, 3π/2, 2π

# Show the plots
plt.show()  # Display the plots
