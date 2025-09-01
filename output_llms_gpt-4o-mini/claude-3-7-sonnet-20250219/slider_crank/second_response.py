import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import numpy as np  # Import numpy for numerical operations

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVectorD(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # Create a box with dimensions 3x1x3 meters and density 1000 kg/m^3
mfloor.SetPos(chrono.ChVectorD(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetBodyFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, chrono.ChVectorD(0, 1, 0))  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, 0))  # Position the crank at (x=-1, y=0.5, z=0)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))  # Position the rod at (x=-0.4, y=0.5, z=0)
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, chrono.ChVectorD(1, 0, 0))  # Create a cylinder along the X-axis with radius 0.2 meters and height 0.3 meters
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))  # Position the piston at (x=0.9, y=0.5, z=0)
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
my_motor.Initialize(mcrank, mfloor, chrono.ChFrame(crank_center))  # Initialize the motor at the crank center, connecting the crank and the floor
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  # Set the angular speed of the motor to π rad/s (180°/s)
my_motor.SetSpeedFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointA.Initialize(mrod, mcrank, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))  # Initialize the joint at (x=-0.6, y=0.5, z=0)
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))  # Initialize the joint at (x=0.9, y=0.5, z=0)
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # Initialize the joint at (x=0.9, y=0.5, z=0) with rotation aligning the Z-axis to the X-axis
sys.Add(mjointC)  # Add the joint to the simulation system

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization (1024x768 pixels)
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))  # Add a camera to the visualization, positioned at (x=1, y=1, z=3) and looking at (x=0, y=1, z=0)
vis.AddTypicalLights()  # Add typical lights for better visualization

# Initialize arrays for storing values to be plotted
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Set the simulation end time
sim_end_time = 20.0  # seconds
sim_time = 0.0
time_step = 1e-3  # Time step size (0.001 seconds)

# Run the interactive simulation loop
while vis.Run() and sim_time < sim_end_time:
    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    
    # Step the simulation
    sys.DoStepDynamics(time_step)  # Advance the simulation by a time step
    sim_time += time_step
    
    # Collect data for plotting
    crank_angle = mcrank.GetRot().Q_to_Euler123().y  # Get the crank angle (y component of Euler angles)
    piston_pos = mpiston.GetPos().x  # Get the piston position (x component)
    piston_speed = mpiston.GetPos_dt().x  # Get the piston speed (x component of velocity)
    
    # Append data to arrays
    array_time.append(sim_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)

# Convert lists to numpy arrays for easier manipulation
array_time = np.array(array_time)
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)

# Create plots using matplotlib
plt.figure(figsize=(12, 10))

# First subplot: Position vs Angle
plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, 'b-')
plt.title('Piston Position vs Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')

# Set x-axis ticks to display π-based intervals
plt.xticks(np.arange(0, 2*np.pi+0.1, np.pi/2), 
           ['0', 'π/2', 'π', '3π/2', '2π'])

# Second subplot: Speed vs Angle
plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, 'r-')
plt.title('Piston Speed vs Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')

# Set x-axis ticks to display π-based intervals
plt.xticks(np.arange(0, 2*np.pi+0.1, np.pi/2), 
           ['0', 'π/2', 'π', '3π/2', '2π'])

plt.tight_layout()
plt.show()