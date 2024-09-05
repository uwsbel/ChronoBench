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
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # Position the rod
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000)  # Create a cylinder along the Y-axis with radius 0.2 meters and height 0.3 meters
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # Position the piston
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Rotate the piston to align it along the X-axis
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))  # Initialize the motor at the crank center, connecting the crank and the floor
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # Set the angular speed of the motor to π rad/s (180°/s)
my_motor.SetMotorFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Initialize the joint
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Initialize the joint
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # Initialize the joint
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

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Set the simulation duration
simulation_duration = 20.0  # seconds
time_step = 1e-3  # seconds
current_time = 0.0

# Run the interactive simulation loop
while vis.Run() and current_time < simulation_duration:
    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    
    # Collect data
    current_angle = mcrank.GetRot().Q_to_Euler123()[2]  # Get the current angle of the crank in radians
    current_pos = mpiston.GetPos().x  # Get the current position of the piston
    current_speed = mrod.GetPos_dt().x  # Get the current speed of the rod along the X-axis
    
    # Append data to arrays
    array_time.append(current_time)
    array_angle.append(current_angle)
    array_pos.append(current_pos)
    array_speed.append(current_speed)
    
    # Advance the simulation by a time step
    sys.DoStepDynamics(time_step)
    current_time += time_step  # Update the current time

# Plotting results
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# First subplot: Position vs. Crank Angle
axs[0].plot(array_angle, array_pos, label='Position [m]', color='b')
axs[0].set_title('Position vs. Crank Angle')
axs[0].set_xlabel('Crank Angle [rad]')
axs[0].set_ylabel('Position [m]')
axs[0].grid()
axs[0].legend()

# Second subplot: Speed vs. Crank Angle
axs[1].plot(array_angle, array_speed, label='Speed [m/s]', color='r')
axs[1].set_title('Speed vs. Crank Angle')
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].grid()
axs[1].legend()

# Adjust x-axis ticks to display π-based intervals
ticks = np.arange(0, 2 * np.pi + np.pi / 2, np.pi / 2)
axs[0].set_xticks(ticks)
axs[0].set_xticklabels([f'{tick/np.pi:.1f}π' for tick in ticks])
axs[1].set_xticks(ticks)
axs[1].set_xticklabels([f'{tick/np.pi:.1f}π' for tick in ticks])

plt.tight_layout()
plt.show()