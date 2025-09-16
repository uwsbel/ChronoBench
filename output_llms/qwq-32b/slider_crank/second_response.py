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
mfloor.SetFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))  # Position the crank at (x=-1, y=0.5, z=-0.1)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Calculate crank's end position along its length (Z-axis)
crank_end = crank_center + chrono.ChVectorD(0, 0, crank_thick / 2)

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
rod_center = crank_end + chrono.ChVectorD(rod_length / 2, 0, 0)
mrod.SetPos(rod_center)  # Position the rod at the midpoint between crank end and piston
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
piston_pos = crank_end + chrono.ChVectorD(rod_length, 0, 0)
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # Create a cylinder along the Y-axis with radius 0.2 meters and height 0.3 meters
mpiston.SetPos(piston_pos)  # Position the piston at the end of the rod
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
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_end))  # Initialize the joint at the crank end
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(piston_pos))  # Initialize the joint at the piston position
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(piston_pos, chrono.Q_ROTATE_Z_TO_X))  # Initialize the joint at piston position with rotation aligning Z-axis to X-axis
sys.Add(mjointC)  # Add the joint to the simulation system

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

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

# Run the interactive simulation loop
while vis.Run():
    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    
    current_time = sys.GetChTime()
    array_time.append(current_time)
    array_angle.append(chrono.CH_PI * current_time)
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    
    if current_time >= 20:
        break
    
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds (1 ms)

# Plotting using Matplotlib
plt.figure(figsize=(12, 8))

# First subplot: Position vs Crank Angle
plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, label='Position')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.grid(True)

# Second subplot: Speed vs Crank Angle
plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, label='Speed')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.grid(True)

plt.tight_layout()
plt.show()