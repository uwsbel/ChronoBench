import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker  # Required for custom tick formatting

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
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  # Position the rod at (x=-0.6 + 0.75, y=0.5, z=0)
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

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Initialize the joint at (x=-0.6, y=0.5, z=0)
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Initialize the joint at (x=0.9, y=0.5, z=0)
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # Initialize the joint at (x=0.9, y=0.5, z=0) with rotation aligning the Z-axis to the X-axis
sys.Add(mjointC)  # Add the joint to the simulation system

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

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Run the interactive simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds (1 ms)

    # Collect instantaneous data
    current_time = sys.GetChTime()
    array_time.append(current_time)
    array_angle.append(my_motor.GetMotorAngle())  # Get crank angle from motor
    array_pos.append(mpiston.GetPos().x)  # Get piston position along X-axis
    array_speed.append(mpiston.GetPos_dt().x)  # Get piston speed along X-axis

    # Stop simulation after 20 seconds
    if current_time > 20:
        break

# Define custom tick formatter for π-based intervals
def pi_formatter(x, pos):
    if x == 0:
        return "0"
    elif x == np.pi:
        return "π"
    elif x == 2*np.pi:
        return "2π"
    elif x == np.pi/2:
        return "π/2"
    elif x == 3*np.pi/2:
        return "3π/2"
    else:
        return f"{x/np.pi:.1f}π"

formatter = ticker.FuncFormatter(pi_formatter)

# Create subplots for position and speed vs crank angle
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Position vs crank angle
ax1.plot(array_angle, array_pos, 'b-', label='Piston Position')
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.xaxis.set_major_formatter(formatter)
ax1.xaxis.set_major_locator(ticker.MultipleLocator(base=np.pi/2))
ax1.grid(True)
ax1.legend()

# Speed vs crank angle
ax2.plot(array_angle, array_speed, 'r-', label='Piston Speed')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.xaxis.set_major_formatter(formatter)
ax2.xaxis.set_major_locator(ticker.MultipleLocator(base=np.pi/2))
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()