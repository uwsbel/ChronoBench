import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import numpy as np  # Import numpy for numerical operations

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters) (This is the cylinder's height)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # Create a box with dimensions 3x1x3 meters and density 1000 kg/m^3
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis
# *** Corrected Crank Position ***
# Original: mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
# Changed to mcrank.SetPos(crank_center). This places the crank's geometric center
# at crank_center, ensuring it rotates about this center and that the crank-rod joint
# is in the crank's median plane, consistent with a planar mechanism setup where
# rod and piston are in the z=0 plane (relative to crank_center.z).
mcrank.SetPos(crank_center)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank so its axis of symmetry (originally Y) aligns with global Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box
# Position the rod: Its center is initially aligned along the X-axis.
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # Create a cylinder (height 0.3 along Y)
# Position the piston: Its center is initially aligned along the X-axis.
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Rotate piston so its axis (originally Y) aligns with global X-axis
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
# Initialize motor between crank and floor, rotating around Z-axis at crank_center
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed_func = chrono.ChFunctionConst(chrono.CH_PI)  # Angular speed function (π rad/s)
my_motor.SetMotorFunction(my_angularspeed_func)  # Assign speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
# Initialize joint at the crank pin location
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
# Initialize joint at the piston center (coincident with one end of the rod)
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
# Initialize joint at piston center. Q_ROTATE_Z_TO_X aligns the prismatic joint's Z-axis (sliding axis) with global X.
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)  # Add the joint to the simulation system

# --- Instruction 3: Array Initialization for Plotting ---
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo
vis.AddSkyBox()  # Add a skybox
# Camera target y-coordinate changed from 1 to 0.5 for a more centered view of the mechanism
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()  # Add typical lights

# Run the interactive simulation loop
simulation_time_limit = 20.0  # seconds for simulation duration

while vis.Run():
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # --- Instruction 4: Collect Data During Simulation ---
    current_time = sys.GetChTime()
    array_time.append(current_time)
    
    # Crank angle from the motor (continuously increasing)
    crank_angle = my_motor.GetMotorRot()
    array_angle.append(crank_angle)
    
    # Piston position (X-coordinate)
    piston_pos_x = mpiston.GetPos().x
    array_pos.append(piston_pos_x)
    
    # Piston speed (X-component)
    piston_speed_x = mpiston.GetPosDt().x
    array_speed.append(piston_speed_x)

    # --- Instruction 5: Conditional to Stop Simulation ---
    if current_time >= simulation_time_limit:
        break

# Ensure Irrlicht window closes if loop broken by time limit rather than user closing it
if vis.Run():
    vis.GetDevice().closeDevice()

# --- Instruction 6: Matplotlib Plotting ---

# Custom formatter function for Pi-based x-axis ticks
def format_func_pi_ticks(value, tick_number):
    N = int(np.round(2 * value / np.pi)) # Number of pi/2 units
    if N == 0:
        return "0"
    elif N == 1:
        return r"$\pi/2$"
    elif N == -1:
        return r"$-\pi/2$"
    elif N == 2:
        return r"$\pi$"
    elif N == -2:
        return r"$-\pi$"
    elif N % 2 == 0:  # Even N, means multiple of pi
        return r"${}\pi$".format(N // 2)
    else:  # Odd N, means multiple of pi/2
        return r"${}\pi/2$".format(N)

plt.figure(figsize=(12, 8))

# First subplot: Piston Position [m] vs. Crank Angle [rad]
plt.subplot(2, 1, 1)
if array_angle: # Check if data was collected
    plt.plot(array_angle, array_pos)
    # Apply Pi-based ticks using numpy for tick locations
    ax1 = plt.gca()
    ax1.xaxis.set_major_locator(plt.MultipleLocator(base=np.pi / 2))
    ax1.xaxis.set_major_formatter(plt.FuncFormatter(format_func_pi_ticks))
    plt.xlim([min(array_angle), max(array_angle)])
else:
    plt.text(0.5, 0.5, "No data collected for plotting.", horizontalalignment='center', verticalalignment='center')
plt.title('Piston Position vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')
plt.grid(True)


# Second subplot: Piston Speed [m/s] vs. Crank Angle [rad]
plt.subplot(2, 1, 2)
if array_angle: # Check if data was collected
    plt.plot(array_angle, array_speed, color='orange')
    # Apply Pi-based ticks using numpy for tick locations
    ax2 = plt.gca()
    ax2.xaxis.set_major_locator(plt.MultipleLocator(base=np.pi / 2))
    ax2.xaxis.set_major_formatter(plt.FuncFormatter(format_func_pi_ticks))
    plt.xlim([min(array_angle), max(array_angle)])
else:
    plt.text(0.5, 0.5, "No data collected for plotting.", horizontalalignment='center', verticalalignment='center')
plt.title('Piston Speed vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')
plt.grid(True)

plt.tight_layout()  # Adjust subplot params for a tight layout
plt.show()  # Display the plots