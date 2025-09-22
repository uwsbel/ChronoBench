import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono simulation
system = chrono.ChSystemNSC()

# Set the simulation time step and output interval
time_step = 1e-3
out_step = 1e-2

# Define the dimensions and positions of the components to be modeled
hlength = 1.0  # It seems to represent half the length of some key component(s)
hh = 2 * hlength  # Double hlength for certain dimensions
hw = chrono.ChVector3d(0, hlength, 0)  # A 3D vector, possibly used for positioning
hd = chrono.ChVector3d(0, -hlength, 0)  # Another 3D vector for positioning

# Define the crankshaft dimensions and motor properties
crankrad = 0.4  # Radius of the crankshaft
cranklen = 2.2  # Length of the crankshaft
motor_angle = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial angle of the motor (quaternion)

# Define the rod/piston characteristics
rod_w = 0.4  # Width of the connecting rod
rod_h = 3.0  # Height of the connecting rod
rod_W = 1.0  # Another dimension of the connecting rod (possibly thickness)
crank2 = 1.5  # Distance between crank centers (?)
piston_rad = 0.6  # Radius of the piston
piston_len = 0.4  # Length of the piston

# Calculate the center positions
center_ground = chrono.ChVector3d(0, -1.1, 0)  # Center of the crank-to-rod connection (?)
crank_center = chrono.ChVector3d(0, -1.1, 0)  # Center of the crank
rod_center = chrono.ChVector3d(0, 1, 0)  # Center of the rod
piston_center = chrono.ChVector3d(0, 2, 0)  # Center of the piston

# Create a truss floor
truss_floor = chrono.ChBodyEasyBox(10, 1, 10, 1000)  # Define a box for the truss floor
truss_floor.SetPos(chrono.ChVector3d(0, -7, 0))  # Position the truss floor
truss_floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # Set texture for the floor
system.Add(truss_floor)  # Add the truss floor to the simulation system

# Create a crank
crank = chrono.ChBodyEasyCylinder(crankrad, cranklen, 1000)  # Define a cylinder for the crank
crank.SetPos(crank_center + hw)  # Position the crank
crank.SetRot(chrono.Q_FROM_AXIS_ANGLE_Y, chrono.CH_PI_2)  # Rotate the crank
crank.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # Set texture for the crank
system.Add(crank)  # Add the crank to the simulation system

# Create a rod
rod_casing = chrono.ChBodyEasyBox(rod_w, rod_h, rod_W, 1000)  # Define a box for the rod casing
rod_casing.SetPos(rod_center + hw)  # Position the rod casing
rod_casing.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/brick.jpg"))  # Set texture for the rod casing
system.Add(rod_casing)  # Add the rod casing to the simulation system

# Create a piston
piston = chrono.ChBodyEasyCylinder(piston_rad, piston_len, 1000)  # Define a cylinder for the piston
piston.SetPos(piston_center + hw)  # Position the piston
piston.SetRot(chrono.Q_FROM_AXIS_ANGLE_Y, chrono.CH_PI_2)  # Rotate the piston
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/pink.png"))  # Set texture for the piston
system.Add(piston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()  # Define a motor with rotation speed
motor_frames = chrono.ChFramesys(
    (crank_center + hd + hw).GetX(),
    (crank_center + hd + hw).GetY(),
    (crank_center + hd + hw).GetZ(),
    1, 0, 0, 0)  # Define the frame system for the motor
motor.Initialize(crank, truss_floor, motor_frames)  # Initialize the motor with the crank and truss floor
speed_fun = chrono.ChFunctionConst(chrono.CH_PI)  # Define a constant speed function for the motor
motor.SetSpeedFunction(speed_fun)  # Set the speed function for the motor
system.Add(motor)  # Add the motor to the simulation system

# Connect crank and truss with a bearing
crank_bearing = chrono.ChLinkLockRevolute()  # Define a revolute (hinge) joint
crank_bearing.Initialize(crank, truss_floor, chrono.ChFramed(
    crank_center + hw))  # Initialize the bearing with the crank and truss floor
system.AddLink(crank_bearing)  # Add the bearing to the system

# Connect rod and crank with a bearing
rod_crank_bearing = chrono.ChLinkLockRevolute()  # Define a revolute joint
rod_crank_bearing.Initialize(
    rod_casing, crank, chrono.ChFramed(crank_center + chrono.VNULL + hw))  # Initialize the bearing
system.AddLink(rod_crank_bearing)  # Add the bearing to the system

# Connect rod and piston with a bearing
rod_piston_bearing = chrono.ChLinkLockRevolute()  # Define a revolute joint
rod_piston_bearing.Initialize(
    rod_casing, piston, chrono.ChFramed(piston_center + chrono.VNULL + hw))  # Initialize the bearing
system.AddLink(rod_piston_bearing)  # Add the bearing to the system

# Connect truss and piston with a sliding bearing
truss_piston_bearing = chrono.ChLinkLockPrismatic()  # Define a prismatic (sliding) joint
truss_piston_bearing.Initialize(
    piston, truss_floor, chrono.ChFramed(piston_center + hw, chrono.Q_FROM_AXIS_ANGLE_X, 90 * chrono.CH_DEG_TO_RAD))  # Initialize the bearing
system.AddLink(truss_piston_bearing)  # Add the bearing to the system

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  # Attach the Chrono system to the visualization
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)  # Set the camera direction
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('ChLinkMotorRotationSpeed demo')  # Set the window title
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(3, 13, 8), chrono.ChVector3d(0, 3, 0))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights for better visualization

# Run the simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time

    # Output data at specified intervals
    if time > out_time:
        time_str = f'{time:0.3f}'  # Format the simulation time
        print(f'             time = {time_str} s')
        out_time += out_step  # Update the output time

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)