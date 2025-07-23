import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1.2, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
try:
    import pychrono.robot as turtlebot
    robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
    robot.Initialize()  # Initialize the robot
except AttributeError:
    print("TurtleBot class or its methods are not available.")

# Create boxes with random positions
box_mat = chrono.ChContactMaterialNSC()
for _ in range(5):
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5)
    box = chrono.ChBodyEasyBox(0.5, 0.5, 1, 1000, True, True, box_mat)
    box.SetPos(box_pos)
    system.Add(box)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Create a sensor manager
manager = sens.ChSensorManager(system)
# Configure and add a lidar sensor
lidar = sens.ChLidarSensor(
    robot.body, 
    10, 
    chrono.ChFrame(chrono.ChVector3d(0, 0, 0.5), chrono.Q_from_AngAxis(0, chrono.VECT_Z)), 
    100, 
    30, 
    chrono.ChVector3d(0.5, 0.1, 0.01)
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAreaReconstruction())
lidar.PushFilter(sens.ChFilterVisualize(512, 512, "Lidar Output"))
manager.AddSensor(lidar)

# Define motion control function for Turtlebot
def move(mode):
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    speed = math.pi  # Example speed
    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    move('straight')  # Control Turtlebot's movement
    
    # Update sensor manager
    manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)