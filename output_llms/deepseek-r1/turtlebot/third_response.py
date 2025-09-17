import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module import

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted Z position to -0.6
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create randomly placed boxes
for i in range(5):
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_mat)
    box_pos = chrono.ChVector3d(
        np.random.uniform(-5, 5),
        np.random.uniform(-5, 5),
        -0.6 + 0.25  # Positioned on top of ground
    )
    box.SetPos(box_pos)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))
    system.Add(box)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 1000.0)

# Configure lidar sensor
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 0, 1)))
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),  # Attach to robot chassis
    10,                      # Scanning rate in Hz
    offset_pose,             # Offset position
    200,                     # Horizontal samples
    100,                     # Vertical channels
    math.radians(360),       # Horizontal FOV
    math.radians(30),        # Vertical FOV
    0.1,                     # Minimum range
    100.0                    # Maximum range
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.1)
lidar.SetCollectionWindow(0.02)

# Add noise model
lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

# Add data access filter
lidar.PushFilter(sens.ChFilterAccessXYZI())

# Add visualization filter
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

# Add sensor to manager
manager.AddSensor(lidar)

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

# Set the simulation time step
time_step = 2e-3

# Wheel ID definitions (moved outside loop)
LEFT_DRIVE_WHEEL = turtlebot.TurtleBot.LEFT_DRIVE_WHEEL
RIGHT_DRIVE_WHEEL = turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL

# Motion control function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)

# Simulation loop
time = 0
while vis.Run():
    # Update sensor manager
    manager.Update()
    
    # Control logic using move function
    if time < 1.0:
        move('straight')
    elif time < 2.0:
        move('left')
    elif time < 3.0:
        move('right')
    else:
        move('straight')

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)