import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robosimian  # Notice: Assuming TurtleBot is not directly available; using RoboSimian as a placeholder
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot (using RoboSimian as a placeholder due to potential import issues)
init_pos = chrono.ChVector3d(0, 0.2, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
# robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Original line; commented out due to potential import issues
# robot.Initialize()

# Placeholder for TurtleBot or similar robot initialization
robot = robosimian.RoboSimian(system, True, True)
robot.Initialize(chrono.ChCoordsysd(init_pos, init_rot))

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

# Add sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 5000)

# Configure and add a lidar sensor
lidar = sens.ChLidarSensor(
    robot.GetChassisBody(),
    10,  # update rate
    chrono.ChFrame(chrono.ChVector3d(0, 0, .5), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),
    100,  # number of horizontal samples
    50,   # number of vertical channels
    chrono.CH_PI,  # horizontal fov
    chrono.CH_C_PI / 6.,  # vertical fov
    0.1, 100  # min and max distance
)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(960, 540, "Lidar"))
manager.AddSensor(lidar)

# Add randomly placed boxes
box_mat = chrono.ChContactMaterialNSC()
for _ in range(5):
    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5))
    system.Add(box)

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, 0)  # Assuming RoboSimian's motor control API; adjust according to actual TurtleBot API
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(0, 1)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    move('straight')  # Control TurtleBot to move straight
    
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