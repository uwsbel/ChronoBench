import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math

# 1. Initialize the Chrono system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Add a rigid ground body
ground = chrono.ChBodyEasyBox(5, 5, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.05))
ground.SetBodyFixed(True)
system.Add(ground)

# Optional: Give the ground a texture
texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("concrete.jpg"))
ground.AddAsset(texture)

# 3. Initialize the Turtlebot
# Set initial position and orientation
turtlebot_pos = chrono.ChVectorD(0, 0, 0.1)  # 10cm above ground
turtlebot_yaw = math.radians(0)  # Facing along +X
turtlebot_rot = chrono.Q_from_AngZ(turtlebot_yaw)

# Create the Turtlebot (using PyChrono's robot module)
turtlebot = robot.Turtlebot()
turtlebot.Initialize(system, turtlebot_pos, turtlebot_rot)

# 4. Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 0.7), chrono.ChVectorD(0, 0, 0.1))
vis.AddTypicalLights()

# 5. Simulation parameters
timestep = 0.01
end_time = 10.0

# Motor control parameters
left_speed = 0.0
right_speed = 0.0
max_speed = 5.0  # rad/s

# 6. Simulation loop
time = 0.0
while vis.Run() and time < end_time:
    # Control logic: turn left for 2s, go straight for 2s, turn right for 2s, repeat
    cycle = int(time // 2) % 3
    if cycle == 0:
        # Turn left
        left_speed = 1.0
        right_speed = 3.0
    elif cycle == 1:
        # Go straight
        left_speed = 2.0
        right_speed = 2.0
    else:
        # Turn right
        left_speed = 3.0
        right_speed = 1.0

    # Clamp speeds
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    # Apply motor speeds
    turtlebot.SetMotorSpeeds(left_speed, right_speed)

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(timestep)
    time += timestep

vis.GetDevice().closeDevice()