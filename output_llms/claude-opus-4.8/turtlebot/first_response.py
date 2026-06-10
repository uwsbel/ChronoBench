import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math

# ---------------------------------------------------------------------
#  1. Initialize the PyChrono environment and core components
# ---------------------------------------------------------------------

# Create the Chrono physical system
system = chrono.ChSystemNSC()

# Set the gravitational acceleration (Earth gravity along -Z)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set collision system type (Bullet)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
#  2. Add the required physical systems and objects
# ---------------------------------------------------------------------

# Define a contact material for the ground
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

# Create the rigid terrain (ground body)
ground = chrono.ChBodyEasyBox(
    20, 20, 1,        # dimensions: length, width, height
    1000,             # density
    True,             # create visualization asset
    True,             # enable collision
    ground_mat        # contact material
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   # position below robot
ground.SetFixed(True)                          # fix the ground in place

# Optional: give the ground a texture for better visualization
ground_texture = chrono.ChVisualMaterial()
ground_texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.GetVisualShape(0).SetMaterial(0, ground_texture)

system.Add(ground)

# ---------------------------------------------------------------------
#  3. Initialize the Turtlebot with a position and orientation
# ---------------------------------------------------------------------

# Define the initial position and orientation
init_pos = chrono.ChVector3d(0, 0, 0.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity rotation

# Create the Turtlebot robot
turtlebot = robot.TurtleBot(system, init_pos, init_rot)

# Initialize the robot (creates bodies, joints, motors, etc.)
turtlebot.Initialize()

# ---------------------------------------------------------------------
#  4. Create the Irrlicht visualization
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Set up the camera
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.5),  # camera location
              chrono.ChVector3d(0, 0, 0))       # camera target

# Add lighting
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),  # light position
    chrono.ChVector3d(0, 0, 0),         # target
    3,                                   # radius
    2, 6,                                # near, far clipping
    50                                   # field of view
)
vis.EnableShadows()

# ---------------------------------------------------------------------
#  5. Simulation loop with motor control for left and right turns
# ---------------------------------------------------------------------

time_step = 0.005   # simulation timestep
time = 0.0

print("Starting Turtlebot simulation...")

while vis.Run():
    time = system.GetChTime()

    # ------------- Motor control logic -------------
    # The Turtlebot has two driving wheels: LEFT and RIGHT.
    # By setting different wheel speeds we make the robot turn.

    if time < 1.0:
        # Drive straight forward
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)   # left drive
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)   # right drive

    elif time >= 1.0 and time < 2.0:
        # Turn LEFT (slow down left wheel, speed up right wheel)
        turtlebot.SetMotorSpeed(0, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)

    elif time >= 2.0 and time < 3.0:
        # Turn RIGHT (speed up left wheel, slow down right wheel)
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(0, robot.TurtleBot_WheelID_RD)

    else:
        # Continue driving straight
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_LD)
        turtlebot.SetMotorSpeed(-math.pi, robot.TurtleBot_WheelID_RD)

    # ------------- Render scene -------------
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # ------------- Advance the simulation -------------
    turtlebot.Update()
    system.DoStepDynamics(time_step)

print("Simulation finished.")