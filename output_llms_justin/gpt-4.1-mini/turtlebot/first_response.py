import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize the PyChrono environment and core components.

# Create a Chrono physical system
system = chrono.ChSystemNSC()

# Set gravity (typical Earth gravity in negative Z axis)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Add the required physical systems and objects as specified.

# Create a rigid ground body (terrain)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
# Create a horizontal ground plane at z=0
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(10, 10, 0.1)  # Large flat box as ground
ground.AddAsset(ground_shape)
# Add color asset (brownish)
ground_color = chrono.ChColorAsset()
ground_color.SetColor(chrono.ChColor(0.4, 0.25, 0))
ground.AddAsset(ground_color)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))  # slightly below zero to make top surface z=0
system.Add(ground)

# Initialize the Turtlebot robot.
# PyChrono provides sample robots in pychrono.robot module (e.g. Turtlebot)
# We will assume you have pychrono.robot.Turtlebot available.

try:
    import pychrono.robot
except ImportError:
    raise ImportError("pychrono.robot module is required for Turtlebot but is not found.")

# Create Turtlebot
turtlebot = pychrono.robot.Turtlebot()
# Set initial position and orientation
initial_pos = chrono.ChVectorD(0, 0, 0)  # on ground plane
# Set orientation: say facing along Y axis (chrono default forward in X, so rotate 90 deg about Z)
initial_rot = chrono.ChQuaternionD()
initial_rot.Q_from_AngAxis(math.radians(90), chrono.ChVectorD(0,0,1))
turtlebot.SetPos(initial_pos)
turtlebot.SetRot(initial_rot)

# Add Turtlebot to system
turtlebot.GetSystem().Add(turtlebot.GetBody())

# 3. Set necessary default parameters such as positions, forces, and interactions.

# The Turtlebot has two motors controlling left and right wheels.
# We will simulate control inputs that command the motors to turn left or right at specified times.

# Setup Irrlicht visualization
app = chronoirr.ChIrrApp(system, "Turtlebot on Terrain", chronoirr.dimension2du(1024,768))

# Add typical elements
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(3, -5, 2), chronoirr.vector3df(0, 0, 0.2))  # position camera behind and above robot, looking at robot
app.AssetBindAll()
app.AssetUpdateAll()

# Simulation parameters
time_step = 0.01  # 10 ms timestep
app.SetTimestep(time_step)

# Simulation control times
turn_left_start = 2.0   # start turning left at 2 seconds
turn_left_end   = 4.0   # stop turning left at 4 seconds
turn_right_start = 6.0  # start turning right at 6 seconds
turn_right_end   = 8.0  # stop turning right at 8 seconds

# Motor torque or velocity command (adjust as needed)
motor_speed = 2.0  # rad/s or rad/s equivalent speed command

# Helper function to drive motors: left_speed and right_speed (rad/s)
def set_wheel_speeds(left_speed, right_speed):
    # Turtlebot internally uses ChMotorTorque or ChLinkMotorRotationAngle.
    m_left = turtlebot.GetMotorLeft()
    m_right = turtlebot.GetMotorRight()
    # Using SetSpeed - Turtlebot's motors are typically speed controlled
    m_left.SetSpeed(left_speed)
    m_right.SetSpeed(right_speed)

# Initially stop the robot
set_wheel_speeds(0, 0)

# Main simulation loop
while app.GetDevice().run():
    current_time = system.GetChTime()

    # Control logic for turning:
    if turn_left_start <= current_time < turn_left_end:
        # Turn left: left wheel slower or reverse compared to right wheel
        set_wheel_speeds(motor_speed*0.5, motor_speed)
    elif turn_right_start <= current_time < turn_right_end:
        # Turn right: right wheel slower or reverse compared to left wheel
        set_wheel_speeds(motor_speed, motor_speed*0.5)
    else:
        # Straight/no motion
        set_wheel_speeds(motor_speed, motor_speed)

    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    system.DoStepDynamics(time_step)