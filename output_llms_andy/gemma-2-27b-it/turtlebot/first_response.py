```python
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#
# ---------------------------------------------------------------------

# Create a Chrono system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# ---------------------------------------------------------------------
#
#   Create the Turtlebot robot
#
# ---------------------------------------------------------------------

# Load the Turtlebot model (assuming you have the .obj file)
robot = chrono.ChRobot(system)
robot.ReadFromFile("turtlebot.obj")

# Set initial position and orientation
robot.SetPos(chrono.ChVectorD(0, 0.5, 0))
robot.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 4, chrono.ChVectorD(0, 0, 1)))

# ---------------------------------------------------------------------
#
#   Set up Irrlicht visualization
#
# ---------------------------------------------------------------------

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")

# Set camera position and lighting
camera = vis.GetCamera()
camera.SetPosition(chrono.ChVectorD(5, 5, 3))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChLight(chrono.ChVectorD(2, 5, 2)))

# ---------------------------------------------------------------------
#
#   Simulation loop
#
# ---------------------------------------------------------------------

# Simulation parameters
timestep = 0.01  # Time step

while vis.Run():
    # Control Turtlebot's motors (example)
    if system.GetChTime() < 5:
        robot.SetMotorSpeed(0, 10)  # Left motor
        robot.SetMotorSpeed(1, -10)  # Right motor
    else:
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(1, 0)

    # Advance the simulation
    system.DoStepDynamics(timestep)

    # Render the scene
    vis.Render()