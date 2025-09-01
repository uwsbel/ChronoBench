import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
system = chrono.ChSystemSMC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

# Create the visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 3, 10))
vis.AddTypicalLights()

# Add ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3D(0, 0, 0))
ground.SetMass(0)
ground.SetFixed(True)
ground.EnableCollision(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3D(10, 10, 1))
ground.AddVisualShape(ground_shape)
system.Add(ground)

# Initialize Turtlebot
turtlebot = chrono.ChBody()
turtlebot.SetPos(chrono.ChVector3D(0, 0, 0.5))  # Position above ground
turtlebot.SetMass(1.0)
turtlebot.SetInertiaXX(chrono.ChVector3D(0.1, 0.1, 0.1))
system.Add(turtlebot)

# Add wheels
wheel_left = chrono.ChBody()
wheel_left.SetPos(chrono.ChVector3D(-0.1, 0, 0))
wheel_left.SetMass(0.1)
wheel_left.SetInertiaXX(chrono.ChVector3D(0.01, 0.01, 0.01))
system.Add(wheel_left)

wheel_right = chrono.ChBody()
wheel_right.SetPos(chrono.ChVector3D(0.1, 0, 0))
wheel_right.SetMass(0.1)
wheel_right.SetInertiaXX(chrono.ChVector3D(0.01, 0.01, 0.01))
system.Add(wheel_right)

# Create revolute joints for wheels
joint_left = chrono.ChLinkLockRevolute()
joint_left.Initialize(turtlebot, wheel_left, chrono.ChCoordsysD(chrono.ChVector3D(-0.1, 0, 0), chrono.QUNIT))
system.Add(joint_left)

joint_right = chrono.ChLinkLockRevolute()
joint_right.Initialize(turtlebot, wheel_right, chrono.ChCoordsysD(chrono.ChVector3D(0.1, 0, 0), chrono.QUNIT))
system.Add(joint_right)

# Add motors to wheels
motor_left = chrono.ChLinkMotorRotationSpeed()
motor_left.Initialize(turtlebot, wheel_left, chrono.ChCoordsysD(chrono.ChVector3D(-0.1, 0, 0), chrono.QUNIT))
system.Add(motor_left)

motor_right = chrono.ChLinkMotorRotationSpeed()
motor_right.Initialize(turtlebot, wheel_right, chrono.ChCoordsysD(chrono.ChVector3D(0.1, 0, 0), chrono.QUNIT))
system.Add(motor_right)

# Visualization for Turtlebot and wheels
turtlebot_shape = chrono.ChVisualShapeBox(chrono.ChVector3D(0.2, 0.2, 0.2))
turtlebot.AddVisualShape(turtlebot_shape)

wheel_shape = chrono.ChVisualShapeCylinder(0.05, 0.1)
wheel_left.AddVisualShape(wheel_shape)
wheel_right.AddVisualShape(wheel_shape)

# Simulation loop
time_step = 0.01
simulation_time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Control logic
    current_time = simulation_time
    if 1 <= current_time < 2:
        # Turn left
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(-1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    elif 2 <= current_time < 3:
        # Turn right
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(-1.0))
    else:
        # Move forward
        motor_left.SetSpeedFunction(chrono.ChFunction_Const(1.0))
        motor_right.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step

# Close the visualization window
vis.Close()